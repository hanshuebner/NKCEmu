/***************************************************************************
 *  SER2 R2 (GXE Electronics) emulation for the NKC emulator.
 *
 *  The card carries an XR88C681 / SCN2681-type dual UART (DUART) decoded at
 *  base 0x90 (ports 0x90..0x9F, 16 registers).  Two physical channels, A and
 *  B, are each exposed on the host as a TCP listening socket: connect with
 *  e.g. `nc 127.0.0.1 2681` and you are wired to the NKC's serial port.
 *
 *  Buffering: while nothing is connected, bytes the NKC transmits are held in
 *  a buffer (up to SER2_TXCAP = 1 MiB) so output produced right after the
 *  emulator starts is not lost before you manage to connect; on connect the
 *  backlog is flushed.  Bytes arriving from a connected client are buffered
 *  (SER2_RXCAP) until the guest reads them.
 *
 *  Interrupts: the DUART's INTRN is modelled as wired to the Z80 /INT line
 *  (the SER-INT jumper, assumed installed in the emulator).  When an unmasked
 *  condition in ISR is active we raise INT_INT; the guest's IM 1 handler then
 *  vectors to 0x0038.  This requires WANT_INT in sim.h.
 *
 *  Register-level behaviour is faithful enough to run SER2.BAS (counter
 *  liveness, MR read-back, IVR scratch, local-loopback TX->RX, and the
 *  ISR/IMR interrupt-logic test) as well as a real interrupt-driven driver.
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include "sim.h"
#include "simglb.h"

#define SER2_TXCAP  (1u << 20)      /* 1 MiB pre-connection output buffer   */
#define SER2_RXCAP  (1u << 16)      /* 64 KiB inbound buffer                */
#define SER2_PORT_A 2681            /* default TCP ports (chip mnemonic)    */
#define SER2_PORT_B 2682

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0              /* macOS: rely on SO_NOSIGPIPE + SIG_IGN */
#endif

/* --- a simple byte ring; push drops the NEWEST byte when full, so the
       earliest output (e.g. a boot banner) is preserved --------------- */
typedef struct {
    unsigned char *buf;
    int cap, head, tail, count;
    long dropped;
} ring_t;

static void ring_init(ring_t *r, int cap)
{
    r->buf = malloc(cap);
    r->cap = cap;
    r->head = r->tail = r->count = 0;
    r->dropped = 0;
}
static int ring_push(ring_t *r, unsigned char b)
{
    if (r->count >= r->cap) { r->dropped++; return 0; }
    r->buf[r->head] = b;
    r->head = (r->head + 1) % r->cap;
    r->count++;
    return 1;
}
static int ring_pop(ring_t *r, unsigned char *b)
{
    if (r->count == 0) return 0;
    *b = r->buf[r->tail];
    r->tail = (r->tail + 1) % r->cap;
    r->count--;
    return 1;
}
static void ring_clear(ring_t *r) { r->head = r->tail = r->count = 0; }

/* --- per-channel state --------------------------------------------------- */
typedef struct {
    int port;                       /* TCP port                              */
    int lfd, cfd;                   /* listen fd, connected fd (-1 = none)   */
    int warned;                     /* one-shot overflow warning             */
    ring_t tx;                      /* guest -> client                       */
    ring_t rx;                      /* client -> guest                       */
    unsigned char mr[2];            /* MR1, MR2                              */
    int mrptr;                      /* mode-register pointer (0 or 1)        */
    unsigned char csr;              /* clock-select (baud); not interpreted  */
    int rx_en, tx_en;              /* receiver / transmitter enabled         */
} chan_t;

static chan_t ch[2];

/* --- chip-wide state ----------------------------------------------------- */
static unsigned char acr;           /* aux control register                 */
static unsigned char imr;           /* interrupt mask                       */
static unsigned char ivr = 0x0F;    /* interrupt vector (reset default)     */
static unsigned char opcr;          /* output-port config                   */
static unsigned int  ctr_preset = 0xFFFF;  /* counter/timer preset          */
static int           ctr_running;
static long          ctr_start_us;
static unsigned int  ctr_phase;     /* guarantees consecutive reads differ  */

static int ser2_started;

static long host_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000000L + tv.tv_usec;
}

/* MR2[7:6] == 10 selects local loopback (TX feeds RX on-chip) */
static int loopback(const chan_t *c) { return (c->mr[1] & 0xC0) == 0x80; }

/* ------------------------------------------------------------------------- */
/*  DUART register model                                                     */
/* ------------------------------------------------------------------------- */

static unsigned int counter_value(void)
{
    unsigned int span = ctr_preset ? ctr_preset + 1u : 0x10000u;
    unsigned long ticks = ctr_phase;
    if (ctr_running) {
        /* X1/16 of a 3.6864 MHz crystal ~= 230400 ticks/s = 0.2304/us */
        long el = host_us() - ctr_start_us;
        ticks += (unsigned long)((double)el * 0.2304);
    }
    return (unsigned int)((ctr_preset - (ticks % span)) & 0xFFFF);
}

static unsigned char status(const chan_t *c)
{
    unsigned char s = 0;
    if (c->rx.count > 0)  s |= 0x01;            /* RxRDY                     */
    if (c->rx.count >= 3) s |= 0x02;            /* FFULL (approx 3-deep FIFO)*/
    if (c->tx_en)         s |= 0x04;            /* TxRDY (host buffer absorbs)*/
    if (c->tx.count == 0) s |= 0x08;            /* TxEMT                     */
    return s;
}

static unsigned char compute_isr(void)
{
    unsigned char i = 0;
    if (status(&ch[0]) & 0x04) i |= 0x01;       /* TxRDYA                   */
    if (status(&ch[0]) & 0x01) i |= 0x02;       /* RxRDY/FFULL A            */
    if (status(&ch[1]) & 0x04) i |= 0x10;       /* TxRDYB                   */
    if (status(&ch[1]) & 0x01) i |= 0x20;       /* RxRDY/FFULL B            */
    return i;
}

/* Raise or drop the emulated INTRN -> Z80 /INT line.  We own INT_INT; only
   ever flip it between NONE and INT (never touch a pending NMI). */
static void ser2_update_irq(void)
{
    int active = (compute_isr() & imr) != 0;
    if (active) {
        if (int_type == INT_NONE) int_type = INT_INT;
    } else {
        if (int_type == INT_INT) int_type = INT_NONE;
    }
}

static unsigned char read_mr(chan_t *c)
{
    unsigned char v = c->mr[c->mrptr];
    if (c->mrptr == 0) c->mrptr = 1;
    return v;
}
static void write_mr(chan_t *c, unsigned char v)
{
    c->mr[c->mrptr] = v;
    if (c->mrptr == 0) c->mrptr = 1;
}

static unsigned char read_rhr(chan_t *c)
{
    unsigned char b = 0;
    ring_pop(&c->rx, &b);
    return b;
}

static void write_thr(chan_t *c, unsigned char v)
{
    if (loopback(c)) {
        ring_push(&c->rx, v);                   /* TX -> own RX, on-chip    */
    } else {
        if (!ring_push(&c->tx, v) && !c->warned) {
            fprintf(stderr, "ser2: TX buffer full on port %d "
                    "(>%u bytes unconsumed); dropping\n", c->port, SER2_TXCAP);
            c->warned = 1;
        }
    }
}

static void command(chan_t *c, unsigned char v)
{
    switch (v & 0x03) {                         /* receiver enable field    */
    case 1: c->rx_en = 1; break;
    case 2: c->rx_en = 0; break;
    }
    switch ((v >> 2) & 0x03) {                  /* transmitter enable field */
    case 1: c->tx_en = 1; break;
    case 2: c->tx_en = 0; break;
    }
    switch ((v >> 4) & 0x07) {                  /* miscellaneous command    */
    case 1: c->mrptr = 0;       break;          /* reset MR pointer         */
    case 2: ring_clear(&c->rx); break;          /* reset receiver           */
    case 3: /* reset transmitter: keep the host TX backlog intentionally */
            break;
    case 4: /* reset error status: no errors are modelled */
            break;
    }
}

static BYTE ser2_read(int reg)
{
    BYTE v = 0;
    switch (reg) {
    case 0:  v = read_mr(&ch[0]);            break;
    case 1:  v = status(&ch[0]);             break;
    case 2:  v = 0;                          break;  /* BRG test          */
    case 3:  v = read_rhr(&ch[0]);           break;  /* RHRA              */
    case 4:  v = 0;                          break;  /* IPCR              */
    case 5:  v = compute_isr();              break;  /* ISR               */
    case 6:  ctr_phase++; v = counter_value() >> 8;  break;  /* CTU       */
    case 7:  ctr_phase++; v = counter_value() & 0xFF; break; /* CTL       */
    case 8:  v = read_mr(&ch[1]);            break;
    case 9:  v = status(&ch[1]);             break;
    case 10: v = 0;                          break;  /* 1x/16x test       */
    case 11: v = read_rhr(&ch[1]);           break;  /* RHRB              */
    case 12: v = ivr;                        break;
    case 13: v = 0xFF;                       break;  /* input port        */
    case 14: ctr_running = 1;                        /* start counter cmd */
             ctr_start_us = host_us(); ctr_phase = 0; v = 0xFF; break;
    case 15: ctr_running = 0; v = 0xFF;      break;  /* stop counter cmd  */
    }
    ser2_update_irq();
    return v;
}

static void ser2_write(int reg, BYTE d)
{
    switch (reg) {
    case 0:  write_mr(&ch[0], d);                              break;
    case 1:  ch[0].csr = d;                                    break;
    case 2:  command(&ch[0], d);                               break;
    case 3:  write_thr(&ch[0], d);                             break;
    case 4:  acr = d;                                          break;
    case 5:  imr = d;                                          break;
    case 6:  ctr_preset = (ctr_preset & 0x00FF) | (d << 8);    break;
    case 7:  ctr_preset = (ctr_preset & 0xFF00) | d;           break;
    case 8:  write_mr(&ch[1], d);                              break;
    case 9:  ch[1].csr = d;                                    break;
    case 10: command(&ch[1], d);                               break;
    case 11: write_thr(&ch[1], d);                             break;
    case 12: ivr = d;                                          break;
    case 13: opcr = d;                                         break;
    case 14: /* set output-port bits   */                      break;
    case 15: /* reset output-port bits */                      break;
    }
    ser2_update_irq();
}

/* ------------------------------------------------------------------------- */
/*  Sockets                                                                  */
/* ------------------------------------------------------------------------- */

static void set_nonblock(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static int open_listen(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("ser2: socket"); return -1; }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) {
        fprintf(stderr, "ser2: bind port %d: %s\n", port, strerror(errno));
        close(fd);
        return -1;
    }
    listen(fd, 1);
    set_nonblock(fd);
    return fd;
}

static void chan_service(chan_t *c)
{
    /* accept one client if none is attached */
    if (c->cfd < 0 && c->lfd >= 0) {
        int fd = accept(c->lfd, NULL, NULL);
        if (fd >= 0) {
            set_nonblock(fd);
            int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
            setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
            c->cfd = fd;
            fprintf(stderr, "ser2: client connected on port %d\n", c->port);
        }
    }
    if (c->cfd < 0) return;

    /* drain inbound socket -> RX ring (ignored while in local loopback) */
    unsigned char tmp[4096];
    ssize_t n = recv(c->cfd, tmp, sizeof tmp, 0);
    if (n > 0) {
        if (!loopback(c))
            for (ssize_t i = 0; i < n; i++) ring_push(&c->rx, tmp[i]);
    } else if (n == 0) {
        close(c->cfd); c->cfd = -1;
        fprintf(stderr, "ser2: client on port %d disconnected\n", c->port);
        return;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        close(c->cfd); c->cfd = -1;
        return;
    }

    /* flush TX ring -> socket in contiguous chunks */
    while (c->tx.count > 0) {
        int chunk = (c->tx.tail < c->tx.head)
                    ? (c->tx.head - c->tx.tail)
                    : (c->tx.cap - c->tx.tail);
        if (chunk > c->tx.count) chunk = c->tx.count;
        ssize_t w = send(c->cfd, c->tx.buf + c->tx.tail, chunk, MSG_NOSIGNAL);
        if (w > 0) {
            c->tx.tail = (c->tx.tail + w) % c->tx.cap;
            c->tx.count -= w;
            c->warned = 0;
        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        } else {
            close(c->cfd); c->cfd = -1;
            break;
        }
    }
}

/* Called periodically from the CPU loop (see sim1.c). */
void ser2_service(void)
{
    if (!ser2_started) return;
    chan_service(&ch[0]);
    chan_service(&ch[1]);
    ser2_update_irq();
}

void ser2_init(void)
{
    signal(SIGPIPE, SIG_IGN);

    const char *ea = getenv("NKCEMU_SER2_PORTA");
    const char *eb = getenv("NKCEMU_SER2_PORTB");
    int pa = ea ? atoi(ea) : SER2_PORT_A;
    int pb = eb ? atoi(eb) : SER2_PORT_B;

    memset(ch, 0, sizeof ch);
    ch[0].port = pa; ch[1].port = pb;
    for (int i = 0; i < 2; i++) {
        ch[i].cfd = -1;
        ring_init(&ch[i].tx, SER2_TXCAP);
        ring_init(&ch[i].rx, SER2_RXCAP);
        ch[i].mr[0] = ch[i].mr[1] = 0;
        ch[i].mrptr = 0;
    }
    ch[0].lfd = open_listen(pa);
    ch[1].lfd = open_listen(pb);
    ser2_started = 1;

    fprintf(stderr, "ser2: 88C681 DUART at port 0x90; "
            "channel A on 127.0.0.1:%d, channel B on 127.0.0.1:%d\n", pa, pb);
}

/* ------------------------------------------------------------------------- */
/*  Per-port I/O wrappers (iosim dispatches one zero-arg fn per port)        */
/* ------------------------------------------------------------------------- */

#define WRAP(n) \
    static BYTE in_##n(void)       { return ser2_read(n); } \
    static void out_##n(BYTE d)    { ser2_write(n, d); }
WRAP(0)  WRAP(1)  WRAP(2)  WRAP(3)  WRAP(4)  WRAP(5)  WRAP(6)  WRAP(7)
WRAP(8)  WRAP(9)  WRAP(10) WRAP(11) WRAP(12) WRAP(13) WRAP(14) WRAP(15)

BYTE (*ser2_in[16])(void) = {
    in_0, in_1, in_2, in_3, in_4, in_5, in_6, in_7,
    in_8, in_9, in_10, in_11, in_12, in_13, in_14, in_15
};
void (*ser2_out[16])(BYTE) = {
    out_0, out_1, out_2, out_3, out_4, out_5, out_6, out_7,
    out_8, out_9, out_10, out_11, out_12, out_13, out_14, out_15
};
