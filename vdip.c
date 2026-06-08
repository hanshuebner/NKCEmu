/***************************************************************************
 *  VDIP1 (FTDI VNC1L) USB-stick card emulation for the NKC, on I/O port 0x30
 *
 *  The Grundprogramm 3.1 talks to the VDIP1 by bit-banging an SPI link on a
 *  single latched port at 0x30.  This file emulates that link at the bit
 *  level and, on top of it, just enough of the VNC1L's VDAP "short command
 *  set" to make the monitor's USB routines (vnc_reset, vnc_write, the file
 *  read/write helpers) work against a directory on the host filesystem.
 *
 *  Wire protocol (derived from the Grundprogramm disassembly $1992..$1B13):
 *
 *    port 0x30 bit 0 : MISO   (VDIP -> CPU, the only input bit)
 *    port 0x30 bit 1 : MOSI   (CPU  -> VDIP)
 *    port 0x30 bit 2 : SCLK   (idle high; one pulse = high->low->high,
 *                              a data bit is committed on the rising edge)
 *    port 0x30 bit 6 : /RESET (a low pulse resets the VDIP)
 *    port 0x30 bit 7 : /CS    (held high for the duration of a transfer)
 *
 *  Each SPI transfer clocks, MSB first:
 *      1 start bit (always 1)
 *      1 R/W bit   (0 = write, 1 = read)
 *      1 REG bit   (0 = data register, 1 = status register)
 *      8 data bits
 *      1 ack bit   (driven on MISO: 0 = ok/valid, 1 = no data)
 *
 *  VDAP layer (the byte stream the monitor actually exchanges):
 *      0x09 OPW <sp>name<cr>   open file for writing (create/truncate)
 *      0x08 WRF <sp>NNNN<cr>.. write: 4-byte big-endian length, CR, then
 *                              exactly that many raw data bytes
 *      0x0A CLF <sp>name<cr>   close file
 *      0x04 RDF <sp>name<cr>   stream the whole file back as read data
 *      0x10           <cr>     mode/echo handshake used during reset
 *  Every command except RDF answers with a ">" prompt; RDF just delivers the
 *  file bytes and then reports "no data" (EOF) on the next read.
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <dirent.h>
#include "sim.h"
#include "simglb.h"

/* ----- where emulated USB-stick files live ----- */
static const char *vdip_dir(void)
{
    const char *d = getenv("NKC_USB_DIR");
    return (d && *d) ? d : "usb";
}

static FILE *dbg = NULL;
static void dbgopen(void)
{
    static int tried = 0;
    if (tried) return;
    tried = 1;
    if (getenv("NKC_USB_DEBUG"))
        dbg = stderr;
}
#define DBG(...) do { if (dbg) fprintf(dbg, __VA_ARGS__); } while (0)

/* ===================================================================== *
 *  RX FIFO: bytes the VDIP sends back to the CPU (prompts + file data)  *
 * ===================================================================== */
#define RXSIZE 0x12000              /* > max BASIC program + prompts */
static unsigned char rxbuf[RXSIZE];
static int rxhead = 0, rxtail = 0;  /* head = next to read, tail = next to write */

static void rx_reset(void) { rxhead = rxtail = 0; }
static int  rx_empty(void) { return rxhead == rxtail; }
static void rx_push(unsigned char b)
{
    int n = (rxtail + 1) % RXSIZE;
    if (n == rxhead) { DBG("VDIP: RX overflow\n"); return; }
    rxbuf[rxtail] = b;
    rxtail = n;
}
static int rx_pop(void)             /* -1 if empty */
{
    if (rx_empty()) return -1;
    int b = rxbuf[rxhead];
    rxhead = (rxhead + 1) % RXSIZE;
    return b;
}
static void rx_push_str(const char *s) { while (*s) rx_push((unsigned char)*s++); }

/* The monitor's vnc_waitres scans the reply until it sees '>' (success).
 * A ">" followed by a CR satisfies both that scan and the extra byte it
 * reads afterwards. */
static void rx_prompt(void) { rx_push_str(">\r"); }

/* ===================================================================== *
 *  VDAP command parser (consumes bytes written to the data register)    *
 * ===================================================================== */
enum { ST_CMD, ST_ARG, ST_WRF_HDR, ST_WRF_DATA };
static int  vstate = ST_CMD;
static int  vcmd = 0;
static char arg[64];
static int  arglen = 0;
static int  hdrcnt = 0;
static unsigned long wrf_len = 0, wrf_rem = 0;
static FILE *wfile = NULL;          /* file currently open for writing */

/* build the host path for a (possibly space-padded) VDAP name */
static void make_path(char *out, size_t n, const char *name)
{
    while (*name == ' ') name++;            /* skip leading space(s) */
    /* keep the basename only, drop any path separators for safety */
    const char *base = name;
    for (const char *p = name; *p; p++)
        if (*p == '/' || *p == '\\') base = p + 1;
    snprintf(out, n, "%s/%s", vdip_dir(), base);
}

static void cmd_open_write(const char *name)
{
    char path[512];
    make_path(path, sizeof path, name);
    mkdir(vdip_dir(), 0777);
    if (wfile) fclose(wfile);
    wfile = fopen(path, "wb");
    DBG("VDIP: OPW '%s' -> %s\n", name, wfile ? "ok" : "FAIL");
    rx_prompt();
}

static void cmd_close(const char *name)
{
    DBG("VDIP: CLF '%s'\n", name);
    if (wfile) { fclose(wfile); wfile = NULL; }
    rx_prompt();
}

/* DIR: the monitor's "USB Inhalt" sends 0x01<CR> and reads a listing of
 * CR-terminated names (a leading throwaway byte first).  We do not append a
 * prompt: the routine ignores read errors and just keeps reading, so once the
 * FIFO empties the over-reads return CR (see vdip_miso) and pad cleanly. */
static void cmd_dir(const char *pattern)
{
    (void)pattern;
    rx_push('\r');                  /* leading byte the monitor discards */
    DIR *d = opendir(vdip_dir());
    if (!d) { DBG("VDIP: DIR (empty/none)\n"); return; }
    struct dirent *e;
    int n = 0;
    while ((e = readdir(d))) {
        if (e->d_name[0] == '.') continue;          /* skip . .. dotfiles */
        rx_push_str(e->d_name);
        rx_push('\r');
        n++;
    }
    closedir(d);
    DBG("VDIP: DIR -> %d entries\n", n);
}

static void cmd_read_stream(const char *name)
{
    char path[512];
    make_path(path, sizeof path, name);
    FILE *f = fopen(path, "rb");
    DBG("VDIP: RDF '%s' -> %s\n", name, f ? "ok" : "FAIL");
    if (!f) { return; }             /* no data -> monitor sees EOF */
    int c;
    while ((c = fgetc(f)) != EOF) rx_push((unsigned char)c);
    fclose(f);
}

/* feed one byte written by the CPU into the command state machine */
static void vdap_write(unsigned char b)
{
    switch (vstate) {
    case ST_CMD:
        vcmd = b;
        arglen = 0; hdrcnt = 0; wrf_len = 0;
        switch (b) {
        case 0x08:  vstate = ST_WRF_HDR; break;             /* WRF */
        case 0x01:  /* DIR */
        case 0x09:  /* OPW */
        case 0x0A:  /* CLF */
        case 0x04:  /* RDF */
        case 0x10:  vstate = ST_ARG; break;                 /* mode/echo */
        default:    /* unknown single-byte command: just re-prompt */
                    rx_prompt(); break;
        }
        break;

    case ST_ARG:
        if (b == 0x0D) {            /* CR ends the argument */
            arg[arglen] = 0;
            switch (vcmd) {
            case 0x01: cmd_dir(arg);        break;          /* no prompt */
            case 0x09: cmd_open_write(arg); break;
            case 0x0A: cmd_close(arg);      break;
            case 0x04: cmd_read_stream(arg);break;          /* no prompt */
            case 0x10: rx_prompt();         break;
            default:   rx_prompt();         break;
            }
            vstate = ST_CMD;
        } else if (arglen < (int)sizeof(arg) - 1) {
            arg[arglen++] = (char)b;
        }
        break;

    case ST_WRF_HDR:
        /* layout: <space> L3 L2 L1 L0 <CR>, then wrf_len data bytes        */
        if (hdrcnt >= 1 && hdrcnt <= 4)
            wrf_len = (wrf_len << 8) | b;
        hdrcnt++;
        if (hdrcnt == 6) {          /* consumed space + 4 len bytes + CR */
            wrf_rem = wrf_len;
            DBG("VDIP: WRF len=%lu\n", wrf_len);
            if (wrf_rem == 0) { rx_prompt(); vstate = ST_CMD; }
            else              { vstate = ST_WRF_DATA; }
        }
        break;

    case ST_WRF_DATA:
        if (wfile) fputc(b, wfile);
        if (--wrf_rem == 0) {
            if (wfile) fflush(wfile);
            rx_prompt();
            vstate = ST_CMD;
        }
        break;
    }
}

/* ===================================================================== *
 *  Bit-level SPI slave on port 0x30                                     *
 * ===================================================================== */
#define B_MISO  0x01
#define B_MOSI  0x02
#define B_SCLK  0x04
#define B_RST   0x40
#define B_CS    0x80

static unsigned char last_out = 0;  /* last value written (for read-back) */
static int  in_xfer  = 0;           /* CS asserted, transfer in progress  */
static int  bitcnt   = 0;           /* rising edges seen this transfer    */
static int  rnw = 0, reg = 0;       /* control bits captured              */
static unsigned wbyte = 0;          /* byte being shifted in from MOSI    */
static unsigned shiftout = 0;       /* byte being shifted out to MISO     */
static int  valid = 0;              /* did we have a byte for a data read */

static void vdip_hw_reset(void)
{
    DBG("VDIP: hardware reset\n");
    if (wfile) { fclose(wfile); wfile = NULL; }
    vstate = ST_CMD;
    rx_reset();
    rx_prompt();                    /* power-on banner the monitor waits for */
    in_xfer = 0; bitcnt = 0;
}

/* status register read by the monitor: bit 1 == 0 means "RX data ready" */
static unsigned char vdip_status(void)
{
    return rx_empty() ? B_MOSI /*0x02, bit1 set = busy/no data*/ : 0x00;
}

/* current MISO level, recomputed on every IN */
static int vdip_miso(void)
{
    if (!in_xfer) return 0;
    if (bitcnt >= 3 && bitcnt <= 10)            /* the 8 data bits, MSB first */
        return (shiftout >> (10 - bitcnt)) & 1;
    if (bitcnt >= 11) {                          /* ack bit */
        if (rnw)  return valid ? 0 : 1;          /* read: 0 = byte was valid */
        return 0;                                /* write: 0 = accepted      */
    }
    return 0;
}

void vdip_p30_out(BYTE data)
{
    dbgopen();
    unsigned char prev = last_out;

    /* /RESET: trigger on the rising edge (reset released) */
    if (!(prev & B_RST) && (data & B_RST))
        vdip_hw_reset();

    /* /CS rising = start of a transfer */
    if (!(prev & B_CS) && (data & B_CS)) {
        in_xfer = 1; bitcnt = 0; wbyte = 0; rnw = reg = 0; valid = 0;
    }
    /* /CS falling = end of a transfer */
    if ((prev & B_CS) && !(data & B_CS))
        in_xfer = 0;

    /* SCLK rising edge commits one bit (only while CS is asserted) */
    if (in_xfer && !(prev & B_SCLK) && (data & B_SCLK)) {
        int mosi = (data & B_MOSI) ? 1 : 0;
        bitcnt++;
        switch (bitcnt) {
        case 1: break;                  /* start bit */
        case 2: rnw = mosi; break;
        case 3:
            reg = mosi;
            if (rnw) {                  /* a read: latch the byte to send out */
                if (reg) { shiftout = vdip_status(); valid = 1; }
                else {
                    int b = rx_pop();
                    /* On empty, hand back CR (0x0D) with the ack bit marking
                     * "no data" (valid=0).  Routines that check the ack/carry
                     * (vnc_clrfifo, ul_load) still stop; the directory reader
                     * ignores it and treats the CR as an empty field. */
                    if (b < 0) { shiftout = 0x0D; valid = 0; }
                    else       { shiftout = b;    valid = 1; }
                }
            }
            break;
        default:                        /* bitcnt 4..11 */
            if (!rnw && bitcnt >= 4 && bitcnt <= 11) {
                wbyte = ((wbyte << 1) | mosi) & 0x1FF;
                if (bitcnt == 11)
                    vdap_write((unsigned char)(wbyte & 0xFF));
            }
            break;
        }
    }

    last_out = data;
}

BYTE vdip_p30_in(void)
{
    dbgopen();
    return (BYTE)((last_out & ~B_MISO) | (vdip_miso() & B_MISO));
}
