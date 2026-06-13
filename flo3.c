/*
 * flo3.c -- FLO3 floppy controller (FD1797 + 9229B) for the NDR-Klein-Computer.
 *
 * Functional, register-level emulation -- NOT bit/MFM-level.  A raw DOS FAT12
 * floppy image (the -d option) is presented as a block device: the NKC ROM
 * implements the FAT filesystem itself, and here we only map a CHS request
 * (track, side, sector) to an image offset and stream the sector's bytes through
 * the data register.  Because the image IS a DOS floppy, disks are
 * PC-interchangeable by construction.
 *
 * Ports (base C0H, set by JMP3 on real hardware; A0/A1 select C0..C3) --
 * see the FLO3 construction manual, section 7.1.1:
 *   C0  command (write) / status (read)   FD1797
 *   C1  track register
 *   C2  sector register
 *   C3  data register   (also holds the SEEK target track before a SEEK)
 *   C4  control latch (write) / DRQ+INTRQ status (read)
 * C4 write bits: 0=side, 1=motor(0=on), 2=mini/maxi, 3=density(1=SD/0=DD),
 *                4..7=drive select A..D (one-hot).
 * C4 read  bits: 1=DRQ, 2=INTRQ, 3=head loaded.
 *
 * Timing: the controller responds with realistic, wall-clock delays so the NKC
 * driver's BUSY/DRQ waits are actually exercised (and the boot/load takes a
 * floppy-like ~1 s rather than being instantaneous).  A Type I command stays
 * BUSY for the head-step time (step rate x tracks + settle); a Type II command
 * paces DRQ at the ~250 kbit/s byte rate.  Times come from the host monotonic
 * clock, so they are independent of how fast the emulated CPU runs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include "sim.h"
#include "simglb.h"

#define MAXSEC 1024

/* realistic timings (microseconds) */
#define US_STEP    6000L     /* head step per track (step-rate, mini DD) */
#define US_SETTLE  30000L    /* head load + settle/verify after a seek */
#define US_RWLAT   8000L     /* sector search before the first data byte */
#define US_BYTE    27L       /* one byte at ~250 kbit/s MFM (~32 us) */

static FILE *img      = NULL;
static int   img_ro   = 0;
static long  img_size = 0;
/* geometry, taken from the image BPB (defaults: 720K DD, 9 sec/trk, 2 heads) */
static int   g_bps = 512, g_spt = 9, g_heads = 2;

/* FD1797 programmer-visible state */
static BYTE r_track  = 0;
static BYTE r_sector = 1;
static BYTE r_data   = 0;
static BYTE r_status = 0;
static BYTE ctrl     = 0;          /* last value written to C4 */
static int  intrq    = 0;

/* whole-sector transfer buffer */
static BYTE buf[MAXSEC];
static int  xfer = 0;              /* 0 idle, 1 reading, 2 writing */
static int  xidx = 0;              /* next byte index within buf */
static long xoff = 0;              /* image offset of the sector being written */

/* timing */
static int  seeking   = 0;         /* a Type I command is stepping/settling */
static long busy_until = 0;        /* us: BUSY clears (seek complete) at this time */
static long drq_at     = 0;        /* us: the next data byte becomes available */

/* status-register bits (FD1797) */
#define ST_BUSY  0x01
#define ST_DRQ   0x02              /* Type II */
#define ST_TRK0  0x04              /* Type I  */
#define ST_RNF   0x10              /* Type II: record not found */
#define ST_HLD   0x20              /* Type I: head loaded */
#define ST_WP    0x40
#define ST_NRDY  0x80

static long now_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000000L + tv.tv_usec;
}

static int flo3_dbg = -1;
static void dbg(const char *fmt, ...)
{
    va_list ap;
    if (flo3_dbg < 0) flo3_dbg = getenv("NKC_FLO3_DEBUG") ? 1 : 0;
    if (!flo3_dbg) return;
    va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
}

void flo3_set_image(const char *path)
{
    unsigned char bs[64];

    img = fopen(path, "r+b");
    if (!img) { img = fopen(path, "rb"); img_ro = 1; }
    if (!img) {
        fprintf(stderr, "flo3: cannot open image '%s'\n", path);
        return;
    }
    fseek(img, 0, SEEK_END);
    img_size = ftell(img);
    fseek(img, 0, SEEK_SET);

    /* BPB: bytes/sector @0x0B, sectors/track @0x18, heads @0x1A (little-endian) */
    if (fread(bs, 1, sizeof bs, img) == sizeof bs) {
        int bps   = bs[0x0B] | (bs[0x0C] << 8);
        int spt   = bs[0x18] | (bs[0x19] << 8);
        int heads = bs[0x1A] | (bs[0x1B] << 8);
        if (bps >= 128 && bps <= MAXSEC && spt > 0 && spt <= 63 &&
            heads >= 1 && heads <= 2) {
            g_bps = bps; g_spt = spt; g_heads = heads;
        }
    }
    fprintf(stderr, "flo3: %s (%ld bytes, %d bytes/sec, %d sec/trk, %d head%s%s)\n",
            path, img_size, g_bps, g_spt, g_heads, g_heads == 1 ? "" : "s",
            img_ro ? ", read-only" : "");
}

/* advance the time-driven state (seek completion) */
static void fdc_tick(void)
{
    if (seeking && now_us() >= busy_until) {
        seeking = 0;
        r_status &= ~ST_BUSY;
        intrq = 1;                 /* seek complete */
    }
}

/* is a data byte available yet (Type II, DRQ paced to the byte rate)? */
static int drq_ready(void)
{
    return xfer && xidx < g_bps && now_us() >= drq_at;
}

/* image offset of (r_track, side=C4.b0, r_sector); -1 if outside the image */
static long sec_offset(void)
{
    int  side = ctrl & 1;
    long lba;
    long off;

    if (r_sector < 1 || r_sector > g_spt) return -1;
    lba = ((long)r_track * g_heads + side) * g_spt + (r_sector - 1);
    off = lba * g_bps;
    if (off < 0 || off + g_bps > img_size) return -1;
    return off;
}

static void start_read(void)
{
    long off = sec_offset();
    if (!img || off < 0) { r_status = ST_RNF; intrq = 1; xfer = 0; return; }
    fseek(img, off, SEEK_SET);
    memset(buf, 0, sizeof buf);
    fread(buf, 1, g_bps, img);
    xfer = 1; xidx = 0;
    r_data = buf[0];
    r_status = ST_BUSY;            /* busy until the last byte is moved */
    intrq = 0;
    drq_at = now_us() + US_RWLAT;  /* first byte after the sector-search latency */
}

static void start_write(void)
{
    long off = sec_offset();
    if (!img || off < 0) { r_status = ST_RNF; intrq = 1; xfer = 0; return; }
    if (img_ro) { r_status = ST_WP; intrq = 1; xfer = 0; return; }
    xoff = off;
    xfer = 2; xidx = 0;
    r_status = ST_BUSY;
    intrq = 0;
    drq_at = now_us() + US_RWLAT;
}

/* ---- C0: command (write) / status (read) ---- */
void flo3_pC0_out(BYTE data)
{
    int top = data >> 4;

    dbg("flo3 t=%ld CMD %02X (trk=%d sec=%d)\n", now_us(), data, r_track, r_sector);

    if (top <= 7) {                          /* Type I: RESTORE/SEEK/STEP */
        int target = (top == 0) ? 0 : (top == 1) ? r_data : r_track;
        int steps  = target - r_track; if (steps < 0) steps = -steps;
        if (top == 0) steps = r_track;       /* RESTORE: step in until track 0 */
        r_track = target;
        seeking = 1;
        busy_until = now_us() + (long)steps * US_STEP + US_SETTLE;
        r_status = ST_BUSY | ST_HLD | (r_track == 0 ? ST_TRK0 : 0);
        intrq = 0; xfer = 0;
    } else if (top == 0x8 || top == 0x9) {   /* READ SECTOR */
        start_read();
    } else if (top == 0xA || top == 0xB) {   /* WRITE SECTOR */
        start_write();
    } else if (top == 0xD) {                 /* FORCE INTERRUPT */
        r_status = 0; seeking = 0; xfer = 0; intrq = (data & 0x0F) ? 1 : 0;
    } else {                                 /* Type III not modelled */
        r_status = ST_RNF; intrq = 1; xfer = 0;
    }
}

BYTE flo3_pC0_in(void)
{
    fdc_tick();
    BYTE s = r_status;
    if (xfer && drq_ready()) s |= ST_DRQ;    /* surface DRQ in the status too */
    if (!img) s |= ST_NRDY;
    return s;
}

/* ---- C1: track, C2: sector ---- */
BYTE flo3_pC1_in(void)        { return r_track; }
void flo3_pC1_out(BYTE data)  { r_track = data; }
BYTE flo3_pC2_in(void)        { return r_sector; }
void flo3_pC2_out(BYTE data)  { r_sector = data; }

/* ---- C3: data register ---- */
BYTE flo3_pC3_in(void)
{
    if (xfer == 1 && drq_ready()) {          /* streaming a sector out */
        BYTE b = buf[xidx];
        r_data = b;
        if (++xidx >= g_bps) {               /* last byte moved */
            xfer = 0; r_status = 0; intrq = 1;
        } else {
            drq_at = now_us() + US_BYTE;      /* pace the next byte */
        }
        return b;
    }
    return r_data;
}

void flo3_pC3_out(BYTE data)
{
    if (xfer == 2 && drq_ready()) {          /* streaming a sector in */
        buf[xidx] = data;
        if (++xidx >= g_bps) {               /* sector full -> flush */
            fseek(img, xoff, SEEK_SET);
            fwrite(buf, 1, g_bps, img);
            fflush(img);
            xfer = 0; r_status = 0; intrq = 1;
        } else {
            drq_at = now_us() + US_BYTE;
        }
        return;
    }
    r_data = data;                           /* idle: holds the SEEK target etc. */
}

/* ---- C4: control latch (write) / DRQ+INTRQ status (read) ---- */
void flo3_pC4_out(BYTE data)  { ctrl = data; }

BYTE flo3_pC4_in(void)
{
    BYTE s = 0x08;                           /* bit3: head loaded (always) */
    fdc_tick();
    if (drq_ready()) s |= 0x02;              /* bit1: DRQ   */
    if (intrq)       s |= 0x04;              /* bit2: INTRQ */
    return s;
}
