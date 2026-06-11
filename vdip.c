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
 *  file bytes and then reports "no data" (EOF) on the next read.  (Real VNC1L
 *  hardware re-prompts after RDF too; the firmware copes by trimming a trailing
 *  ">\r" off file reads -- see VDLOAD / the monitor C command.)
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <fnmatch.h>
#include "sim.h"
#include "simglb.h"

/* ----- where emulated USB-stick files live -----
 * The "stick" can be a MERGE of several host directories: each -u option adds
 * one (e.g. `-u build -u examples`).  Reads search the directories in order and
 * use the first match; writes (and mkdir) go to the first directory.  This lets
 * the stick combine the build output (BASIC.BIN) with the examples without
 * copying or symlinking.
 * Precedence when no -u is given: NKC_USB_DIR env > current directory. */
#define VDIP_MAXDIRS 16
static const char *vdip_dirs[VDIP_MAXDIRS];
static int vdip_ndirs = 0;

void vdip_set_dir(const char *path)
{
    if (path && *path && vdip_ndirs < VDIP_MAXDIRS)
        vdip_dirs[vdip_ndirs++] = path;
}

/* primary directory: used for writes, mkdir, and as the fallback for reads */
static const char *vdip_dir(void)
{
    if (vdip_ndirs) return vdip_dirs[0];
    const char *d = getenv("NKC_USB_DIR");
    return (d && *d) ? d : ".";
}

/* number of directories to scan and the i-th one (covers the no-`-u` case) */
static int vdip_dircount(void) { return vdip_ndirs ? vdip_ndirs : 1; }
static const char *vdip_diri(int i)
{
    return vdip_ndirs ? vdip_dirs[i] : vdip_dir();
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

/* Prompts and error responses are mode-dependent, like the real chip:
 * verbose extended forms after reset ("D:\>", "Bad Command"), terse short
 * forms once SCS is selected (">", "BC"). */
static int scs_mode = 0;            /* 0 = extended (power-on default) */

static void rx_prompt(void)
{
    rx_push_str(scs_mode ? ">\r" : "D:\\>\r");
}

static void rx_err(const char *code2, const char *verbose)
{
    rx_push_str(scs_mode ? code2 : verbose);
    rx_push('\r');
}

/* ===================================================================== *
 *  VDAP command parser (consumes bytes written to the data register)    *
 *  Faithful to the Vinculum Firmware User Manual: extended vs short     *
 *  command sets, ONE open file, binary dword parameters MSB first,      *
 *  DIR <file> size replies LSB first, write-truncation semantics.       *
 * ===================================================================== */
enum { ST_CMD, ST_NAMEARG, ST_TEXT, ST_BINPAR, ST_WRF_DATA };
static int  vstate = ST_CMD;
static int  vcmd = 0;
static char arg[64];
static int  arglen = 0;
static int  parcnt = 0;             /* binary-parameter parse position */
static unsigned long parval = 0;    /* the 32-bit MSB-first parameter  */
static unsigned long wrf_rem = 0;
static int  wrf_ok = 0;

/* --- THE open file: the real VNC1L has exactly one --- */
static FILE *ofile = NULL;
static char oname[64];
static int  omode = 0;              /* 0 none, 1 read (OPR), 2 write (OPW) */

/* basename of a (possibly space-padded) VDAP name, with path separators dropped */
static const char *vdip_basename(const char *name)
{
    while (*name == ' ') name++;            /* skip leading space(s) */
    const char *base = name;
    for (const char *p = name; *p; p++)
        if (*p == '/' || *p == '\\') base = p + 1;
    return base;
}

/* host path for writing/creating: always the primary directory */
static void make_path(char *out, size_t n, const char *name)
{
    snprintf(out, n, "%s/%s", vdip_dir(), vdip_basename(name));
}

/* host path for reading: the first directory that actually has the file
 * (falls back to the primary directory's path, so fopen then fails -> EOF) */
static void make_read_path(char *out, size_t n, const char *name)
{
    const char *base = vdip_basename(name);
    for (int i = 0; i < vdip_dircount(); i++) {
        struct stat st;
        snprintf(out, n, "%s/%s", vdip_diri(i), base);
        if (stat(out, &st) == 0 && S_ISREG(st.st_mode)) return;
    }
    snprintf(out, n, "%s/%s", vdip_dir(), base);
}

static long file_size(FILE *f)
{
    long cur = ftell(f), sz;
    fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, cur, SEEK_SET);
    return sz;
}

static int dir_cmp(const void *a, const void *b)
{
    return strcasecmp(*(const char *const *)a, *(const char *const *)b);
}

/* DIR (no argument): blank line, the names sorted and CR-terminated, prompt. */
static void cmd_dir(void)
{
    rx_push('\r');                  /* the leading blank line */

    static char *names[2048];
    const int cap = (int)(sizeof names / sizeof names[0]);
    int n = 0;
    /* union of all stick directories; first occurrence of a name wins */
    for (int di = 0; di < vdip_dircount(); di++) {
        DIR *d = opendir(vdip_diri(di));
        if (!d) continue;
        struct dirent *e;
        while ((e = readdir(d)) && n < cap) {
            if (e->d_name[0] == '.') continue;          /* skip . .. dotfiles */
            int dup = 0;
            for (int k = 0; k < n; k++)
                if (strcasecmp(names[k], e->d_name) == 0) { dup = 1; break; }
            if (!dup) names[n++] = strdup(e->d_name);
        }
        closedir(d);
    }
    qsort(names, n, sizeof names[0], dir_cmp);
    for (int i = 0; i < n; i++) {
        rx_push_str(names[i]);
        rx_push('\r');
        free(names[i]);
    }
    DBG("VDIP: DIR -> %d entries\n", n);
    rx_prompt();
}

/* DIR <name>: blank line, "NAME <size>" (4 binary bytes, LSB first), CR,
 * prompt.  This is how the firmware learns a file's exact length. */
static void cmd_dirfile(const char *name)
{
    char path[512]; struct stat st;
    make_read_path(path, sizeof path, name);
    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
        DBG("VDIP: DIR '%s' -> CF\n", name);
        rx_err("CF", "Command Failed");
        return;
    }
    unsigned long v = (unsigned long)st.st_size;
    rx_push('\r');
    rx_push_str(vdip_basename(name));
    rx_push(' ');
    rx_push(v & 0xFF); rx_push((v >> 8) & 0xFF);
    rx_push((v >> 16) & 0xFF); rx_push((v >> 24) & 0xFF);
    rx_push('\r');
    rx_prompt();
    DBG("VDIP: DIR '%s' -> %lu bytes\n", name, v);
}

static void cmd_opr(const char *name)
{
    if (omode == 2) { rx_err("FO", "File Open"); return; }
    if (ofile) { fclose(ofile); ofile = NULL; omode = 0; }  /* replace a read */
    char path[512];
    make_read_path(path, sizeof path, name);
    ofile = fopen(path, "rb");
    if (!ofile) { DBG("VDIP: OPR '%s' -> CF\n", name); rx_err("CF", "Command Failed"); return; }
    omode = 1;
    snprintf(oname, sizeof oname, "%s", vdip_basename(name));
    DBG("VDIP: OPR '%s' (%ld bytes)\n", oname, file_size(ofile));
    rx_prompt();
}

static void cmd_opw(const char *name)
{
    if (omode == 2) { rx_err("FO", "File Open"); return; }
    if (ofile) { fclose(ofile); ofile = NULL; omode = 0; }
    char path[512]; struct stat st;
    make_read_path(path, sizeof path, name);
    if (stat(path, &st) != 0) { make_path(path, sizeof path, name); mkdir(vdip_dir(), 0777); }
    ofile = fopen(path, "r+b");
    if (!ofile) ofile = fopen(path, "w+b");
    if (!ofile) { rx_err("CF", "Command Failed"); return; }
    fseek(ofile, 0, SEEK_END);                  /* OPW appends to existing data */
    omode = 2;
    snprintf(oname, sizeof oname, "%s", vdip_basename(name));
    DBG("VDIP: OPW '%s' (pos %ld)\n", oname, ftell(ofile));
    rx_prompt();
}

static void cmd_clf(const char *name)
{
    if (!ofile || strcasecmp(vdip_basename(name), oname) != 0) {
        DBG("VDIP: CLF '%s' -> CF (open: '%s')\n", name, ofile ? oname : "-");
        rx_err("CF", "Command Failed");
        return;
    }
    if (omode == 2) {               /* write files truncate at the pointer */
        fflush(ofile);
        ftruncate(fileno(ofile), ftell(ofile));
    }
    fclose(ofile); ofile = NULL; omode = 0;
    DBG("VDIP: CLF '%s'\n", name);
    rx_prompt();
}

static void cmd_rdf(unsigned long count)
{
    if (!ofile) { rx_err("FI", "Invalid"); return; }
    long sz = file_size(ofile), pos = ftell(ofile);
    if (pos >= sz) { DBG("VDIP: RDF %lu at EOF\n", count); rx_err("CF", "Command Failed"); return; }
    unsigned long avail = (unsigned long)(sz - pos);
    if (count > avail) count = avail;   /* short read at the end, no error */
    DBG("VDIP: RDF %lu (pos %ld)\n", count, pos);
    for (unsigned long i = 0; i < count; i++) {
        int c = fgetc(ofile);
        if (c == EOF) break;
        rx_push((unsigned char)c);
    }
    rx_prompt();
}

static void cmd_sek(unsigned long pos)
{
    if (!ofile) { rx_err("FI", "Invalid"); return; }
    if ((long)pos > file_size(ofile)) { rx_err("CF", "Command Failed"); return; }
    fseek(ofile, (long)pos, SEEK_SET);
    DBG("VDIP: SEK %lu\n", pos);
    rx_prompt();
}

static void cmd_wrf_done(void)      /* runs after the data bytes arrived */
{
    if (!wrf_ok) { rx_err("FI", "Invalid"); return; }
    fflush(ofile);
    /* "The end of the file is moved to the position of the file pointer
     * after a write operation" -- modelled literally, so the firmware's
     * matching assumption is exercised here.  (To be verified on the real
     * chip; relax both sides together if it turns out kinder.) */
    ftruncate(fileno(ofile), ftell(ofile));
    rx_prompt();
}

static void cmd_read_stream(const char *name)   /* RD: whole file + prompt */
{
    if (omode == 2) { rx_err("FO", "File Open"); return; }
    char path[512];
    make_read_path(path, sizeof path, name);
    FILE *f = fopen(path, "rb");
    if (!f) { DBG("VDIP: RD '%s' -> CF\n", name); rx_err("CF", "Command Failed"); return; }
    int c, n = 0;
    while ((c = fgetc(f)) != EOF) { rx_push((unsigned char)c); n++; }
    fclose(f);
    DBG("VDIP: RD '%s' -> %d bytes\n", name, n);
    rx_prompt();
}

static void cmd_delete(const char *name)
{
    if (omode == 2) { rx_err("FO", "File Open"); return; }
    char path[512];
    make_read_path(path, sizeof path, name);   /* delete it wherever it lives */
    int r = unlink(path);
    DBG("VDIP: DLF '%s' -> %s\n", name, r == 0 ? "ok" : "CF");
    if (r != 0) { rx_err("CF", "Command Failed"); return; }
    rx_prompt();
}

static void cmd_rename(const char *a)
{
    if (omode == 2) { rx_err("FO", "File Open"); return; }
    char oldn[256], newn[256], oldp[512], newp[512];
    const char *sp = strchr(a, ' ');
    if (!sp) { rx_err("BC", "Bad Command"); return; }
    snprintf(oldn, sizeof oldn, "%.*s", (int)(sp - a), a);
    while (*sp == ' ') sp++;
    snprintf(newn, sizeof newn, "%s", sp);
    make_read_path(oldp, sizeof oldp, oldn);
    make_path(newp, sizeof newp, newn);
    int r = rename(oldp, newp);
    DBG("VDIP: REN '%s' -> '%s': %s\n", oldn, newn, r == 0 ? "ok" : "CF");
    if (r != 0) { rx_err("CF", "Command Failed"); return; }
    rx_prompt();
}

/* a full ASCII command line (the only form in extended mode; in short mode
 * this is how "SCS"/"ECS"/"E"/"e" still work as text, per the datasheet) */
static void text_dispatch(const char *line)
{
    DBG("VDIP: text '%s' (%s)\n", line, scs_mode ? "SCS" : "ECS");
    if (!*line)                        { rx_prompt(); return; }  /* disk check */
    if (strcasecmp(line, "SCS") == 0)  { scs_mode = 1; rx_prompt(); return; }
    if (strcasecmp(line, "ECS") == 0)  { scs_mode = 0; rx_prompt(); return; }
    if (strcmp(line, "E") == 0)        { rx_push_str("E\r"); return; }
    if (strcmp(line, "e") == 0)        { rx_push_str("e\r"); return; }
    if (strcasecmp(line, "IPA") == 0 ||
        strcasecmp(line, "IPH") == 0)  { rx_prompt(); return; }
    if (strcasecmp(line, "FWV") == 0)  { rx_push_str("\rMAIN 03.68VDAPF\rRPRG 1.00R\r");
                                         rx_prompt(); return; }
    if (strncasecmp(line, "DIR", 3) == 0 && (line[3] == 0 || line[3] == ' ')) {
        const char *a = line + 3;
        while (*a == ' ') a++;
        if (*a) cmd_dirfile(a); else cmd_dir();
        return;
    }
    rx_err("BC", "Bad Command");
}

/* feed one byte written by the CPU into the command state machine */
static void vdap_write(unsigned char b)
{
    switch (vstate) {
    case ST_CMD:
        arglen = 0;
        if (!scs_mode) {                    /* extended mode: text lines only */
            if (b == 0x0D) { text_dispatch(""); break; }
            arg[arglen++] = (char)b;
            vstate = ST_TEXT;
            break;
        }
        vcmd = b;
        switch (b) {
        case 0x01: case 0x04: case 0x07: case 0x09:
        case 0x0A: case 0x0C: case 0x0E:
        case 0x10: case 0x11:
            vstate = ST_NAMEARG; break;     /* ASCII argument (or none) + CR */
        case 0x08: case 0x0B: case 0x28:
            parcnt = 0; parval = 0;
            vstate = ST_BINPAR; break;      /* ' ' + dword (MSB first) + CR */
        case 0x0D:
            rx_prompt(); break;             /* bare CR: disk presence check */
        default:
            if (b >= 0x20 && b < 0x7F) {    /* ASCII text command in SCS mode */
                arg[arglen++] = (char)b;
                vstate = ST_TEXT;
            } else {
                vcmd = 0;                   /* unknown opcode: eat until CR */
                vstate = ST_NAMEARG;
            }
            break;
        }
        break;

    case ST_TEXT:
        if (b == 0x0D) { arg[arglen] = 0; vstate = ST_CMD; text_dispatch(arg); }
        else if (arglen < (int)sizeof(arg) - 1) arg[arglen++] = (char)b;
        break;

    case ST_NAMEARG:
        if (b != 0x0D) {
            if (arglen < (int)sizeof(arg) - 1) arg[arglen++] = (char)b;
            break;
        }
        arg[arglen] = 0;
        vstate = ST_CMD;
        {
            const char *a = arg;
            while (*a == ' ') a++;
            switch (vcmd) {
            case 0x01: if (*a) cmd_dirfile(a); else cmd_dir();   break;
            case 0x04: cmd_read_stream(a);                       break;
            case 0x07: cmd_delete(a);                            break;
            case 0x09: cmd_opw(a);                               break;
            case 0x0A: cmd_clf(a);                               break;
            case 0x0C: cmd_rename(a);                            break;
            case 0x0E: cmd_opr(a);                               break;
            case 0x10: scs_mode = 1; rx_prompt();                break;
            case 0x11: scs_mode = 0; rx_prompt();                break;
            default:   rx_err("BC", "Bad Command");              break;
            }
        }
        break;

    case ST_BINPAR:
        /* ' ' + 4 value bytes + CR.  The value bytes are BINARY -- they may
         * be 0x20 or 0x0D themselves -- so parse strictly by position. */
        if (parcnt == 0) {
            if (b != ' ') { vstate = ST_CMD; rx_err("BC", "Bad Command"); break; }
        } else if (parcnt <= 4) {
            parval = (parval << 8) | b;     /* MSB first */
        } else {
            vstate = ST_CMD;
            if (b != 0x0D) { rx_err("BC", "Bad Command"); break; }
            switch (vcmd) {
            case 0x0B: cmd_rdf(parval); break;
            case 0x28: cmd_sek(parval); break;
            case 0x08:                      /* WRF: the data follows */
                wrf_rem = parval;
                wrf_ok = (omode == 2);
                DBG("VDIP: WRF %lu (%s)\n", parval, wrf_ok ? "ok" : "no write file");
                if (wrf_rem == 0) cmd_wrf_done();
                else              vstate = ST_WRF_DATA;
                break;
            }
            break;
        }
        parcnt++;
        break;

    case ST_WRF_DATA:
        if (wrf_ok) fputc(b, ofile);
        if (--wrf_rem == 0) { vstate = ST_CMD; cmd_wrf_done(); }
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
    if (ofile) { fclose(ofile); ofile = NULL; }     /* unflushed tail is lost */
    omode = 0;
    vstate = ST_CMD;
    scs_mode = 0;               /* the chip powers up in the EXTENDED set */
    rx_reset();
    /* Power-on banner, matching the real VNC1L (firmware 0.3.68): a few verbose
     * lines ending in the "D:\>" disk-ready prompt.  vnc_reset drains this
     * until the '>' and then sends the ASCII "SCS\r" to enter the short set. */
    rx_push_str("Ver 0.3.68.DEPF On-Line:\r"
                "Device Detected P2\r"
                "No Upgrade\r"
                "D:\\>\r");
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
