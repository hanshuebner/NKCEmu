/***************************************************************************
 *  Centisecond timer for the NKC emulator (drives BBC BASIC's TIME).
 *
 *  Four I/O ports 0x31..0x34 expose a 32-bit centisecond counter, LSB first.
 *  Reading port 0x31 latches the live value so the four bytes are consistent;
 *  0x32..0x34 return the remaining latched bytes.  Writing the four bytes
 *  (0x34 last) sets the counter (PUTIME).  The count tracks host wall-clock
 *  time, so delays run in real time even though the CPU is unthrottled.
 *
 *  iosim's port dispatch uses zero-argument handlers, so there is one in/out
 *  function per port.
 ***************************************************************************/

#include <sys/time.h>
#include "sim.h"

static long origin = -1;            /* host csec corresponding to TIME == 0 */
static unsigned long latched;       /* snapshot taken when 0x31 is read     */
static unsigned long wbuf;          /* assembled across writes              */

static long host_csec(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 100 + tv.tv_usec / 10000;
}

static BYTE rd(int byte)
{
    if (byte == 0) {                /* low byte latches the live value */
        if (origin < 0) origin = host_csec();
        latched = (unsigned long)(host_csec() - origin);
    }
    return (BYTE)((latched >> (8 * byte)) & 0xFF);
}

static void wr(int byte, BYTE data)
{
    int sh = 8 * byte;
    wbuf = (wbuf & ~(0xFFUL << sh)) | ((unsigned long)data << sh);
    if (byte == 3) origin = host_csec() - (long)wbuf;   /* MSB last -> apply */
}

BYTE timer_p31_in(void) { return rd(0); }
BYTE timer_p32_in(void) { return rd(1); }
BYTE timer_p33_in(void) { return rd(2); }
BYTE timer_p34_in(void) { return rd(3); }
void timer_p31_out(BYTE d) { wr(0, d); }
void timer_p32_out(BYTE d) { wr(1, d); }
void timer_p33_out(BYTE d) { wr(2, d); }
void timer_p34_out(BYTE d) { wr(3, d); }
