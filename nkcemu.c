/***************************************************************************
 *   Copyright (C) 2007 by Torsten Evers   *
 *   tevers@onlinehome.de   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/**
 * Emulator f�r den NDR-Klein-Computer
 * Ausbaustufe SBC2+GDP64+KEY+ROA64
 * Basis: Z80-Emulator z80pack von Udo Munk
 */


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <memory.h>
#include <stdbool.h>
#include "sim.h"
#include "simglb.h"
#include "nkcglobal.h"

extern void int_on(void), int_off(void), mon(void);
extern void 


init_io(bool), exit_io(void);
extern int exatoi(char *);
extern void vdip_set_dir(const char *);
extern void flo3_set_image(const char *);
extern void key_inject_file(const char *);
int CAS_FILE=0;

/*
 *  This function saves the CPU and the memory into the file core.z80
 *
 */
/* The core.z80 layout: every CPU register in order, then the two 32K RAM
 * halves.  save_core and load_core walk the same table so the two stay in
 * sync. */
#define CORE_PARTS {                                                         \
    {&A, sizeof A}, {&F, sizeof F}, {&B, sizeof B}, {&C, sizeof C},           \
    {&D, sizeof D}, {&E, sizeof E}, {&H, sizeof H}, {&L, sizeof L},           \
    {&A_, sizeof A_}, {&F_, sizeof F_}, {&B_, sizeof B_}, {&C_, sizeof C_},   \
    {&D_, sizeof D_}, {&E_, sizeof E_}, {&H_, sizeof H_}, {&L_, sizeof L_},   \
    {&I, sizeof I}, {&IFF, sizeof IFF}, {&R, sizeof R}, {&PC, sizeof PC},     \
    {&STACK, sizeof STACK}, {&IX, sizeof IX}, {&IY, sizeof IY},               \
    {ram, 32768}, {ram + 32768, 32768},                                      \
}

static void save_core(void)
{
    int fd;
    struct { const void *p; size_t n; } parts[] = CORE_PARTS;
    size_t i;

    if ((fd = open("core.z80", O_WRONLY | O_CREAT, 0600)) == -1)
    {
        puts("can't open file core.z80");
        return;
    }
    for (i = 0; i < sizeof parts / sizeof parts[0]; i++)
        if (write(fd, parts[i].p, parts[i].n) != (ssize_t) parts[i].n)
        {
            perror("write core.z80");
            break;
        }
    close(fd);
}

/*
 *  This function loads the CPU and memory from the file core.z80
 *
 */
static void load_core(void)
{
    int fd;
    struct { void *p; size_t n; } parts[] = CORE_PARTS;
    size_t i;

    if ((fd = open("core.z80", O_RDONLY)) == -1)
    {
        puts("can't open file core.z80");
        return;
    }
    for (i = 0; i < sizeof parts / sizeof parts[0]; i++)
        if (read(fd, parts[i].p, parts[i].n) != (ssize_t) parts[i].n)
        {
            perror("read core.z80");
            break;
        }
    close(fd);
}

int main(int argc, char *argv[])
{
    int opt;
    char *pn = argv[0];
    int a_offset = 0;       /* -a: start-address offset (applied after PC set) */
    CAS_FILE=0;
    bool windowed=true;     /* default to windowed; -F forces fullscreen */

    while ((opt = getopt(argc, argv, "slhiwFm:f:x:b:c:a:u:k:d:")) != -1)
    {
        switch (opt)
        {
            case 's': s_flag = 1; break;            /* save core+CPU on exit */
            case 'l': l_flag = 1; break;            /* load core+CPU at start */
            case 'h': break_flag = 0; break;        /* execute HALT opcode */
            case 'i': i_flag = 1; break;            /* trap I/O on unused ports */
            case 'w': windowed = true; break;       /* windowed (the default) */
            case 'F': windowed = false; break;      /* fullscreen */
            case 'm': m_flag = exatoi(optarg); break;          /* init memory */
            case 'f': f_flag = atoi(optarg);                   /* CPU MHz */
                      tmax = f_flag * 10000; break;
            case 'x': x_flag = 1;                              /* load+execute */
                      strncpy(xfn, optarg, LENCMD-1); xfn[LENCMD-1] = '\0';
                      break;
            case 'b': b_flag = 1;                              /* EPROM @ $0000 */
                      strncpy(xfn, optarg, LENCMD-1); xfn[LENCMD-1] = '\0';
                      break;
            case 'a': a_offset = exatoi(optarg); break;        /* start address */
            case 'u': vdip_set_dir(optarg); break;             /* USB directory */
            case 'd': flo3_set_image(optarg); break;           /* FLO3 floppy image */
            case 'k': key_inject_file(optarg); break;          /* inject keys */
            case 'c':                                          /* CAS file */
                if ((CAS_FILE = open(optarg, O_RDWR | O_CREAT,
                                     S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)) == -1)
                {
                    puts("can't open CAS file");
                    return 1;
                }
                break;
            default:    /* '?' : unknown option or missing argument */
                fprintf(stderr, "usage: %s [-s] [-l] [-i] [-h] [-w] [-F] "
                        "[-m hex] [-f MHz] [-x file] [-b file] [-c file] "
                        "[-a hex] [-u dir] [-k file]\n", pn);
                fputs("\ts = save core and cpu on exit\n"
                      "\tl = load core and cpu at start\n"
                      "\ti = trap on I/O to unused ports\n"
                      "\th = execute HALT op-code\n"
                      "\tw = windowed (default)    F = fullscreen\n"
                      "\tm = init memory with hex value\n"
                      "\tf = CPU frequency in MHz\n"
                      "\tx = load and execute file\n"
                      "\tb = load EPROM image at $0000\n"
                      "\tc = file for CAS emulation\n"
                      "\ta = start-address offset (hex)\n"
                      "\tu = directory used as the USB stick (repeat to merge; default: .)\n"
                      "\tk = inject keystrokes from a file\n", stderr);
                return 1;
        }
    }
    putchar('\n');
    puts("NDR-Klein-Computer Simulator V0.6.1");
    puts("===================================");
    puts("based on Z80 emulator 'z80pack'");
    puts("by Udo Munk");
    putchar('\n');
    fflush(stdout);

    wrk_ram = PC = STACK = ram;
    PC += a_offset;                     /* -a start-address offset */
    memset((char *) ram, m_flag, 32768);
    memset((char *) ram + 32768, m_flag, 32768);
    if (l_flag)
        load_core();
    int_on();
    init_io(windowed);
    mon();
    if (s_flag)
        save_core();
    exit_io();
    int_off();
    return(0);
}
