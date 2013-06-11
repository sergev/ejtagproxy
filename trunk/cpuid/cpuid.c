/*
 * Detect CPUID of the device, connected to EJTAG port.
 *
 * Copyright (C) 2013 Serge Vakulenko
 *
 * This file is part of PIC32PROG project, which is distributed
 * under the terms of the GNU General Public License (GPL).
 * See the accompanying file "COPYING" for more details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

#include "adapter.h"

#define VERSION         "1."SVNVERSION

int debug_level;
adapter_t *adapter;
char *progname;
const char *copyright = "Copyright: (C) 2013 Serge Vakulenko";

void quit (void)
{
    if (adapter != 0) {
        adapter->close (adapter, 0);
        adapter = 0;
    }
}

void interrupted (int signum)
{
    fprintf (stderr, "\nInterrupted.\n");
    quit();
    _exit (-1);
}

void do_probe ()
{
    unsigned cpuid, impcode;

    /* Open and detect the device. */
    atexit (quit);
    adapter = adapter_open_mpsse ();
    if (! adapter) {
        fprintf (stderr, "Error detecting device -- check cable!\n");
        exit (1);
    }

    /* Check CPU identifier. */
    cpuid = adapter->get_idcode (adapter);
    if (cpuid == 0 || cpuid == ~0) {
        /* Device not detected. */
        fprintf (stderr, "Bad CPUID=%08x.\n", cpuid);
        return;
    }
    printf ("CPUID = %08x\n", cpuid);
    if ((cpuid & 0xfff) == 0x053) {
        /* Microchip PIC32. */
        printf ("Microchip PIC32 detected.\n");
    } else {
        /* Read IMPCODE. */
        impcode = adapter->get_impcode (adapter);
        printf ("IMPCODE = %08x\n", impcode);
        if (impcode & 1)
            printf ("MIPS64 core detected.\n");
        else
            printf ("MIPS32 core detected.\n");
    }
}

/*
 * Print copying part of license
 */
static void gpl_show_copying (void)
{
    printf ("%s.\n\n", copyright);
    printf ("This program is free software; you can redistribute it and/or modify\n");
    printf ("it under the terms of the GNU General Public License as published by\n");
    printf ("the Free Software Foundation; either version 2 of the License, or\n");
    printf ("(at your option) any later version.\n");
    printf ("\n");
    printf ("This program is distributed in the hope that it will be useful,\n");
    printf ("but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
    printf ("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
    printf ("GNU General Public License for more details.\n");
    printf ("\n");
}

/*
 * Print NO WARRANTY part of license
 */
static void gpl_show_warranty (void)
{
    printf ("%s.\n\n", copyright);
    printf ("BECAUSE THE PROGRAM IS LICENSED FREE OF CHARGE, THERE IS NO WARRANTY\n");
    printf ("FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW.  EXCEPT WHEN\n");
    printf ("OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES\n");
    printf ("PROVIDE THE PROGRAM \"AS IS\" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED\n");
    printf ("OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF\n");
    printf ("MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE RISK AS\n");
    printf ("TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.  SHOULD THE\n");
    printf ("PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,\n");
    printf ("REPAIR OR CORRECTION.\n");
    printf("\n");
    printf ("IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING\n");
    printf ("WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY AND/OR\n");
    printf ("REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES,\n");
    printf ("INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING\n");
    printf ("OUT OF THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED\n");
    printf ("TO LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY\n");
    printf ("YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER\n");
    printf ("PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE\n");
    printf ("POSSIBILITY OF SUCH DAMAGES.\n");
    printf("\n");
}

int main (int argc, char **argv)
{
    int ch;
    static const struct option long_options[] = {
        { "help",        0, 0, 'h' },
        { "warranty",    0, 0, 'W' },
        { "copying",     0, 0, 'C' },
        { "version",     0, 0, 'V' },
        { NULL,          0, 0, 0 },
    };

    setvbuf (stdout, (char *)NULL, _IOLBF, 0);
    setvbuf (stderr, (char *)NULL, _IOLBF, 0);
    progname = argv[0];
    signal (SIGINT, interrupted);
#ifdef __linux__
    signal (SIGHUP, interrupted);
#endif
    signal (SIGTERM, interrupted);

    while ((ch = getopt_long (argc, argv, "hWCVD",
      long_options, 0)) != -1) {
        switch (ch) {
        case 'h':
            break;
        case 'W':
            gpl_show_warranty ();
            return 0;
        case 'C':
            gpl_show_copying ();
            return 0;
        case 'V':
            printf ("Detect device on EJTAG port, Version %s\n", VERSION);
            printf ("%s\n", copyright);
            return 0;
        case 'D':
            ++debug_level;
            continue;
        }
        printf ("Detect device on EJTAG port, Version %s\n", VERSION);
        printf ("%s.\n\n", copyright);
        printf ("CPUID comes with ABSOLUTELY NO WARRANTY; for details\n");
        printf ("use `--warranty' option. This is Open Source software. You are\n");
        printf ("welcome to redistribute it under certain conditions. Use the\n");
        printf ("'--copying' option for details.\n");
        printf ("\nUsage:\n");
        printf ("       cpuid [-D]\n");
        printf ("\nOptions:\n");
        printf ("       -D                  Debug mode\n");
        printf ("       -h, --help          Print this help message\n");
        printf ("       -V, --version       Print version\n");
        printf ("       -C, --copying       Print copying information\n");
        printf ("       -W, --warranty      Print warranty information\n");
        printf ("\n");
        return 0;
    }
    argc -= optind;
    argv += optind;

    do_probe ();
    quit ();
    return 0;
}
