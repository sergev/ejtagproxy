/*
 * Interface to a standard MIPS EJTAG debug port.
 * This file is independent of any JTAG or ICSP adapter used.
 * Most of codelets borrowed from OpenOCD project.
 *
 * Copyright (C) 2012 Serge Vakulenko
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. The name of the author may not be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include "target.h"
#include "adapter.h"
#include "mips.h"
#include "ejtag.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))

struct _target_t {
    adapter_t   *adapter;
    const char  *cpu_name;
    unsigned    cpuid;
    unsigned    impcode;
    unsigned    prid;
    unsigned    is_running;
    unsigned    is_pic32;
    unsigned    flash_kbytes;
    unsigned    boot_kbytes;

    unsigned    reg [38];
#define REG_STATUS      32              /* Should match GDB. */
#define REG_LO          33
#define REG_HI          34
#define REG_BADVADDR    35
#define REG_CAUSE       36
#define REG_DEPC        37

    unsigned    dcr;                    /* Debug control register. */
    int         dcr_valid;
    unsigned    nbpoints;               /* Number of hardware breakpoints. */
    unsigned    bp_value [15];
    unsigned    bp_used [15];
    unsigned    bp_id;
    unsigned    nwatchpoints;           /* Number of hardware watchpoints. */
    unsigned    watch_value [15];
    unsigned    watch_used [15];
    unsigned    watch_id;
};

static target_t *target_current;        /* For sigint handler */

static const struct {
    unsigned devid;
    const char *name;
    unsigned flash_kbytes;
    unsigned boot_kbytes;
} microchip[] = {
    /* PIC32 MX1xx/2xx family */
    {0x4A07053, "Microchip PIC32MX110F016B",  16,  3},
    {0x4A09053, "Microchip PIC32MX110F016C",  16,  3},
    {0x4A0B053, "Microchip PIC32MX110F016D",  16,  3},
    {0x4A06053, "Microchip PIC32MX120F032B",  32,  3},
    {0x4A08053, "Microchip PIC32MX120F032C",  32,  3},
    {0x4A0A053, "Microchip PIC32MX120F032D",  32,  3},
    {0x4D07053, "Microchip PIC32MX130F064B",  64,  3},
    {0x4D09053, "Microchip PIC32MX130F064C",  64,  3},
    {0x4D0B053, "Microchip PIC32MX130F064D",  64,  3},
    {0x4D06053, "Microchip PIC32MX150F128B", 128,  3},
    {0x4D08053, "Microchip PIC32MX150F128C", 128,  3},
    {0x4D0A053, "Microchip PIC32MX150F128D", 128,  3},
    {0x4A01053, "Microchip PIC32MX210F016B",  16,  3},
    {0x4A03053, "Microchip PIC32MX210F016C",  16,  3},
    {0x4A05053, "Microchip PIC32MX210F016D",  16,  3},
    {0x4A00053, "Microchip PIC32MX220F032B",  32,  3},
    {0x4A02053, "Microchip PIC32MX220F032C",  32,  3},
    {0x4A04053, "Microchip PIC32MX220F032D",  32,  3},
    {0x4D01053, "Microchip PIC32MX230F064B",  64,  3},
    {0x4D03053, "Microchip PIC32MX230F064C",  64,  3},
    {0x4D05053, "Microchip PIC32MX230F064D",  64,  3},
    {0x4D00053, "Microchip PIC32MX250F128B", 128,  3},
    {0x4D02053, "Microchip PIC32MX250F128C", 128,  3},
    {0x4D04053, "Microchip PIC32MX250F128D", 128,  3},

    /* PIC32 MX3xx/4xx family */
    {0x0902053, "Microchip PIC32MX320F032H",  32, 12},
    {0x0906053, "Microchip PIC32MX320F064H",  64, 12},
    {0x090A053, "Microchip PIC32MX320F128H", 128, 12},
    {0x092A053, "Microchip PIC32MX320F128L", 128, 12},
    {0x090D053, "Microchip PIC32MX340F128H", 128, 12},
    {0x092D053, "Microchip PIC32MX340F128L", 128, 12},
    {0x0912053, "Microchip PIC32MX340F256H", 256, 12},
    {0x0916053, "Microchip PIC32MX340F512H", 512, 12},
    {0x0934053, "Microchip PIC32MX360F256L", 256, 12},
    {0x0938053, "Microchip PIC32MX360F512L", 512, 12},
    {0x0942053, "Microchip PIC32MX420F032H",  32, 12},
    {0x094D053, "Microchip PIC32MX440F128H", 128, 12},
    {0x096D053, "Microchip PIC32MX440F128L", 128, 12},
    {0x0952053, "Microchip PIC32MX440F256H", 256, 12},
    {0x0956053, "Microchip PIC32MX440F512H", 512, 12},
    {0x0974053, "Microchip PIC32MX460F256L", 256, 12},
    {0x0978053, "Microchip PIC32MX460F512L", 512, 12},

    /* PIC32 MX5xx/6xx/7xx family */
    {0x4400053, "Microchip PIC32MX534F064H",  64, 12},
    {0x440C053, "Microchip PIC32MX534F064L",  64, 12},
    {0x4401053, "Microchip PIC32MX564F064H",  64, 12},
    {0x440D053, "Microchip PIC32MX564F064L",  64, 12},
    {0x4403053, "Microchip PIC32MX564F128H", 128, 12},
    {0x440F053, "Microchip PIC32MX564F128L", 128, 12},
    {0x4317053, "Microchip PIC32MX575F256H", 256, 12},
    {0x4333053, "Microchip PIC32MX575F256L", 256, 12},
    {0x4309053, "Microchip PIC32MX575F512H", 512, 12},
    {0x430F053, "Microchip PIC32MX575F512L", 512, 12},
    {0x4405053, "Microchip PIC32MX664F064H",  64, 12},
    {0x4411053, "Microchip PIC32MX664F064L",  64, 12},
    {0x4407053, "Microchip PIC32MX664F128H", 128, 12},
    {0x4413053, "Microchip PIC32MX664F128L", 128, 12},
    {0x430B053, "Microchip PIC32MX675F256H", 256, 12},
    {0x4305053, "Microchip PIC32MX675F256L", 256, 12},
    {0x430C053, "Microchip PIC32MX675F512H", 512, 12},
    {0x4311053, "Microchip PIC32MX675F512L", 512, 12},
    {0x4325053, "Microchip PIC32MX695F512H", 512, 12},
    {0x4341053, "Microchip PIC32MX695F512L", 512, 12},
    {0x440B053, "Microchip PIC32MX764F128H", 128, 12},
    {0x4417053, "Microchip PIC32MX764F128L", 128, 12},
    {0x4303053, "Microchip PIC32MX775F256H", 256, 12},
    {0x4312053, "Microchip PIC32MX775F256L", 256, 12},
    {0x430D053, "Microchip PIC32MX775F512H", 512, 12},
    {0x4306053, "Microchip PIC32MX775F512L", 512, 12},
    {0x430E053, "Microchip PIC32MX795F512H", 512, 12},
    {0x4307053, "Microchip PIC32MX795F512L", 512, 12},
    {0}
};

static const struct {
    unsigned devid;
    const char *name;
} devtab[] = {
    /* Legacy processors */
    { 0x000100, "R2000"                     },
//  { 0x000200, "R3000"                     },
    { 0x000300, "R6000"                     },
    { 0x000400, "R4000"                     },
    { 0x000600, "R6000A"                    },
    { 0x000900, "R10000"                    },
    { 0x000b00, "R4300"                     },
    { 0x000c00, "VR41xx"                    },
    { 0x000e00, "R12000"                    },
    { 0x000f00, "R14000"                    },
    { 0x001000, "R8000"                     },
    { 0x001200, "PR4450"                    },
    { 0x002000, "R4600"                     },
    { 0x002100, "R4700"                     },
    { 0x002200, "TX39xx"                    },
    { 0x002200, "R4640"                     },
    { 0x002300, "R5000"                     },
    { 0x002d00, "TX49xx"                    },
    { 0x002400, "Sonic"                     },
    { 0x002500, "Magic"                     },
    { 0x002700, "RM7000"                    },
    { 0x002800, "RM5260"                    },
    { 0x003400, "RM9000"                    },
    { 0x004200, "Loongson1"                 },
    { 0x005400, "R5432"                     },
    { 0x005500, "R5500"                     },
    { 0x006300, "Loongson2"                 },

    /* MIPS Technologies */
    { 0x018000, "MIPS Technologies 4Kc"     },
    { 0x018100, "MIPS Technologies 5Kc"     },
    { 0x018200, "MIPS Technologies 20Kc"    },
    { 0x018300, "MIPS Technologies 4Kmp"    },
    { 0x018400, "MIPS Technologies 4KEc"    },
    { 0x018500, "MIPS Technologies 4KEmp"   },
    { 0x018600, "MIPS Technologies 4KSc"    },
    { 0x018700, "MIPS Technologies M4K"     },
    { 0x018800, "MIPS Technologies 25Kf"    },
    { 0x018900, "MIPS Technologies 5KE"     },
    { 0x019000, "MIPS Technologies 4KEcR2"  },
    { 0x019100, "MIPS Technologies 4KEmpR2" },
    { 0x019200, "MIPS Technologies 4Ksd"    },
    { 0x019300, "MIPS Technologies 24K"     },
    { 0x019500, "MIPS Technologies 34K"     },
    { 0x019600, "MIPS Technologies 24KE"    },
    { 0x019700, "MIPS Technologies 74K"     },
    { 0x019900, "MIPS Technologies 1004K"   },
    { 0x019a00, "MIPS Technologies 1074K"   },
    { 0x019c00, "MIPS Technologies M14Kc"   },
    { 0x019e00, "MIPS Technologies M14KEc"  },

    /* Broadcom */
    { 0x024000, "Broadcom BCM4710"          },
    { 0x029000, "Broadcom BCM6338"          },
    { 0x028000, "Broadcom BCM6345"          },
    { 0x029100, "Broadcom BCM6348"          },
    { 0x02A000, "Broadcom BCM4350"          },
    { 0x020000, "Broadcom BCM6358"          },

    /* SiByte */
    { 0x040100, "SiByte SB1"                },
    { 0x041100, "SiByte SB1A"               },

    /* SandCraft */
    { 0x050400, "SandCraft SR71000"         },

    /* Cavium */
    { 0x0d0000, "Cavium CN38XX"             },
    { 0x0d0100, "Cavium CN31XX"             },
    { 0x0d0200, "Cavium CN30XX"             },
    { 0x0d0300, "Cavium CN58XX"             },
    { 0x0d0400, "Cavium CN56XX"             },
    { 0x0d0600, "Cavium CN50XX"             },
    { 0x0d0700, "Cavium CN52XX"             },

    /* Ingenic */
    { 0xd00200, "Ingenic JZRISC"            },
    { 0x000200, "Ingenic JZ4780"            },
    { 0xe10200, "Ingenic JZ4780"            },
    { 0 }
};

#if defined (__CYGWIN32__) || defined (MINGW32)
/*
 * Delay in milliseconds: Windows.
 */
#include <windows.h>

void mdelay (unsigned msec)
{
    Sleep (msec);
}
#else
/*
 * Delay in milliseconds: Unix.
 */
void mdelay (unsigned msec)
{
    usleep (msec * 1000);
}
#endif

/*
 * Interrupted by user (^C).
 * Close adapter connection and exit.
 */
static void target_sigint (int sig)
{
    printf ("\ninterrupted by user.\n");
    if (target_current) {
        target_current->adapter->close (target_current->adapter, 0);
    }
    exit (0);
}

/*
 * Save the state of CPU.
 */
static void target_save_state (target_t *t)
{
    static const unsigned code[] = {                /* start: */
        MIPS_MTC0 (2, 31, 0),                       /* move $2 to COP0 DeSave */
        MIPS_LUI (2, UPPER16(PRACC_PARAM_OUT)),     /* $2 = PRACC_PARAM_OUT */
        MIPS_ORI (2, 2, LOWER16(PRACC_PARAM_OUT)),
        MIPS_SW (0, 0*4, 2),                        /* sw $0,0*4($2) */
        MIPS_SW (1, 1*4, 2),                        /* sw $1,1*4($2) */
        MIPS_SW (15, 15*4, 2),                      /* sw $15,15*4($2) */
        MIPS_MFC0 (2, 31, 0),                       /* move COP0 DeSave to $2 */
        MIPS_MTC0 (15, 31, 0),                      /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),        /* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (1, 0, 15),                         /* sw $1,($15) */
        MIPS_SW (2, 0, 15),                         /* sw $2,($15) */
        MIPS_LUI (1, UPPER16(PRACC_PARAM_OUT)),     /* $1 = PRACC_PARAM_OUT */
        MIPS_ORI (1, 1, LOWER16(PRACC_PARAM_OUT)),
        MIPS_SW (2, 2*4, 1),                        /* sw $2,2*4($1) */
        MIPS_SW (3, 3*4, 1),                        /* sw $3,3*4($1) */
        MIPS_SW (4, 4*4, 1),                        /* sw $4,4*4($1) */
        MIPS_SW (5, 5*4, 1),                        /* sw $5,5*4($1) */
        MIPS_SW (6, 6*4, 1),                        /* sw $6,6*4($1) */
        MIPS_SW (7, 7*4, 1),                        /* sw $7,7*4($1) */
        MIPS_SW (8, 8*4, 1),                        /* sw $8,8*4($1) */
        MIPS_SW (9, 9*4, 1),                        /* sw $9,9*4($1) */
        MIPS_SW (10, 10*4, 1),                      /* sw $10,10*4($1) */
        MIPS_SW (11, 11*4, 1),                      /* sw $11,11*4($1) */
        MIPS_SW (12, 12*4, 1),                      /* sw $12,12*4($1) */
        MIPS_SW (13, 13*4, 1),                      /* sw $13,13*4($1) */
        MIPS_SW (14, 14*4, 1),                      /* sw $14,14*4($1) */
        MIPS_SW (16, 16*4, 1),                      /* sw $16,16*4($1) */
        MIPS_SW (17, 17*4, 1),                      /* sw $17,17*4($1) */
        MIPS_SW (18, 18*4, 1),                      /* sw $18,18*4($1) */
        MIPS_SW (19, 19*4, 1),                      /* sw $19,19*4($1) */
        MIPS_SW (20, 20*4, 1),                      /* sw $20,20*4($1) */
        MIPS_SW (21, 21*4, 1),                      /* sw $21,21*4($1) */
        MIPS_SW (22, 22*4, 1),                      /* sw $22,22*4($1) */
        MIPS_SW (23, 23*4, 1),                      /* sw $23,23*4($1) */
        MIPS_SW (24, 24*4, 1),                      /* sw $24,24*4($1) */
        MIPS_SW (25, 25*4, 1),                      /* sw $25,25*4($1) */
        MIPS_SW (26, 26*4, 1),                      /* sw $26,26*4($1) */
        MIPS_SW (27, 27*4, 1),                      /* sw $27,27*4($1) */
        MIPS_SW (28, 28*4, 1),                      /* sw $28,28*4($1) */
        MIPS_SW (29, 29*4, 1),                      /* sw $29,29*4($1) */
        MIPS_SW (30, 30*4, 1),                      /* sw $30,30*4($1) */
        MIPS_SW (31, 31*4, 1),                      /* sw $31,31*4($1) */

        MIPS_MFC0 (2, 12, 0),                       /* move status to $2 */
        MIPS_SW (2, 32*4, 1),                       /* sw $2,32*4($1) */
        MIPS_MFLO (2),                              /* move lo to $2 */
        MIPS_SW (2, 33*4, 1),                       /* sw $2,33*4($1) */
        MIPS_MFHI (2),                              /* move hi to $2 */
        MIPS_SW (2, 34*4, 1),                       /* sw $2,34*4($1) */
        MIPS_MFC0 (2, 8, 0),                        /* move badvaddr to $2 */
        MIPS_SW (2, 35*4, 1),                       /* sw $2,35*4($1) */
        MIPS_MFC0 (2, 13, 0),                       /* move cause to $2 */
        MIPS_SW (2, 36*4, 1),                       /* sw $2,36*4($1) */
        MIPS_MFC0 (2, 24, 0),                       /* move depc (pc) to $2 */
        MIPS_SW (2, 37*4, 1),                       /* sw $2,37*4($1) */

        MIPS_LW (2, 0, 15),                         /* lw $2,($15) */
        MIPS_LW (1, 0, 15),                         /* lw $1,($15) */
        MIPS_B (NEG16(58)),                         /* b start */
        MIPS_MFC0 (15, 31, 0),                      /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };

    t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code,
        0, 0, ARRAY_SIZE(t->reg), t->reg);

    if (! t->dcr_valid) {
        /* Get the parameters of debug block. */
        t->dcr = target_read_word (t, EJTAG_DCR);
        t->dcr_valid = 1;
        //fprintf (stderr, "DCR = %08x\n", t->dcr);

        if (t->dcr & DCR_INSTBRK) {
            /* Instruction breakpoints supported.
             * Get number of inst breakpoints.
             * Clear break status. */
            unsigned ibs = target_read_word (t, EJTAG_IBS);
            target_write_word (t, EJTAG_IBS, 0);

            t->nbpoints = (ibs >> 24) & 0x0F;
            //fprintf (stderr, "IBS = %08x, %u instruction breakpoints\n", ibs, t->nbpoints);
        }
        if (t->dcr & DCR_DATABRK) {
            /* Data breakpoints supported.
             * Get number of data breakpoints.
             * Clear break status. */
            unsigned dbs = target_read_word (t, EJTAG_DBS);
            target_write_word (t, EJTAG_DBS, 0);

            t->nwatchpoints = (dbs >> 24) & 0x0F;
            //fprintf (stderr, "DBS = %08x, %u watchpoints\n", dbs, t->nwatchpoints);
        }
        printf ("hardware: %u breakpoints, %u watchpoints\n",
            t->nbpoints, t->nwatchpoints);
    }
}

/*
 * Print settings of PIC32 oscillator.
 */
static void print_oscillator (unsigned osccon,
    unsigned devcfg1, unsigned devcfg2)
{
    static const short pllmult[]  = { 15, 16, 17, 18, 19, 20, 21, 24 };
    static const short pllidiv[]  = {  1,  2,  3,  4,  5,  6, 10, 12 };
    static const short pllodiv[]  = {  1,  2,  4,  8, 16, 32, 64, 256 };
    static const char *poscmod[] = { "external", "XT crystal",
                                     "HS crystal", "(disabled)" };

    /* COSC: current oscillator selection bits. */
    unsigned cosc = (osccon >> 12) & 7;
    printf ("oscillator: ");
    switch (cosc) {
    case 0:                             /* Fast RC oscillator */
        printf ("internal Fast RC\n");
        break;
    case 1:                             /* Fast RC with PLL */
        printf ("internal Fast RC, PLL div 1:%d:%d mult x%d\n",
                pllidiv [devcfg2 & 7],
                pllodiv [devcfg2 >> 16 & 7], pllmult [osccon >> 16 & 7]);
        break;
    case 2:                             /* Primary oscillator XT, HS, EC */
        printf ("%s\n", poscmod [devcfg1 >> 8 & 3]);
        break;
    case 3:                             /* Primary with PLL */
        printf ("%s, PLL div 1:%d:%d mult x%d\n",
                poscmod [devcfg1 >> 8 & 3], pllidiv [devcfg2 & 7],
                pllodiv [devcfg2 >> 16 & 7], pllmult [osccon >> 16 & 7]);
        break;
    case 4:                             /* Secondary oscillator */
        printf ("secondary\n");
        break;
    case 5:                             /* Low-power RC */
        printf ("internal Low-Power RC\n");
        break;
    case 6:                             /* Fast RC divided 1:16 */
        printf ("internal Fast RC, divided 1:16\n");
        break;
    case 7:                             /* Fast RC divided by FRCDIV */
        printf ("internal Fast RC, divided 1:%d\n",
                pllodiv [osccon >> 24 & 7]);
        break;
    }
}

/*
 * Set value of OSCCON register.
 */
#define REG_OSCCON      0xbf80f000

static unsigned set_osccon (target_t *t, unsigned osccon)
{
    int retry;

    static const unsigned code[] = {            /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),	/* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (7, 0, 15),			/* sw $7,($15) */
        MIPS_SW (8, 0, 15),			/* sw $8,($15) */
        MIPS_SW (9, 0, 15),			/* sw $9,($15) */
        MIPS_LW (7, NEG16(PRACC_STACK - PRACC_PARAM_IN), 15), /* load R7 @ param_in[0] = osccon */
        MIPS_LUI (9, 0xbf81),                   /* $9 = bf810000 */
        MIPS_LUI (8, 0xaa99),                   /* $8 = magic key1 */
        MIPS_ORI (8, 8, 0x6655),
        MIPS_SW (8, 0xf230, 9),			/* sw $8,SYSKEY($9) - key1 */
        MIPS_LUI (8, 0x5566),                   /* $8 = magic key2 */
        MIPS_ORI (8, 8, 0x99aa),
        MIPS_SW (8, 0xf230, 9),			/* sw $8,SYSKEY($9) - key2 */
        MIPS_SW (7, 0xf000, 9),			/* sw $7,OSCCON($9) */
        MIPS_ORI (7, 7, 1),
        MIPS_SW (7, 0xf000, 9),			/* sw $7,OSCCON($9) - set OSWEN */
        MIPS_SW (0, 0xf230, 9),			/* sw zero,SYSKEY($9) - lock */
        MIPS_LW (9, 0, 15),			/* lw $9,($15) */
        MIPS_LW (8, 0, 15),			/* lw $8,($15) */
        MIPS_LW (7, 0, 15),			/* lw $7,($15) */
        MIPS_B (NEG16(22)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };
    t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code,
        1, &osccon, 0, 0);

    /* Wait for OSWEN to become 0. */
    for (retry=3; retry>0; retry--) {
        osccon = target_read_word (t, REG_OSCCON);
        if (! (osccon & 1))
            break;
        if (debug_level > 0)
            printf ("OSCCON = %08x\n", osccon);
    }
    if (retry <= 0) {
        printf ("failed to change oscillator, OSCCON = %08x\n", osccon);
    }
    return osccon;
}

/*
 * Connect to JTAG adapter.
 * Must stop a processor, but avoid resetting it, if possible.
 */
target_t *target_open ()
{
    target_t *t;
    unsigned i;

    t = calloc (1, sizeof (target_t));
    if (! t) {
        fprintf (stderr, "Out of memory\n");
        return 0;
    }
    t->cpu_name = "Unknown";

    /* Find adapter. */
    t->adapter = adapter_open_pickit ();
#ifdef USE_MPSSE
    if (! t->adapter)
        t->adapter = adapter_open_mpsse ();
#endif
    if (! t->adapter) {
        //fprintf (stderr, "No target found.\n");
        return 0;
    }
    target_current = t;
    signal (SIGINT, target_sigint);

    /* Check CPU identifier. */
    t->cpuid = t->adapter->get_idcode (t->adapter);
    if (t->cpuid == 0 || t->cpuid == ~0) {
        /* Device not detected. */
        fprintf (stderr, "Bad CPUID=%08x.\n", t->cpuid);
        t->adapter->close (t->adapter, 0);
        return 0;
    }
    if ((t->cpuid & 0xfff) == 0x053) {
        /* Microchip PIC32. */
        t->is_pic32 = 1;
        for (i=0; (t->cpuid ^ microchip[i].devid) & 0x0fffffff; i++) {
            if (microchip[i].devid == 0) {
                /* Device not detected. */
                fprintf (stderr, "Unknown CPUID=%08x.\n", t->cpuid);
                t->adapter->close (t->adapter, 0);
                return 0;
            }
        }
        t->cpu_name = microchip[i].name;
        t->flash_kbytes = microchip[i].flash_kbytes;
        t->boot_kbytes = microchip[i].boot_kbytes;
        printf ("processor: %s\n", t->cpu_name);
    } else {
        /* Read IMPCODE. */
        t->impcode = t->adapter->get_impcode (t->adapter);
        if (t->impcode & 1)
            t->cpu_name = "MIPS64";
        else
            t->cpu_name = "MIPS32";
    }

    /* Stop the processor. */
    t->adapter->stop_cpu (t->adapter);
    target_save_state (t);

    if (! t->is_pic32) {
        /* Read COP0 PRId register. */
        t->prid = target_read_cop0_register (t, 15, 0);
        if (debug_level > 0)
            printf ("PRID = %08x\n", t->prid);
        for (i=0; (t->prid ^ devtab[i].devid) & 0x00ffff00; i++) {
            if (devtab[i].devid == 0) {
                /* Device not detected. */
                fprintf (stderr, "Unknown PRID=%08x.\n", t->prid);
                t->adapter->close (t->adapter, 0);
                return 0;
            }
        }
        t->cpu_name = devtab[i].name;
        printf ("processor: %s\n", t->cpu_name);
    }

    if (t->is_pic32) {
        /* PIC32-specific: identify oscillator. */
        unsigned osccon = target_read_word (t, REG_OSCCON);
        unsigned devcfg1 = target_read_word (t, 0xbfc00000 + t->boot_kbytes * 1024 - 8);
        unsigned devcfg2 = target_read_word (t, 0xbfc00000 + t->boot_kbytes * 1024 - 12);
        if (debug_level > 0) {
            printf ("OSCCON = %08x\n", osccon);
            printf ("DEVCFG1 = %08x\n", devcfg1);
            printf ("DEVCFG2 = %08x\n", devcfg2);
        }

        /* COSC: current oscillator selection. */
        unsigned cosc = (osccon >> 12) & 7;

        /* NOSC: new oscillator selection. */
        unsigned fnosc = devcfg1 & 7;
        if (cosc != fnosc) {
            /* Switch oscillator. */
            if (debug_level > 0) {
                print_oscillator (osccon, devcfg1, devcfg2);
                printf ("change oscillator from %u to %u\n", cosc, fnosc);
            }
            osccon &= ~(7 << 8);
            osccon |= fnosc << 8;
            osccon = set_osccon (t, osccon);
        }
        print_oscillator (osccon, devcfg1, devcfg2);
    }
    return t;
}

/*
 * Close the device.
 */
void target_close (target_t *t, int power_on)
{
    t->adapter->close (t->adapter, power_on);
    target_current = 0;
}

/*
 * Get name of target CPU.
 */
const char *target_cpu_name (target_t *t)
{
    return t->cpu_name;
}

/*
 * Get device ID of target CPU.
 */
unsigned target_idcode (target_t *t)
{
    return t->cpuid;
}

/*
 * Check that the address is in ROM region.
 */
int target_is_rom_address (target_t *t, unsigned addr)
{
    /* For Microchip PIC32MX.
     * TODO: put base addresses into microchip[]. */
    if (! t->is_pic32)
        return 0;

    if (addr >= 0x9d000000 && addr < 0x9d000000 + t->flash_kbytes * 1024)
        return 1;
    if (addr >= 0xbd000000 && addr < 0xbd000000 + t->flash_kbytes * 1024)
        return 1;
    if (addr >= 0xbfc00000 && addr < 0xbfc00000 + t->boot_kbytes * 1024)
        return 1;
    if (addr >= 0x9fc00000 && addr < 0x9fc00000 + t->boot_kbytes * 1024)
        return 1;
    return 0;
}

/*
 * Read a register:
 *      0-31  - GPRs
 *      32    - CP0 Status
 *      33    - LO
 *      34    - HI
 *      35    - CP0 BadVAddr
 *      36    - CP0 Cause
 *      37    - PC
 * FPU registers not supported for PIC32.
 */
unsigned target_read_register (target_t *t, unsigned regno)
{
    //fprintf (stderr, "target_read_register (regno = %u)\n", regno);
    if (regno < 32) {
        /* General purpose registers */
        return t->reg [regno];
    }
    switch (regno) {
    case REG_STATUS:
    case REG_LO:
    case REG_HI:
    case REG_BADVADDR:
    case REG_CAUSE:
    case REG_DEPC:
        return t->reg [regno];
    }
    return 0;
}

/*
 * The processor is running: stop it.
 */
void target_stop (target_t *t)
{
    if (! t->is_running)
        return;
    t->adapter->stop_cpu (t->adapter);
    t->is_running = 0;
    target_save_state (t);
}

/*
 * The processor is supposedly running: check if it has been stopped.
 * Return 1 when stopped or 0 if still running.
 */
int target_is_stopped (target_t *t, int *is_aborted)
{
    if (is_aborted != 0)
        *is_aborted = 0;

    /* Tiny delay. */
    mdelay (100);
    if (! t->adapter->cpu_stopped (t->adapter))
        return 0;

    /* Stopped. */
    if (t->is_running) {
        t->is_running = 0;
        target_save_state (t);
    }
#if 0
    /* TODO: BREAKD instruction detected. */
    if ((t->adapter->oscr & OSCR_SWO) && is_aborted != 0)
        *is_aborted = 1;
#endif
    return 1;
}

/*
 * The processor is stopped: let it run.
 */
void target_resume (target_t *t)
{
    static const unsigned code[] = {
        MIPS_DRET,                            /* return from debug mode */
        MIPS_NOP,
        MIPS_NOP,
    };

    if (t->is_running)
        return;
    t->is_running = 1;
    t->adapter->exec (t->adapter, 0, ARRAY_SIZE(code), code, 0, 0, 0, 0);
}

/*
 * The processor is stopped: start it from the given address.
 */
void target_run (target_t *t, unsigned addr)
{
    static const unsigned code[] = {
	MIPS_MTC0 (15, 31, 0),                      /* move $15 to COP0 DeSave */
	MIPS_LUI (15, UPPER16(PRACC_PARAM_IN)),     /* $15 = PRACC_PARAM_IN */
	MIPS_LW (15, LOWER16(PRACC_PARAM_IN), 15),  /* $15 = addr */
	MIPS_MTC0 (15, 24, 0),                      /* move $15 to COP0 DEPC */
	MIPS_MFC0 (15, 31, 0),                      /* move COP0 DeSave to $15 */
        MIPS_DRET,                                  /* return from debug mode */
        MIPS_NOP,
        MIPS_NOP,
    };

    if (t->is_running)
        return;

    t->is_running = 1;
    t->adapter->exec (t->adapter, 0, ARRAY_SIZE(code), code, 1, &addr, 0, 0);
}

/*
 * Restart the processor, using hardware reset.
 */
void target_restart (target_t *t)
{
    t->adapter->reset_cpu (t->adapter);
    t->is_running = 0;
    target_save_state (t);
}

/*
 * The processor is stopped: run it for one instruction only.
 */
void target_step (target_t *t)
{
    static const unsigned code_step[] = {
        MIPS_MTC0 (1, 31, 0),			/* move $1 to COP0 DeSave */
        MIPS_MFC0 (1, 23, 0),			/* move COP0 Debug to $1 */
        MIPS_ORI (1, 1, 0x0100),		/* set SSt bit in debug reg */
        MIPS_MTC0 (1, 23, 0),			/* move $1 to COP0 Debug */
        MIPS_MFC0 (1, 31, 0),			/* move COP0 DeSave to $1 */
        MIPS_DRET,                              /* return from debug mode */
        MIPS_NOP,
        MIPS_NOP,
    };
    static const unsigned code_step_disable[] = {
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),    /* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (1, 0, 15),			/* sw $1,($15) */
        MIPS_SW (2, 0, 15),			/* sw $2,($15) */
        MIPS_MFC0 (1, 23, 0),			/* move COP0 Debug to $1 */
        MIPS_LUI (2, 0xFFFF),			/* $2 = 0xfffffeff */
        MIPS_ORI (2, 2, 0xFEFF),
        MIPS_AND (1, 1, 2),
        MIPS_MTC0 (1, 23, 0),			/* move $1 to COP0 Debug */
        MIPS_LW (2, 0, 15),
        MIPS_LW (1, 0, 15),
        MIPS_B (NEG16(13)),
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };

    if (t->is_running)
        return;

    t->adapter->exec (t->adapter, 1,
        ARRAY_SIZE(code_step), code_step, 0, 0, 0, 0);
    target_save_state (t);

    t->adapter->exec (t->adapter, 1,
        ARRAY_SIZE(code_step_disable), code_step_disable, 0, 0, 0, 0);
}

/*
 * Read a word from memory.
 */
unsigned target_read_word (target_t *t, unsigned addr)
{
    static const unsigned code[] = {            /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),	/* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (8, 0, 15),			/* sw $8,($15) */

        MIPS_LW (8, NEG16(PRACC_STACK - PRACC_PARAM_IN), 15), /* load R8 @ param_in[0] = address */

        MIPS_LW (8, 0, 8),			/* lw $8,0($8), Load $8 with the word @mem[$8] */
        MIPS_SW (8, NEG16(PRACC_STACK - PRACC_PARAM_OUT), 15), /* store R8 @ param_out[0] */

        MIPS_LW (8, 0, 15),			/* lw $8,($15) */
        MIPS_B (NEG16(9)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };
    unsigned word;

    if (! t->adapter->exec (t->adapter, 1,
        ARRAY_SIZE(code), code, 1, &addr, 1, &word))
    {
        /* Exception: bad address. */
        fprintf (stderr, "ERROR: cannot read address %08x\n", addr);
        return 0;
    }
    return word;
}

/*
 * Read a chunk of data from memory.
 */
void target_read_block (target_t *t, unsigned addr,
    unsigned nwords, unsigned *data)
{
    if (nwords == 1) {
        *data = target_read_word (t, addr);
        return;
    }

    static const unsigned code[] = {            /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),	/* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (8, 0, 15),			/* sw $8,($15) */
        MIPS_SW (9, 0, 15),			/* sw $9,($15) */
        MIPS_SW (10, 0, 15),			/* sw $10,($15) */
        MIPS_SW (11, 0, 15),			/* sw $11,($15) */

        MIPS_LUI (8, UPPER16(PRACC_PARAM_IN)),  /* $8 = PRACC_PARAM_IN */
        MIPS_ORI (8, 8, LOWER16(PRACC_PARAM_IN)),
        MIPS_LW (9, 0, 8),			/* $9 = mem[$8]; read addr */
        MIPS_LW (10, 4, 8),			/* $10 = mem[$8 + 4]; read nwords */
        MIPS_LUI (11, UPPER16(PRACC_PARAM_OUT)),/* $11 = PRACC_PARAM_OUT */
        MIPS_ORI (11, 11, LOWER16(PRACC_PARAM_OUT)),
                                                /* loop: */
        MIPS_BEQ (0, 10, 8),			/* beq 0, $10, end */
        MIPS_NOP,

        MIPS_LW (8, 0, 9),			/* lw $8,0($9), Load $8 with the word @mem[$9] */
        MIPS_SW (8, 0, 11),			/* sw $8,0($11) */

        MIPS_ADDI (10, 10, NEG16(1)),		/* $10-- */
        MIPS_ADDI (9, 9, 4),			/* $1 += 4 */
        MIPS_ADDI (11, 11, 4),                  /* $11 += 4 */

        MIPS_B (NEG16(8)),			/* b loop */
        MIPS_NOP,
                                                /* end: */
        MIPS_LW (11, 0, 15),			/* lw $11,($15) */
        MIPS_LW (10, 0, 15),			/* lw $10,($15) */
        MIPS_LW (9, 0, 15),			/* lw $9,($15) */
        MIPS_LW (8, 0, 15),			/* lw $8,($15) */
        MIPS_B (NEG16(27)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };
    int n, nread;
    unsigned param_in[2];

    nread = 0;
    while (nwords > 0) {
        n = nwords;
        if (nwords > 0x400)
            n = 0x400;

        param_in[0] = addr;
        param_in[1] = n;

        if (! t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code,
            ARRAY_SIZE(param_in), param_in, n, &data[nread]))
        {
            /* Exception: bad address. */
            fprintf (stderr, "ERROR: cannot read address %08x\n", addr);
            memset (&data[nread], 0, 4*n);
        }

        nwords -= n;
        addr += n;
        nread += n;
    }
}

/*
 * Write a word to memory.
 */
void target_write_word (target_t *t, unsigned addr, unsigned word)
{
    static const unsigned code[] = {            /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),	/* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (8, 0, 15),			/* sw $8,($15) */
        MIPS_SW (9, 0, 15),			/* sw $9,($15) */

        MIPS_LW (8, NEG16((PRACC_STACK - PRACC_PARAM_IN) - 4), 15), /* load R8 @ param_in[1] = data */
        MIPS_LW (9, NEG16(PRACC_STACK - PRACC_PARAM_IN), 15), /* load R9 @ param_in[0] = address */

        MIPS_SW (8, 0, 9),			/* sw $8,0($9) */

        MIPS_LW (9, 0, 15),			/* lw $9,($15) */
        MIPS_LW (8, 0, 15),			/* lw $8,($15) */
        MIPS_B (NEG16(11)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };
    unsigned param_in [2];

    param_in[0] = addr;
    param_in[1] = word;
    if (! t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code,
        ARRAY_SIZE(param_in), param_in, 0, 0))
    {
        fprintf (stderr, "ERROR: cannot write %08x to address %08x\n", word, addr);
    }
}

/*
 * Write chunk of data to memory.
 */
void target_write_block (target_t *t, unsigned addr,
    unsigned nwords, unsigned *data)
{
    static const unsigned code[] = {            /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),	/* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (8, 0, 15),			/* sw $8,($15) */
        MIPS_SW (9, 0, 15),			/* sw $9,($15) */
        MIPS_SW (10, 0, 15),			/* sw $10,($15) */
        MIPS_SW (11, 0, 15),			/* sw $11,($15) */

        MIPS_ADDI (8, 15, NEG16(PRACC_STACK - PRACC_PARAM_IN)), /* $8= PRACC_PARAM_IN */
        MIPS_LW (9, 0, 8),			/* Load write addr to $9 */
        MIPS_LW (10, 4, 8),			/* Load write count to $10 */
        MIPS_ADDI (8, 8, 8),			/* $8 += 8 beginning of data */
                                                /* loop: */
        MIPS_LW (11, 0, 8),			/* lw $11,0($8), Load $11 with the word @mem[$8] */
        MIPS_SW (11, 0, 9),			/* sw $11,0($9) */

        MIPS_ADDI (9, 9, 4),			/* $9 += 4 */
        MIPS_BNE (10, 9, NEG16(4)),		/* bne $10, $9, loop */
        MIPS_ADDI (8, 8, 4),			/* $8 += 4 */
                                                /* end: */
        MIPS_LW (11, 0, 15),			/* lw $11,($15) */
        MIPS_LW (10, 0, 15),			/* lw $10,($15) */
        MIPS_LW (9, 0, 15),			/* lw $9,($15) */
        MIPS_LW (8, 0, 15),			/* lw $8,($15) */
        MIPS_B (NEG16(21)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };
    unsigned param_in [nwords + 2];

    if (nwords == 1) {
        target_write_word (t, addr, *data);
        return;
    }
    param_in[0] = addr;
    param_in[1] = addr + (nwords * sizeof(unsigned));	/* last address */
    memcpy (&param_in[2], data, nwords * sizeof(unsigned));

    if (! t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code,
        nwords + 2, param_in, 0, 0))
    {
        fprintf (stderr, "ERROR: cannot write %u words to address %08x\n", nwords, addr);
    }
}

/*
 * Write a register:
 *      0-31  - GPRs
 *      32    - CP0 Status
 *      33    - LO
 *      34    - HI
 *      35    - CP0 BadVAddr
 *      36    - CP0 Cause
 *      37    - PC
 * FPU registers not supported for PIC32.
 */
void target_write_register (target_t *t, unsigned regno, unsigned val)
{
    static unsigned code_gpr[] = {              /* start: */
        0,                                      /* lui regno, upper_val */
        MIPS_B (NEG16(2)),			/* b start */
        0,                                      /* ori regno, lower_val */
        MIPS_NOP,
        MIPS_NOP,
    };
    static unsigned code[] = {                  /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        0,                                      /* lui $15, upper_val */
        0,                                      /* ori $15, lower_val */
        0,                                      /* mtlo, mthi or mtc0 */
        MIPS_B (NEG16(5)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };

    if (regno < 32) {
        /* General purpose registers: special case. */
        t->reg [regno] = val;
        code_gpr [0] = MIPS_LUI (regno, UPPER16(val));
        code_gpr [2] = MIPS_ORI (regno, regno, LOWER16(val));
        t->adapter->exec (t->adapter, 1,
            ARRAY_SIZE(code_gpr), code_gpr, 0, 0, 0, 0);
        return;
    }
    code [1] = MIPS_LUI (15, UPPER16(val));
    code [2] = MIPS_ORI (15, 15, LOWER16(val));
    switch (regno) {
    case REG_STATUS:
        code [3] = MIPS_MTC0 (15, CP0_STATUS, 0);
        break;
    case REG_LO:
        code [3] = MIPS_MTLO (15);
        break;
    case REG_HI:
        code [3] = MIPS_MTHI (15);
        break;
    case REG_BADVADDR:
        code [3] = MIPS_MTC0 (15, CP0_BADVADDR, 0);
        break;
    case REG_CAUSE:
        code [3] = MIPS_MTC0 (15, CP0_CAUSE, 0);
        break;
    case REG_DEPC:
        code [3] = MIPS_MTC0 (15, CP0_DEPC, 0);
        break;
    default:
        return;
    }
    t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code, 0, 0, 0, 0);
    t->reg [regno] = val;
}

/*
 * Read COP0 register.
 */
unsigned target_read_cop0_register (target_t *t, unsigned regno, unsigned sel)
{
    static unsigned code[] = {                  /* start: */
        MIPS_MTC0 (15, 31, 0),                  /* move $15 to COP0 DeSave */
        MIPS_LUI (15, UPPER16(PRACC_STACK)),	/* $15 = PRACC_STACK */
        MIPS_ORI (15, 15, LOWER16(PRACC_STACK)),
        MIPS_SW (8, 0, 15),			/* sw $8,($15) */

        0,                                      /* mfc $8,regno,sel */
        MIPS_SW (8, NEG16(PRACC_STACK - PRACC_PARAM_OUT), 15), /* store R8 @ param_out[0] */

        MIPS_LW (8, 0, 15),			/* lw $8,($15) */
        MIPS_B (NEG16(8)),			/* b start */
        MIPS_MFC0 (15, 31, 0),                  /* move COP0 DeSave to $15 */
        MIPS_NOP,
        MIPS_NOP,
    };
    unsigned word;

    code [4] = MIPS_MFC0 (8, regno, sel);
    t->adapter->exec (t->adapter, 1, ARRAY_SIZE(code), code, 0, 0, 1, &word);
    return word;
}

/*
 * Set an instruction breakpoint or a data watchpoint.
 */
void target_add_break (target_t *t, unsigned addr, int type)
{
    int i, control, last;
    unsigned last_id;

    if (type == 'b') {
        /* Instruction breakpoint. */
        if (t->nbpoints <= 0)
            return;

        /* Find free slot, or the oldest used. */
        last = 0;
        last_id = t->bp_used[0];
        for (i=0; i<t->nbpoints; i++) {
            if (! t->bp_used[i])
                break;
            if (t->bp_used[i] < last_id) {
                last = i;
                last_id = t->bp_used[i];
            }
        }
        if (i >= t->nbpoints) {
            /* All breakpoints used.
             * Forget the oldest one. */
            i = last;
        }
        t->bp_used[i] = ++t->bp_id;
        t->bp_value[i] = addr;

	/* Enable breakpoint. */
        target_write_word (t, EJTAG_IBA(i), addr);
        target_write_word (t, EJTAG_IBM(i), 0x00000000);
        target_write_word (t, EJTAG_IBC(i), DBC_BE);
        if (debug_level > 0)
            fprintf (stderr, "target_add_break: set breakpoint #%d at %08x\n",
                     i, addr);
    } else {
        /* Data watchpoint. */
        if (t->nwatchpoints <= 0)
            return;

        /* Find free slot, or the oldest used. */
        last = 0;
        last_id = t->watch_used[0];
        for (i=0; i<t->nwatchpoints; i++) {
            if (! t->watch_used[i])
                break;
            if (t->watch_used[i] < last_id) {
                last = i;
                last_id = t->watch_used[i];
            }
        }
        if (i >= t->nwatchpoints) {
            /* All watchpoints used.
             * Forget the oldest one. */
            i = last;
        }
        t->watch_used[i] = ++t->watch_id;
        t->watch_value[i] = addr;

	/* Enable watchpoint, ignore all byte lanes in value register
	 * and exclude both load and store accesses from  watchpoint
	 * condition evaluation. */
	control = DBC_NOSB | DBC_NOLB | DBC_BE | (0xff << DBC_BLM_SHIFT);
        switch (type) {
        case 'r':
            control &= ~DBC_NOLB;
            break;
        case 'w':
            control &= ~DBC_NOSB;
            break;
        case 'a':
            control &= ~(DBC_NOLB | DBC_NOSB);
            break;
	}
        target_write_word (t, EJTAG_DBA(i), addr);
        target_write_word (t, EJTAG_DBM(i), 0x00000000);
        target_write_word (t, EJTAG_DBASID(i), 0x00000000);
        target_write_word (t, EJTAG_DBC(i), control);
        target_write_word (t, EJTAG_DBV(i), 0);
        if (debug_level > 0)
            fprintf (stderr, "target_add_break: set %c-watchpoint #%d at %08x\n",
                     type, i, addr);
    }
}

/*
 * Clear a breakpoint or watchpoint by address.
 */
void target_remove_break (target_t *t, unsigned addr)
{
    int i;

    for (i=0; i<t->nbpoints; i++) {
        if (t->bp_used[i] && t->bp_value[i] == addr) {
            /* Disable breakpoint. */
            target_write_word (t, EJTAG_IBC(i), 0);
            t->bp_used[i] = 0;
            if (debug_level > 0)
                fprintf (stderr, "target_remove_break: clear breakpoint #%d at %08x\n",
                         i, addr);
        }
    }
    for (i=0; i<t->nwatchpoints; i++) {
        if (t->watch_used[i] && t->watch_value[i] == addr) {
            /* Disable watchpoint. */
            target_write_word (t, EJTAG_DBC(i), 0);
            t->watch_used[i] = 0;
            if (debug_level > 0)
                fprintf (stderr, "target_remove_break: clear watchpoint #%d at %08x\n",
                         i, addr);
        }
    }
}
