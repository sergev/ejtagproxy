/*
 * MIPS EJTAG definitions.
 *
 * Copyright (C) 2011-2012 Serge Vakulenko
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

/*
 * TAP instructions (5-bit).
 */
#define TAP_SW_MTAP     4       // Switch to MCHP TAP controller
#define TAP_SW_ETAP     5       // Switch to EJTAG TAP controller

/*
 * EJTAG-specific instructions.
 */
#define ETAP_IDCODE     1       // Device identification
#define ETAP_IMPCODE    3       // Implementation register
#define ETAP_ADDRESS    8       // Select Address register
#define ETAP_DATA       9       // Select Data register
#define ETAP_CONTROL    10      // Select EJTAG Control register
#define ETAP_ALL        11      // Select Address, Data and Control registers
#define ETAP_EJTAGBOOT  12      // On reset, take debug exception
#define ETAP_NORMALBOOT 13      // On reset, enter reset handler
#define ETAP_FASTDATA   14      // Select FastData register

/*
 * Microchip-specific instructions.
 */
#define MTAP_IDCODE     1       // Select chip identification register
#define MTAP_COMMAND    7       // Connect to MCHP command register

/*
 * Ingenic-specific instructions.
 */
#define ITAP_EN_CORE0   20      // Select core 0
#define ITAP_EN_CORE1   21      // Select core 1
#define ITAP_EN_CORE2   22      // Select core 2
#define ITAP_EN_CORE3   23      // Select core 3

/*
 * Microchip DR commands (8-bit).
 */
#define MCHP_STATUS        0x00 // Return Status
#define MCHP_ASSERT_RST    0xD1 // Assert device reset
#define MCHP_DEASSERT_RST  0xD0 // Remove device reset
#define MCHP_ERASE         0xFC // Flash chip erase
#define MCHP_FLASH_ENABLE  0xFE // Enable access from CPU to flash
#define MCHP_FLASH_DISABLE 0xFD // Disable access from CPU to flash

/*
 * Microchip status value (8-bit).
 */
#define MCHP_STATUS_CPS    0x80 // Device is NOT code-protected
#define MCHP_STATUS_NVMERR 0x20 // Error occured during NVM operation
#define MCHP_STATUS_CFGRDY 0x08 // Configuration has been read and
                                // Code-Protect State bit is valid
#define MCHP_STATUS_FCBUSY 0x04 // Flash Controller is Busy (erase is in progress)
#define MCHP_STATUS_FAEN   0x02 // Flash access is enabled
#define MCHP_STATUS_DEVRST 0x01 // Device reset is active

/*
 * EJTAG Control register.
 */
#define CONTROL_ROCC            (1 << 31)   /* Reset occured */
#define CONTROL_PSZ_MASK        (3 << 29)   /* Size of pending access */
#define CONTROL_PSZ_BYTE        (0 << 29)   /* Byte */
#define CONTROL_PSZ_HALFWORD    (1 << 29)   /* Half-word */
#define CONTROL_PSZ_WORD        (2 << 29)   /* Word */
#define CONTROL_PSZ_TRIPLE      (3 << 29)   /* Triple, double-word */
#define CONTROL_VPED            (1 << 23)   /* VPE disabled */
#define CONTROL_DOZE            (1 << 22)   /* Processor in low-power mode */
#define CONTROL_HALT            (1 << 21)   /* System bus clock stopped */
#define CONTROL_PERRST          (1 << 20)   /* Peripheral reset applied */
#define CONTROL_PRNW            (1 << 19)   /* Store access */
#define CONTROL_PRACC           (1 << 18)   /* Pending processor access */
#define CONTROL_RDVEC           (1 << 17)   /* Relocatable debug exception vector */
#define CONTROL_PRRST           (1 << 16)   /* Processor reset applied */
#define CONTROL_PROBEN          (1 << 15)   /* Probe will service processor accesses */
#define CONTROL_PROBTRAP        (1 << 14)   /* Debug vector at ff200200 */
#define CONTROL_EJTAGBRK        (1 << 12)   /* Debug interrupt exception */
#define CONTROL_DM              (1 << 3)    /* Debug mode */

/*
 * Debug memory segment.
 */
#define PRACC_FASTDATA_AREA	0xFF200000
#define PRACC_FASTDATA_SIZE	16
#define PRACC_TEXT		0xFF200200
#define PRACC_STACK		0xFF204000
#define PRACC_PARAM_IN		0xFF201000
#define PRACC_PARAM_IN_SIZE	0x1000
#define PRACC_PARAM_OUT		(PRACC_PARAM_IN + PRACC_PARAM_IN_SIZE)
#define PRACC_PARAM_OUT_SIZE	0x1000

/*
 * Breakpoint support registers.
 */
#define EJTAG_DCR	0xFF300000
#define EJTAG_IBS	0xFF301000
#define EJTAG_IBA(n)	(0xFF301100 + ((n) << 8))
#define EJTAG_IBM(n)	(0xFF301108 + ((n) << 8))
#define EJTAG_IBASID(n)	(0xFF301110 + ((n) << 8))
#define EJTAG_IBC(n)	(0xFF301118 + ((n) << 8))
#define EJTAG_IBCC(n)	(0xFF301120 + ((n) << 8))
#define EJTAG_IBPC(n)	(0xFF301128 + ((n) << 8))
#define EJTAG_DBS	0xFF302000
#define EJTAG_DBA(n)	(0xFF302100 + ((n) << 8))
#define EJTAG_DBM(n)	(0xFF302108 + ((n) << 8))
#define EJTAG_DBASID(n)	(0xFF302110 + ((n) << 8))
#define EJTAG_DBC(n)	(0xFF302118 + ((n) << 8))
#define EJTAG_DBV(n)	(0xFF302120 + ((n) << 8))
#define EJTAG_DBCC(n)	(0xFF302128 + ((n) << 8))
#define EJTAG_DBPC(n)	(0xFF302130 + ((n) << 8))

/*
 * DCR - Debug Control Register.
 */
#define DCR_ENM         (1 << 29)
#define DCR_DATABRK     (1 << 17)
#define DCR_INSTBRK     (1 << 16)
#define DCR_INTE        (1 << 4)

/*
 * DBC - Debug Breakpoint Control.
 */
#define DBC_NOSB        (1 << 13)
#define DBC_NOLB        (1 << 12)
#define DBC_BLM_MASK    0xff
#define DBC_BLM_SHIFT   4
#define DBC_BE          (1 << 0)
