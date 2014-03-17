/*
 * Interface to PIC32 ICSP port via Microchip PICkit2 or
 * PICkit3 USB adapter.
 *
 * To use PICkit3, you would need to upgrade the firmware
 * using free PICkit 3 Scripting Tool from Microchip.
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <usb.h>

#include "adapter.h"
#include "hidapi.h"
#include "pickit2.h"
#include "mips.h"
#include "ejtag.h"

typedef struct {
    /* Common part */
    adapter_t adapter;
    int is_pk3;

    /* Device handle for libusb. */
    hid_device *hiddev;

    unsigned char reply [64];

    /* EJTAG Control register. */
    unsigned control;

    /* Execution context. */
    unsigned *local_iparam;
    int num_iparam;
    unsigned *local_oparam;
    int num_oparam;
    const unsigned *code;
    int code_len;
    unsigned stack [32];
    int stack_offset;
} pickit_adapter_t;

/*
 * Programmable power voltages.
 */
#define VDD_VOLTAGE             3.3     /* Power supply */
#define VDD_LIMIT               2.81
#define VPP_VOLTAGE             3.28    /* Reset voltage */
#define VPP_LIMIT               2.26

/*
 * Identifiers of USB adapter.
 */
#define MICROCHIP_VID           0x04d8
#define PICKIT2_PID             0x0033  /* Microchip PICkit 2 */
#define PICKIT3_PID             0x900a  /* Microchip PICkit 3 */

#define WORD_AS_BYTES(w)  (unsigned char) (w), \
                          (unsigned char) ((w) >> 8), \
                          (unsigned char) ((w) >> 16), \
                          (unsigned char) ((w) >> 24)
/*
 * Send a 64-byte packet to USB HID device.
 * Print trace info, if needed.
 * Actual length of used data is given by `nbytes' argument.
 */
static void pickit_send_buf (pickit_adapter_t *a, unsigned char *buf, unsigned nbytes)
{
    if (debug_level > 1) {
        int k;
        fprintf (stderr, "---Send");
        for (k=0; k<nbytes; ++k) {
            if (k != 0 && (k & 15) == 0)
                fprintf (stderr, "\n       ");
            fprintf (stderr, " %02x", buf[k]);
        }
        fprintf (stderr, "\n");
    }
    hid_write (a->hiddev, buf, 64);
}

/*
 * Send a data packet to PICkit device.
 * Length of packet is given by `argc' argument.
 * Fill empty space by end-of-buffer value.
 */
static void pickit_send (pickit_adapter_t *a, unsigned argc, ...)
{
    va_list ap;
    unsigned i;
    unsigned char buf [64];

    memset (buf, CMD_END_OF_BUFFER, 64);
    va_start (ap, argc);
    for (i=0; i<argc; ++i)
        buf[i] = va_arg (ap, int);
    va_end (ap);
    pickit_send_buf (a, buf, i);
}

/*
 * Receive a data packet from USB HID device.
 * Place the data into a->reply[] array.
 * Print trace info, if needed.
 */
static void pickit_recv (pickit_adapter_t *a)
{
    if (hid_read (a->hiddev, a->reply, 64) != 64) {
        fprintf (stderr, "pickit: error receiving packet\n");
        exit (-1);
    }
    if (debug_level > 1) {
        int k;
        fprintf (stderr, "--->>>>");
        for (k=0; k<64; ++k) {
            if (k != 0 && (k & 15) == 0)
                fprintf (stderr, "\n       ");
            fprintf (stderr, " %02x", a->reply[k]);
        }
        fprintf (stderr, "\n");
    }
}

/*
 * Receive and check PICkit status.
 * In case of timeout, stop the program with fatal message.
 */
static void check_timeout (pickit_adapter_t *a, const char *message)
{
    unsigned status;

    pickit_send (a, 1, CMD_READ_STATUS);
    pickit_recv (a);
    status = a->reply[0] | a->reply[1] << 8;
    if (status & STATUS_ICD_TIMEOUT) {
        fprintf (stderr, "pickit: timed out at %s, status = %04x\n",
            message, status);
        exit (-1);
    }
}

/*
 * Hardware reset.
 */
static void pickit_reset_cpu (adapter_t *adapter)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    unsigned ctl;

    /* Set EJTAGBOOT mode - request a Debug exception on reset.
     * Clear a 'reset occured' flag. */
    ctl = (a->control & ~CONTROL_ROCC) | CONTROL_EJTAGBRK;
    pickit_send (a, 30, CMD_CLEAR_UPLOAD_BUFFER,
        CMD_EXECUTE_SCRIPT, 26,
            SCRIPT_JT2_SETMODE, 6, 0x1f,
            SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,
            SCRIPT_JT2_SENDCMD, ETAP_EJTAGBOOT,     // stop on boot vector
            SCRIPT_JT2_SENDCMD, TAP_SW_MTAP,
            SCRIPT_JT2_SENDCMD, MTAP_COMMAND,
            SCRIPT_JT2_XFERDATA8_LIT, MCHP_ASSERT_RST,
            SCRIPT_JT2_XFERDATA8_LIT, MCHP_DEASSERT_RST,
            SCRIPT_JT2_XFERDATA8_LIT, MCHP_FLASH_ENABLE,
            SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,    /* set ETAP mode */
            SCRIPT_JT2_SENDCMD, ETAP_CONTROL,   /* select Control Register */
            SCRIPT_JT2_XFERDATA32_LIT,
                WORD_AS_BYTES (ctl),            /* write/read Control reg */
        CMD_UPLOAD_DATA);
    pickit_recv (a);
    if (a->reply[0] != 7) {
        fprintf (stderr, "pickit_reset_cpu: bad reply length = %u\n", a->reply[0]);
        exit (-1);
    }
    ctl = a->reply[4] | (a->reply[5] << 8) |
          (a->reply[6] << 16) | (a->reply[7] << 24);
    if (debug_level > 0)
        fprintf (stderr, "pickit_reset_cpu: control = %08x\n", ctl);
    if (! (ctl & CONTROL_ROCC)) {
        fprintf (stderr, "pickit_reset_cpu: reset failed\n");
        exit (-1);
    }
}

/*
 * Finish the application:.
 * Clear debug mode and let the CPU run freely.
 */
static void pickit_finish (pickit_adapter_t *a, int power_on)
{
    /* Exit programming mode. */
    pickit_send (a, 18, CMD_CLEAR_UPLOAD_BUFFER, CMD_EXECUTE_SCRIPT, 15,
        SCRIPT_JT2_SETMODE, 5, 0x1f,
        SCRIPT_VPP_OFF,                         // disconnect /MCLR from power
        SCRIPT_MCLR_GND_ON,                     // connect /MCLR to ground
        SCRIPT_VPP_PWM_OFF,                     // disable power pump
        SCRIPT_SET_ICSP_PINS, 6,                // set PGC high, PGD input
        SCRIPT_SET_ICSP_PINS, 2,                // set PGC low, PGD input
        SCRIPT_SET_ICSP_PINS, 3,                // set PGC and PGD as input
        SCRIPT_DELAY_LONG, 10,                  // 50 msec
        SCRIPT_BUSY_LED_OFF);

    if (! power_on) {
        /* Detach power from the board. */
        pickit_send (a, 4, CMD_EXECUTE_SCRIPT, 2,
            SCRIPT_VDD_OFF,                     // disable power output
            SCRIPT_VDD_GND_ON);                 // add ballast
    }

    /* Disable reset. */
    pickit_send (a, 3, CMD_EXECUTE_SCRIPT, 1,
        SCRIPT_MCLR_GND_OFF);                   // detach /MCLR from ground

    /* Read board status. */
    check_timeout (a, "finish");
}

/*
 * Close a JTAG connection and deallocate the data.
 */
static void pickit_close (adapter_t *adapter, int power_on)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    //fprintf (stderr, "pickit: close\n");

    pickit_finish (a, power_on);
    free (a);
}

/*
 * Read the Device Identification code.
 */
static unsigned pickit_get_idcode (adapter_t *adapter)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    unsigned idcode;

    /* Read device id. */
    pickit_send (a, 13, CMD_CLEAR_UPLOAD_BUFFER,
        CMD_EXECUTE_SCRIPT, 9,
            SCRIPT_JT2_SENDCMD, TAP_SW_MTAP,
            SCRIPT_JT2_SENDCMD, MTAP_IDCODE,
            SCRIPT_JT2_XFERDATA32_LIT, 0, 0, 0, 0,
        CMD_UPLOAD_DATA);
    pickit_recv (a);
    //fprintf (stderr, "pickit: read id, %d bytes: %02x %02x %02x %02x\n",
    //  a->reply[0], a->reply[1], a->reply[2], a->reply[3], a->reply[4]);
    if (a->reply[0] != 4)
        return 0;
    idcode = a->reply[1] | a->reply[2] << 8 | a->reply[3] << 16 | a->reply[4] << 24;
    return idcode;
}

/*
 * Read the Device Implementation register.
 */
static unsigned pickit_get_impcode (adapter_t *adapter)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    unsigned impcode;

    /* Read device id. */
    pickit_send (a, 13, CMD_CLEAR_UPLOAD_BUFFER,
        CMD_EXECUTE_SCRIPT, 9,
            SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,
            SCRIPT_JT2_SENDCMD, ETAP_IMPCODE,
            SCRIPT_JT2_XFERDATA32_LIT, 0, 0, 0, 0,
        CMD_UPLOAD_DATA);
    pickit_recv (a);
    //fprintf (stderr, "pickit: read impcode, %d bytes: %02x %02x %02x %02x\n",
    //  a->reply[0], a->reply[1], a->reply[2], a->reply[3], a->reply[4]);
    if (a->reply[0] != 4)
        return 0;
    impcode = a->reply[1] | a->reply[2] << 8 | a->reply[3] << 16 | a->reply[4] << 24;
    return impcode;
}

/*
 * Is the processor stopped?
 */
static int pickit_cpu_stopped (adapter_t *adapter)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    unsigned ctl;

    pickit_send (a, 13, CMD_CLEAR_UPLOAD_BUFFER,
        CMD_EXECUTE_SCRIPT, 9,
            SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,    /* set ETAP mode */
            SCRIPT_JT2_SENDCMD, ETAP_CONTROL,   /* select Control Register */
            SCRIPT_JT2_XFERDATA32_LIT,
                WORD_AS_BYTES (a->control),     /* write/read Control reg */
        CMD_UPLOAD_DATA);
    pickit_recv (a);
    if (a->reply[0] != 4) {
        fprintf (stderr, "pickit_cpu_stopped: bad reply length = %u\n", a->reply[0]);
        exit (-1);
    }
    ctl = a->reply[1] | (a->reply[2] << 8) |
          (a->reply[3] << 16) | (a->reply[4] << 24);
    if (debug_level > 0)
        fprintf (stderr, "is_stopped: control = %08x\n", ctl);

    /* Check if Debug Mode bit is set. */
    return ctl & CONTROL_DM;
}

/*
 * Stop the processor.
 */
static void pickit_stop_cpu (adapter_t *adapter)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    unsigned ctl = 0;

    /* Loop until Debug Mode entered. */
    while (! (ctl & CONTROL_DM)) {
        /* Set EjtagBrk bit - request a Debug exception.
         * Clear a 'reset occured' flag. */
        ctl = (a->control & ~CONTROL_ROCC) | CONTROL_EJTAGBRK;
        pickit_send (a, 13, CMD_CLEAR_UPLOAD_BUFFER,
            CMD_EXECUTE_SCRIPT, 9,
                SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,    /* set ETAP mode */
                SCRIPT_JT2_SENDCMD, ETAP_CONTROL,   /* select Control Register */
                SCRIPT_JT2_XFERDATA32_LIT,
                    WORD_AS_BYTES (ctl),            /* write/read Control reg */
            CMD_UPLOAD_DATA);
        pickit_recv (a);
        if (a->reply[0] != 4) {
            fprintf (stderr, "pickit_stop_cpu: bad reply length = %u\n", a->reply[0]);
            exit (-1);
        }
        ctl = a->reply[1] | (a->reply[2] << 8) |
              (a->reply[3] << 16) | (a->reply[4] << 24);
        if (debug_level > 0)
            fprintf (stderr, "stop_cpu: control = %08x\n", ctl);

        if (ctl & CONTROL_ROCC) {
            fprintf (stderr, "processor: reset occured\n");
        }
    }
}

/*
 * Perform a read CPU access in debug mode.
 * Send the data out.
 */
static void pracc_exec_read (pickit_adapter_t *a, unsigned address)
{
    int offset;
    unsigned data, ctl;

    if ((address >= PRACC_PARAM_IN) &&
        (address <= PRACC_PARAM_IN + a->num_iparam * 4))
    {
        offset = (address - PRACC_PARAM_IN) / 4;
        data = a->local_iparam [offset];

    } else if ((address >= PRACC_PARAM_OUT) &&
               (address <= PRACC_PARAM_OUT + a->num_oparam * 4))
    {
        offset = (address - PRACC_PARAM_OUT) / 4;
        data = a->local_oparam [offset];

    } else if ((address >= PRACC_TEXT) &&
               (address <= PRACC_TEXT + a->code_len * 4))
    {
        offset = (address - PRACC_TEXT) / 4;
        data = a->code [offset];

    } else if (address == PRACC_STACK)
    {
        /* save to our debug stack */
        offset = --a->stack_offset;
        data = a->stack [offset];
    } else
    {
        fprintf (stderr, "%s: error reading unexpected address %08x\n",
            a->adapter.name, address);
        exit (-1);
    }
    if (debug_level > 1)
        fprintf (stderr, "exec: read address %08x -> %08x\n", address, data);

    /* Send the data out.
     * Clear the access pending bit (let the processor eat!) */
    ctl = a->control & ~CONTROL_PRACC;
    pickit_send (a, 17, CMD_CLEAR_UPLOAD_BUFFER,
        CMD_EXECUTE_SCRIPT, 14,
            SCRIPT_JT2_SENDCMD, ETAP_DATA,      /* write Data */
            SCRIPT_JT2_XFERDATA32_LIT,
                WORD_AS_BYTES (data),
            SCRIPT_JT2_SENDCMD, ETAP_CONTROL,   /* select Control Register */
            SCRIPT_JT2_XFERDATA32_LIT,
                WORD_AS_BYTES (ctl));           /* write/read Control reg */
}

/*
 * Perform a write CPU access in debug mode.
 * Get data from CPU.
 */
static void pracc_exec_write (pickit_adapter_t *a, unsigned address)
{
    unsigned data;
    int offset;

    /* Get data.
     * Clear access pending bit. */
    pickit_send (a, 5, CMD_CLEAR_UPLOAD_BUFFER,
        CMD_EXECUTE_SCRIPT, 1,
            SCRIPT_JT2_GET_PE_RESP,
        CMD_UPLOAD_DATA);
    pickit_recv (a);
    if (a->reply[0] != 4) {
        fprintf (stderr, "pickit pracc_exec_write: bad reply length = %u\n", a->reply[0]);
        exit (-1);
    }
    data = a->reply[1] | (a->reply[2] << 8) |
           (a->reply[3] << 16) | (a->reply[4] << 24);

    if ((address >= PRACC_PARAM_IN) &&
        (address <= PRACC_PARAM_IN + a->num_iparam * 4))
    {
        offset = (address - PRACC_PARAM_IN) / 4;
        a->local_iparam [offset] = data;

    } else if ((address >= PRACC_PARAM_OUT) &&
               (address <= PRACC_PARAM_OUT + a->num_oparam * 4))
    {
        offset = (address - PRACC_PARAM_OUT) / 4;
        a->local_oparam [offset] = data;

    } else if (address == PRACC_STACK)
    {
        /* save data onto our stack */
        a->stack [a->stack_offset++] = data;
    } else
    {
        fprintf (stderr, "%s: error writing unexpected address %08x\n",
            a->adapter.name, address);
        exit (-1);
    }
    if (debug_level > 1)
        fprintf (stderr, "exec: write address %08x := %08x\n", address, data);
}

/*
 * Execute a codelet.
 * The processor is in debug mode.  Every next instruction to execute is
 * supplied via EJTAG block.  Input and output data are mapped to
 * special regions in debug memory segment.  A separate stack region
 * exists for temporary storage.
 * Return zero, when failed (stack disbalanced).
 */
static int pickit_exec (adapter_t *adapter, int stay_in_debug_mode,
    int code_len, const unsigned *code,
    int num_param_in, unsigned *param_in,
    int num_param_out, unsigned *param_out)
{
    pickit_adapter_t *a = (pickit_adapter_t*) adapter;
    unsigned ctl, address;
    int access_count = 0, poll_count;

    a->local_iparam = param_in;
    a->local_oparam = param_out;
    a->num_iparam = num_param_in;
    a->num_oparam = num_param_out;
    a->code = code;
    a->code_len = code_len;
    a->stack_offset = 0;

    for (;;) {
	/* Wait for the PrAcc to become "1". */
        for (poll_count=0; ; poll_count++) {
            /* Write/read Control register. */
            pickit_send (a, 20, CMD_CLEAR_UPLOAD_BUFFER,
                CMD_EXECUTE_SCRIPT, 16,
                    SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,    /* set ETAP mode */
                    SCRIPT_JT2_SENDCMD, ETAP_CONTROL,   /* select Control Register */
                    SCRIPT_JT2_XFERDATA32_LIT,
                        WORD_AS_BYTES (a->control),     /* write/read Control reg */
                    SCRIPT_JT2_SENDCMD, ETAP_ADDRESS,   /* select Address Register */
                    SCRIPT_JT2_XFERDATA32_LIT,
                        0, 0, 0, 0,                     /* read Address reg */
                CMD_UPLOAD_DATA);
            pickit_recv (a);
            if (a->reply[0] != 8) {
                fprintf (stderr, "pickit_exec: bad ctl reply length = %u\n", a->reply[0]);
                exit (-1);
            }
            ctl = a->reply[1] | (a->reply[2] << 8) |
                  (a->reply[3] << 16) | (a->reply[4] << 24);
            if (debug_level > 1)
                fprintf (stderr, "exec: ctl = %08x\n", ctl);
            if (ctl & CONTROL_PRACC)
                break;

            /* No pending access from the processor. */
            if (! stay_in_debug_mode && poll_count > 2) {
                goto done;
            }
        }

        /* Get Address register. */
        address = a->reply[5] | (a->reply[6] << 8) |
              (a->reply[7] << 16) | (a->reply[8] << 24);
        if (debug_level > 1)
            fprintf (stderr, "exec: address = %08x\n", address);

        /* Check for read or write */
        if (ctl & CONTROL_PRNW) {
            pracc_exec_write (a, address);
        } else {
            /* Check to see if its reading at the debug vector. The first pass through
             * the module is always read at the vector, so the first one we allow.  When
             * the second read from the vector occurs we are done and just exit. */
            if (address == PRACC_TEXT && access_count > 0) {
                break;
            }
            pracc_exec_read (a, address);
        }
        access_count++;
    }
done:
    /* Stack sanity check */
    return (a->stack_offset == 0);
}

/*
 * Initialize adapter PICkit2/PICkit3.
 * Return a pointer to a data structure, allocated dynamically.
 * When adapter not found, return 0.
 * Unfortunately, on any connect (or disconnect) to ICSP port.
 * the processor needs to be reset.
 */
adapter_t *adapter_open_pickit (void)
{
    pickit_adapter_t *a;
    hid_device *hiddev;
    int is_pk3 = 0;

    hiddev = hid_open (MICROCHIP_VID, PICKIT2_PID, 0);
    if (! hiddev) {
        hiddev = hid_open (MICROCHIP_VID, PICKIT3_PID, 0);
        if (! hiddev) {
            /*fprintf (stderr, "HID bootloader not found: vid=%04x, pid=%04x\n",
                MICROCHIP_VID, BOOTLOADER_PID);*/
            return 0;
        }
        is_pk3 = 1;
    }
    a = calloc (1, sizeof (*a));
    if (! a) {
        fprintf (stderr, "Out of memory\n");
        return 0;
    }
    a->hiddev = hiddev;
    a->is_pk3 = is_pk3;
    a->adapter.name = is_pk3 ? "PICkit3" : "PICkit2";
    a->control = CONTROL_ROCC | CONTROL_PRACC |
                 CONTROL_PROBEN | CONTROL_PROBTRAP;

    /* Read version of adapter. */
    unsigned vers_major, vers_minor, vers_rev;
    if (a->is_pk3) {
        pickit_send (a, 2, CMD_GETVERSIONS_MPLAB, 0);
        pickit_recv (a);
        if (a->reply[30] != 'P' ||
            a->reply[31] != 'k' ||
            a->reply[32] != '3')
        {
            free (a);
            fprintf (stderr, "Incompatible PICkit3 firmware detected.\n");
            fprintf (stderr, "Please, upgrade the firmware using PICkit 3 Scripting Tool.\n");
            return 0;
        }
        vers_major = a->reply[33];
        vers_minor = a->reply[34];
        vers_rev = a->reply[35];
    } else {
        pickit_send (a, 2, CMD_CLEAR_UPLOAD_BUFFER, CMD_GET_VERSION);
        pickit_recv (a);
        vers_major = a->reply[0];
        vers_minor = a->reply[1];
        vers_rev = a->reply[2];
    }
    printf ("adapter: %s Version %d.%d.%d\n",
        a->adapter.name, vers_major, vers_minor, vers_rev);

    /* Detach power from the board. */
    pickit_send (a, 4, CMD_EXECUTE_SCRIPT, 2,
        SCRIPT_VDD_OFF,                         // disable power output
        SCRIPT_VDD_GND_ON);                     // add ballast

    /* Setup power voltage 3.3V, fault limit 2.81V. */
    if (a->is_pk3) {
        /* PICkit 3 */
        unsigned vdd = (unsigned) (VDD_VOLTAGE * 8 + 2.5);
        pickit_send (a, 3, CMD_SET_VDD, vdd, vdd >> 8);
    } else {
        /* PICkit 2 */
        unsigned vdd = (unsigned) (VDD_VOLTAGE * 32 + 10.5) << 6;
        unsigned vdd_limit = (unsigned) ((VDD_LIMIT / 5) * 255);
        pickit_send (a, 4, CMD_SET_VDD, vdd, vdd >> 8, vdd_limit);
    }

    /* Setup reset voltage 3.28V, fault limit 2.26V. */
    if (a->is_pk3) {
        /* PICkit 3 */
        unsigned vpp = (unsigned) (VPP_VOLTAGE * 8 + 2.5);
        pickit_send (a, 3, CMD_SET_VPP, vpp, vpp >> 8);
    } else {
        /* PICkit 2 */
        unsigned vpp = (unsigned) (VPP_VOLTAGE * 18.61);
        unsigned vpp_limit = (unsigned) (VPP_LIMIT * 18.61);
        pickit_send (a, 4, CMD_SET_VPP, 0x40, vpp, vpp_limit);
    }

    /* Setup serial speed as 8MHz/divisor. */
    unsigned divisor = 10;
    pickit_send (a, 4, CMD_EXECUTE_SCRIPT, 2,
        SCRIPT_SET_ICSP_SPEED, divisor);

    /* Reset active low. */
    pickit_send (a, 3, CMD_EXECUTE_SCRIPT, 1,
        SCRIPT_MCLR_GND_ON);                    // connect /MCLR to ground

    /* Read board status. */
    pickit_send (a, 2, CMD_CLEAR_UPLOAD_BUFFER, CMD_READ_STATUS);
    pickit_recv (a);
    unsigned status = a->reply[0] | a->reply[1] << 8;
    if (debug_level > 0)
        fprintf (stderr, "pickit: status %04x\n", status);

    switch (status & ~(STATUS_RESET | STATUS_BUTTON_PRESSED)) {
    case STATUS_VPP_GND_ON:
    case STATUS_VPP_GND_ON | STATUS_VPP_ON:
        /* Explorer 16 board: no need to enable power. */
        break;

    case STATUS_VDD_GND_ON | STATUS_VDD_ON | STATUS_VPP_GND_ON:
    case STATUS_VDD_GND_ON | STATUS_VDD_ON | STATUS_VPP_GND_ON | STATUS_VPP_ON:
        /* Microstick II board: no need to enable power. */
        break;

    case STATUS_VDD_GND_ON | STATUS_VPP_GND_ON:
        /* Enable power to the board. */
        pickit_send (a, 4, CMD_EXECUTE_SCRIPT, 2,
            SCRIPT_VDD_GND_OFF,                 // disable ballast
            SCRIPT_VDD_ON);                     // enable power output

        /* Read board status. */
        pickit_send (a, 2, CMD_CLEAR_UPLOAD_BUFFER, CMD_READ_STATUS);
        pickit_recv (a);
        status = a->reply[0] | a->reply[1] << 8;
        if (debug_level > 0)
            fprintf (stderr, "pickit: status %04x\n", status);
        if (status != (STATUS_VDD_ON | STATUS_VPP_GND_ON)) {
            fprintf (stderr, "pickit: invalid status = %04x.\n", status);
            return 0;
        }
        /* Wait for power to stabilize. */
        mdelay (200);
        break;

    default:
        fprintf (stderr, "pickit: invalid status = %04x\n", status);
        return 0;
    }

    /* Enter programming mode. */
    pickit_send (a, 52, CMD_CLEAR_UPLOAD_BUFFER, CMD_EXECUTE_SCRIPT, 49,
        SCRIPT_VPP_OFF,                         // disconnect /MCLR from power
        SCRIPT_MCLR_GND_ON,                     // connect /MCLR to ground
        SCRIPT_VPP_PWM_ON,                      // enable power pump
        SCRIPT_BUSY_LED_ON,
        SCRIPT_SET_ICSP_PINS, 0,                // set PGC and PGD output low
        SCRIPT_DELAY_LONG, 20,                  // 100 msec
        SCRIPT_MCLR_GND_OFF,                    // detach /MCLR from ground
        SCRIPT_VPP_ON,                          // connect /MCLR to power
        SCRIPT_DELAY_SHORT, 23,                 // 1 msec
        SCRIPT_VPP_OFF,                         // disconnect /MCLR from power
        SCRIPT_MCLR_GND_ON,                     // connect /MCLR to ground
        SCRIPT_DELAY_SHORT, 47,                 // 2 msec
        SCRIPT_WRITE_BYTE_LITERAL, 0xb2,        // magic word
        SCRIPT_WRITE_BYTE_LITERAL, 0xc2,
        SCRIPT_WRITE_BYTE_LITERAL, 0x12,
        SCRIPT_WRITE_BYTE_LITERAL, 0x0a,
        SCRIPT_MCLR_GND_OFF,                    // detach /MCLR from ground
        SCRIPT_VPP_ON,                          // connect /MCLR to power
        SCRIPT_DELAY_LONG, 2,                   // 10 msec
        SCRIPT_SET_ICSP_PINS, 2,                // set PGC low, PGD input
        SCRIPT_JT2_SETMODE, 6, 0x1f,
        SCRIPT_JT2_SENDCMD, TAP_SW_ETAP,
        SCRIPT_JT2_SENDCMD, ETAP_EJTAGBOOT,     // stop on boot vector
        SCRIPT_JT2_SENDCMD, TAP_SW_MTAP,
        SCRIPT_JT2_SENDCMD, MTAP_COMMAND,
        SCRIPT_JT2_XFERDATA8_LIT, MCHP_ASSERT_RST,
        SCRIPT_JT2_XFERDATA8_LIT, MCHP_DEASSERT_RST,
        SCRIPT_JT2_XFERDATA8_LIT, MCHP_FLASH_ENABLE,
        SCRIPT_JT2_XFERDATA8_LIT, MCHP_STATUS);
    pickit_send (a, 1, CMD_UPLOAD_DATA);
    pickit_recv (a);
    if (debug_level > 1)
        fprintf (stderr, "pickit: got %02x-%02x\n", a->reply[0], a->reply[1]);
    if (a->reply[0] != 4) {
        fprintf (stderr, "pickit: cannot get MCHP STATUS\n");
        pickit_finish (a, 0);
        return 0;
    }
    if (! (a->reply[3] & MCHP_STATUS_CFGRDY)) {
        fprintf (stderr, "No device attached.\n");
        pickit_finish (a, 0);
        return 0;
    }
    if (! (a->reply[3] & MCHP_STATUS_CPS)) {
        fprintf (stderr, "WARNING: device is code protected.\n");
    }

    /* User functions. */
    a->adapter.close = pickit_close;
    a->adapter.get_idcode = pickit_get_idcode;
    a->adapter.get_impcode = pickit_get_impcode;
    a->adapter.cpu_stopped = pickit_cpu_stopped;
    a->adapter.stop_cpu = pickit_stop_cpu;
    a->adapter.reset_cpu = pickit_reset_cpu;
    a->adapter.exec = pickit_exec;
    return &a->adapter;
}
