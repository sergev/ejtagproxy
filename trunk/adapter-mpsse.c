/*
 * Interface to PIC32 JTAG port via FT2232-based USB adapter.
 * Supported hardware:
 * 1) Olimex ARM-USB-Tiny adapter
 * 2) Olimex ARM-USB-Tiny-H adapter
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
#include "mips.h"
#include "ejtag.h"

typedef struct {
    /* Common part */
    adapter_t adapter;

    /* Device handle for libusb. */
    usb_dev_handle *usbdev;

    /* Transmit buffer for MPSSE packet. */
    unsigned char output [256*16];
    int bytes_to_write;

    /* Receive buffer. */
    unsigned char input [64];
    int bytes_to_read;
    int bytes_per_word;
    unsigned long long fix_high_bit;
    unsigned long long high_byte_mask;
    unsigned long long high_bit_mask;
    unsigned high_byte_bits;

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
} mpsse_adapter_t;

/*
 * Identifiers of USB adapter.
 */
#define OLIMEX_VID              0x15ba
#define OLIMEX_ARM_USB_TINY     0x0004  /* ARM-USB-Tiny */
#define OLIMEX_ARM_USB_TINY_H   0x002a	/* ARM-USB-Tiny-H */

/*
 * USB endpoints.
 */
#define IN_EP                   0x02
#define OUT_EP                  0x81

/* Requests */
#define SIO_RESET               0 /* Reset the port */
#define SIO_MODEM_CTRL          1 /* Set the modem control register */
#define SIO_SET_FLOW_CTRL       2 /* Set flow control register */
#define SIO_SET_BAUD_RATE       3 /* Set baud rate */
#define SIO_SET_DATA            4 /* Set the data characteristics of the port */
#define SIO_POLL_MODEM_STATUS   5
#define SIO_SET_EVENT_CHAR      6
#define SIO_SET_ERROR_CHAR      7
#define SIO_SET_LATENCY_TIMER   9
#define SIO_GET_LATENCY_TIMER   10
#define SIO_SET_BITMODE         11
#define SIO_READ_PINS           12
#define SIO_READ_EEPROM         0x90
#define SIO_WRITE_EEPROM        0x91
#define SIO_ERASE_EEPROM        0x92

/* MPSSE commands. */
#define CLKWNEG                 0x01
#define BITMODE                 0x02
#define CLKRNEG                 0x04
#define LSB                     0x08
#define WTDI                    0x10
#define RTDO                    0x20
#define WTMS                    0x40

/*
 * Send a packet to USB device.
 */
static void bulk_write (mpsse_adapter_t *a, unsigned char *output, int nbytes)
{
    int bytes_written;

    if (debug_level > 1) {
        int i;
        fprintf (stderr, "usb bulk write %d bytes:", nbytes);
        for (i=0; i<nbytes; i++)
            fprintf (stderr, "%c%02x", i ? '-' : ' ', output[i]);
        fprintf (stderr, "\n");
    }
    bytes_written = usb_bulk_write (a->usbdev, IN_EP, (char*) output,
        nbytes, 1000);
    if (bytes_written < 0) {
        fprintf (stderr, "usb bulk write failed\n");
        exit (-1);
    }
    if (bytes_written != nbytes)
        fprintf (stderr, "usb bulk written %d bytes of %d",
            bytes_written, nbytes);
}

/*
 * If there are any data in transmit buffer -
 * send them to device.
 */
static void mpsse_flush_output (mpsse_adapter_t *a)
{
    int bytes_read, n;
    unsigned char reply [64];

    if (a->bytes_to_write <= 0)
        return;

    bulk_write (a, a->output, a->bytes_to_write);
    a->bytes_to_write = 0;
    if (a->bytes_to_read <= 0)
        return;

    /* Get reply. */
    bytes_read = 0;
    while (bytes_read < a->bytes_to_read) {
        n = usb_bulk_read (a->usbdev, OUT_EP, (char*) reply,
            a->bytes_to_read - bytes_read + 2, 2000);
        if (n < 0) {
            fprintf (stderr, "usb bulk read failed\n");
            exit (-1);
        }
        if (debug_level > 1) {
            if (n != a->bytes_to_read + 2)
                fprintf (stderr, "usb bulk read %d bytes of %d\n",
                    n, a->bytes_to_read - bytes_read + 2);
            else {
                int i;
                fprintf (stderr, "usb bulk read %d bytes:", n);
                for (i=0; i<n; i++)
                    fprintf (stderr, "%c%02x", i ? '-' : ' ', reply[i]);
                fprintf (stderr, "\n");
            }
        }
        if (n > 2) {
            /* Copy data. */
            memcpy (a->input + bytes_read, reply + 2, n - 2);
            bytes_read += n - 2;
        }
    }
    if (debug_level > 1) {
        int i;
        fprintf (stderr, "mpsse_flush_output received %d bytes:", a->bytes_to_read);
        for (i=0; i<a->bytes_to_read; i++)
            fprintf (stderr, "%c%02x", i ? '-' : ' ', a->input[i]);
        fprintf (stderr, "\n");
    }
    a->bytes_to_read = 0;
}

/*
 * Send a packet to FT2232 chip.
 * Create a series of commands for JTAG interface.
 */
static void mpsse_send (mpsse_adapter_t *a,
    unsigned tms_prolog_nbits, unsigned tms_prolog,
    unsigned tdi_nbits, unsigned long long tdi, int read_flag)
{
    unsigned tms_epilog_nbits = 0, tms_epilog = 0;

    if (tdi_nbits > 0) {
        /* We have some data; add generic prologue TMS 1-0-0
         * and epilogue TMS 1-0. */
        tms_prolog |= 1 << tms_prolog_nbits;
        tms_prolog_nbits += 3;
        tms_epilog = 1;
        tms_epilog_nbits = 2;
    }
    /* Check that we have enough space in output buffer.
     * Max size of one packet is 23 bytes (6+8+3+3+3). */
    if (a->bytes_to_write > sizeof (a->output) - 23)
        mpsse_flush_output (a);

    /* Prepare a packet of MPSSE commands. */
    if (tms_prolog_nbits > 0) {
        /* Prologue TMS, from 1 to 14 bits.
         * 4b - Clock Data to TMS Pin (no Read) */
        a->output [a->bytes_to_write++] = WTMS + BITMODE + CLKWNEG + LSB;
        if (tms_prolog_nbits < 8) {
            a->output [a->bytes_to_write++] = tms_prolog_nbits - 1;
            a->output [a->bytes_to_write++] = tms_prolog;
        } else {
            a->output [a->bytes_to_write++] = 7 - 1;
            a->output [a->bytes_to_write++] = tms_prolog & 0x7f;
            a->output [a->bytes_to_write++] = WTMS + BITMODE + CLKWNEG + LSB;
            a->output [a->bytes_to_write++] = tms_prolog_nbits - 7 - 1;
            a->output [a->bytes_to_write++] = tms_prolog >> 7;
        }
    }
    if (tdi_nbits > 0) {
        /* Data, from 1 to 64 bits. */
        if (tms_epilog_nbits > 0) {
            /* Last bit should be accompanied with signal TMS=1. */
            tdi_nbits--;
        }
        unsigned nbytes = tdi_nbits / 8;
        unsigned last_byte_bits = tdi_nbits & 7;
        if (read_flag) {
            a->high_byte_bits = last_byte_bits;
            a->fix_high_bit = 0;
            a->high_byte_mask = 0;
            a->bytes_per_word = nbytes;
            if (a->high_byte_bits > 0)
                a->bytes_per_word++;
            a->bytes_to_read += a->bytes_per_word;
        }
        if (nbytes > 0) {
            /* Whole bytes.
             * 39 - Clock Data Bytes In and Out LSB First
             * 19 - Clock Data Bytes Out LSB First (no Read) */
            a->output [a->bytes_to_write++] = read_flag ?
                (WTDI + RTDO + CLKWNEG + LSB) :
                (WTDI + CLKWNEG + LSB);
            a->output [a->bytes_to_write++] = nbytes - 1;
            a->output [a->bytes_to_write++] = (nbytes - 1) >> 8;
            while (nbytes-- > 0) {
                a->output [a->bytes_to_write++] = tdi;
                tdi >>= 8;
            }
        }
        if (last_byte_bits) {
            /* Last partial byte.
             * 3b - Clock Data Bits In and Out LSB First
             * 1b - Clock Data Bits Out LSB First (no Read) */
            a->output [a->bytes_to_write++] = read_flag ?
                (WTDI + RTDO + BITMODE + CLKWNEG + LSB) :
                (WTDI + BITMODE + CLKWNEG + LSB);
            a->output [a->bytes_to_write++] = last_byte_bits - 1;
            a->output [a->bytes_to_write++] = tdi;
            tdi >>= last_byte_bits;
            a->high_byte_mask = 0xffULL << (a->bytes_per_word - 1) * 8;
        }
        if (tms_epilog_nbits > 0) {
            /* Last bit (actually two bits).
             * 6b - Clock Data to TMS Pin with Read
             * 4b - Clock Data to TMS Pin (no Read) */
            tdi_nbits++;
            a->output [a->bytes_to_write++] = read_flag ?
                (WTMS + RTDO + BITMODE + CLKWNEG + LSB) :
                (WTMS + BITMODE + CLKWNEG + LSB);
            a->output [a->bytes_to_write++] = 1;
            a->output [a->bytes_to_write++] = tdi << 7 | 1 | tms_epilog << 1;
            tms_epilog_nbits--;
            tms_epilog >>= 1;
            if (read_flag) {
                /* Last bit wil come in next byte.
                 * Compute a mask for correction. */
                a->fix_high_bit = 0x40ULL << (a->bytes_per_word * 8);
                a->bytes_per_word++;
                a->bytes_to_read++;
            }
        }
        if (read_flag)
            a->high_bit_mask = 1ULL << (tdi_nbits - 1);
    }
    if (tms_epilog_nbits > 0) {
        /* Epiloque TMS, from 1 to 7 bits.
         * 4b - Clock Data to TMS Pin (no Read) */
        a->output [a->bytes_to_write++] = WTMS + BITMODE + CLKWNEG + LSB;
        a->output [a->bytes_to_write++] = tms_epilog_nbits - 1;
        a->output [a->bytes_to_write++] = tms_epilog;
    }
}

/*
 * Decode data, received from FT2232 chip.
 * Return a word of received data, up to 64 bits.
 */
static unsigned long long mpsse_fix_data (mpsse_adapter_t *a, unsigned long long word)
{
    unsigned long long fix_high_bit = word & a->fix_high_bit;
    //if (debug) fprintf (stderr, "fix (%08llx) high_bit=%08llx\n", word, a->fix_high_bit);

    if (a->high_byte_bits) {
        /* Fix a high byte of received data. */
        unsigned long long high_byte = a->high_byte_mask &
            ((word & a->high_byte_mask) >> (8 - a->high_byte_bits));
        word = (word & ~a->high_byte_mask) | high_byte;
        //if (debug) fprintf (stderr, "Corrected byte %08llx -> %08llx\n", a->high_byte_mask, high_byte);
    }
    word &= a->high_bit_mask - 1;
    if (fix_high_bit) {
        /* Fix a high bit of received data. */
        word |= a->high_bit_mask;
        //if (debug) fprintf (stderr, "Corrected bit %08llx -> %08llx\n", a->high_bit_mask, word >> 9);
    }
    return word;
}

/*
 * Receive data from FT2232 chip.
 * Return a received word, up to 64 bits.
 */
static unsigned long long mpsse_recv (mpsse_adapter_t *a)
{
    unsigned long long word;

    /* Send a packet. */
//a->output [a->bytes_to_write++] = 0x87;
    mpsse_flush_output (a);

    /* Process a reply: one 64-bit word. */
    memcpy (&word, a->input, sizeof (word));
    return mpsse_fix_data (a, word);
}

/*
 * Control /TRST, /SYSRST and LED hardware signals.
 */
static void mpsse_reset (mpsse_adapter_t *a, int trst, int sysrst, int led)
{
    unsigned char output [3];
    unsigned low_output = 0x08; /* TCK idle high */
    unsigned low_direction = 0x1b;
    unsigned high_direction = 0x0f;
    unsigned high_output = 0;

    /* command "set data bits low byte" */
    output [0] = 0x80;
    output [1] = low_output;
    output [2] = low_direction;
    bulk_write (a, output, 3);

    if (! trst)
        high_output |= 1;

    if (sysrst)
        high_output |= 2;

    if (led)
        high_output |= 8;

    /* command "set data bits high byte" */
    output [0] = 0x82;
    output [1] = high_output;
    output [2] = high_direction;

    bulk_write (a, output, 3);
    if (debug_level > 1)
        fprintf (stderr, "mpsse_reset (trst=%d, sysrst=%d) high_output=0x%2.2x, high_direction: 0x%2.2x\n",
            trst, sysrst, high_output, high_direction);
}

/*
 * Set a JTAG speed for FT2232 chip.
 */
static void mpsse_speed (mpsse_adapter_t *a, int divisor)
{
    unsigned char output [3];

    /* command "set TCK divisor" */
    output [0] = 0x86;
    output [1] = divisor;
    output [2] = divisor >> 8;
    bulk_write (a, output, 3);
}

/*
 * Close a JTAG connection and deallocate the data.
 */
static void mpsse_close (adapter_t *adapter, int power_on)
{
    mpsse_adapter_t *a = (mpsse_adapter_t*) adapter;

    /* Clear EJTAGBOOT mode. */
    mpsse_send (a, 1, 1, 5, TAP_SW_ETAP, 0);
    mpsse_send (a, 6, 31, 0, 0, 0);             /* TMS 1-1-1-1-1-0 */
    mpsse_flush_output (a);

    /* LED off. */
    mpsse_reset (a, 0, 0, 0);

    usb_release_interface (a->usbdev, 0);
    usb_close (a->usbdev);
    free (a);
}

/*
 * Read the Device Identification code.
 */
static unsigned mpsse_get_idcode (adapter_t *adapter)
{
    mpsse_adapter_t *a = (mpsse_adapter_t*) adapter;
    unsigned idcode;

    /* Reset the JTAG TAP controller: TMS 1-1-1-1-1-0.
     * After reset, the IDCODE register is always selected.
     * Read out 32 bits of data. */
    mpsse_send (a, 6, 31, 32, 0, 1);
    idcode = mpsse_recv (a);

    if (debug_level > 0)
        fprintf (stderr, "%s: idcode %08x\n", a->adapter.name, idcode);
    return idcode;
}

/*
 * Hardware reset.
 */
static void mpsse_reset_cpu (adapter_t *adapter)
{
    mpsse_adapter_t *a = (mpsse_adapter_t*) adapter;
    unsigned ctl;

    /* Set EJTAGBOOT mode - request a Debug exception on reset.
     * Clear a 'reset occured' flag. */
    ctl = (a->control & ~CONTROL_ROCC) | CONTROL_EJTAGBRK;

    mpsse_send (a, 6, 31, 0, 0, 0);                 /* TMS 1-1-1-1-1-0 */
    mpsse_send (a, 1, 1, 5, TAP_SW_ETAP, 0);
    mpsse_send (a, 1, 1, 5, ETAP_EJTAGBOOT, 0);     /* stop on boot vector */
    mpsse_send (a, 1, 1, 5, TAP_SW_MTAP, 0);
    mpsse_send (a, 1, 1, 5, MTAP_COMMAND, 0);
    mpsse_send (a, 0, 0, 8, MCHP_ASSERT_RST, 0);    /* toggle Reset */
    mpsse_send (a, 0, 0, 8, MCHP_DEASSERT_RST, 0);
    mpsse_send (a, 0, 0, 8, MCHP_FLASH_ENABLE, 0);
    mpsse_send (a, 1, 1, 5, TAP_SW_ETAP, 0);

    /* Set EjtagBrk bit - request a Debug exception.
     * Clear a 'reset occured' flag. */
    mpsse_send (a, 1, 1, 5, ETAP_CONTROL, 0);
    mpsse_send (a, 0, 0, 32, (a->control & ~CONTROL_ROCC) |
                             CONTROL_EJTAGBRK, 1);
    ctl = mpsse_recv (a);
    if (debug_level > 0)
        fprintf (stderr, "mpsse_reset_cpu: control = %08x\n", ctl);
    if (! (ctl & CONTROL_ROCC)) {
        fprintf (stderr, "mpsse_reset_cpu: reset failed\n");
        exit (-1);
    }
}

/*
 * Is the processor stopped?
 */
static int mpsse_cpu_stopped (adapter_t *adapter)
{
    mpsse_adapter_t *a = (mpsse_adapter_t*) adapter;
    unsigned ctl;

    /* Select Control Register. */
    mpsse_send (a, 1, 1, 5, ETAP_CONTROL, 0);

    /* Check if Debug Mode bit is set. */
    mpsse_send (a, 0, 0, 32, a->control, 1);
    ctl = mpsse_recv (a);
    return ctl & CONTROL_DM;
}

/*
 * Stop the processor.
 */
static void mpsse_stop_cpu (adapter_t *adapter)
{
    mpsse_adapter_t *a = (mpsse_adapter_t*) adapter;
    unsigned ctl = 0;

    /* Select Control Register. */
    mpsse_send (a, 1, 1, 5, ETAP_CONTROL, 0);

    /* Loop until Debug Mode entered. */
    while (! (ctl & CONTROL_DM)) {
        /* Set EjtagBrk bit - request a Debug exception.
         * Clear a 'reset occured' flag. */
        mpsse_send (a, 0, 0, 32, (a->control & ~CONTROL_ROCC) |
                                 CONTROL_EJTAGBRK, 1);
        ctl = mpsse_recv (a);
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
static void pracc_exec_read (mpsse_adapter_t *a, unsigned address)
{
    int offset;
    unsigned data;

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

    /* Send the data out */
    mpsse_send (a, 1, 1, 5, ETAP_DATA, 0);
    mpsse_send (a, 0, 0, 32, data, 0);

    /* Clear the access pending bit (let the processor eat!) */
    mpsse_send (a, 1, 1, 5, ETAP_CONTROL, 0);
    mpsse_send (a, 0, 0, 32, a->control & ~CONTROL_PRACC, 0);
    mpsse_flush_output (a);
}

/*
 * Perform a write CPU access in debug mode.
 * Get data from CPU.
 */
static void pracc_exec_write (mpsse_adapter_t *a, unsigned address)
{
    unsigned data;
    int offset;

    /* Get data */
    mpsse_send (a, 1, 1, 5, ETAP_DATA, 0);
    mpsse_send (a, 0, 0, 32, 0, 1);
    data = mpsse_recv (a);

    /* Clear access pending bit */
    mpsse_send (a, 1, 1, 5, ETAP_CONTROL, 0);
    mpsse_send (a, 0, 0, 32, a->control & ~CONTROL_PRACC, 0);
    mpsse_flush_output (a);

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
 */
static void mpsse_exec (adapter_t *adapter, int cycle,
    int code_len, const unsigned *code,
    int num_param_in, unsigned *param_in,
    int num_param_out, unsigned *param_out)
{
    mpsse_adapter_t *a = (mpsse_adapter_t*) adapter;
    unsigned ctl, address;
    int pass = 0;

    a->local_iparam = param_in;
    a->local_oparam = param_out;
    a->num_iparam = num_param_in;
    a->num_oparam = num_param_out;
    a->code = code;
    a->code_len = code_len;
    a->stack_offset = 0;

    for (;;) {
        /* Select Control register. */
        mpsse_send (a, 1, 1, 5, ETAP_CONTROL, 0);

	/* Wait for the PrAcc to become "1". */
        do {
            mpsse_send (a, 0, 0, 32, a->control, 1);
            ctl = mpsse_recv (a);
            if (debug_level > 1)
                fprintf (stderr, "exec: ctl = %08x\n", ctl);
        } while (! (ctl & CONTROL_PRACC));

        /* Read Address register. */
        mpsse_send (a, 1, 1, 5, ETAP_ADDRESS, 0);
        mpsse_send (a, 0, 0, 32, 0, 1);
        address = mpsse_recv (a);
        if (debug_level > 1)
            fprintf (stderr, "exec: address = %08x\n", address);

        /* Check for read or write */
        if (ctl & CONTROL_PRNW) {
            pracc_exec_write (a, address);
        } else {
            /* Check to see if its reading at the debug vector. The first pass through
             * the module is always read at the vector, so the first one we allow.  When
             * the second read from the vector occurs we are done and just exit. */
            if ((address == PRACC_TEXT) && (pass++)) {
                break;
            }
            pracc_exec_read (a, address);
        }

        if (cycle == 0)
            break;
    }

    /* Stack sanity check */
    if (a->stack_offset != 0) {
        fprintf (stderr, "%s: exec stack not zero = %d\n",
            a->adapter.name, a->stack_offset);
        exit (-1);
    }
}

/*
 * Initialize adapter F2232.
 * Return a pointer to a data structure, allocated dynamically.
 * When adapter not found, return 0.
 */
adapter_t *adapter_open_mpsse (void)
{
    mpsse_adapter_t *a;
    struct usb_bus *bus;
    struct usb_device *dev;
    char *name;

    usb_init();
    usb_find_busses();
    usb_find_devices();
    for (bus = usb_get_busses(); bus; bus = bus->next) {
        for (dev = bus->devices; dev; dev = dev->next) {
            if (dev->descriptor.idVendor == OLIMEX_VID &&
                dev->descriptor.idProduct == OLIMEX_ARM_USB_TINY) {
                name = "Olimex ARM-USB-Tiny";
                goto found;
            }
            if (dev->descriptor.idVendor == OLIMEX_VID &&
                dev->descriptor.idProduct == OLIMEX_ARM_USB_TINY_H) {
                name = "Olimex ARM-USB-Tiny-H";
                goto found;
            }
        }
    }
    //printf ("FT2232 adapter not found\n");
    return 0;
found:
    a = calloc (1, sizeof (*a));
    if (! a) {
        fprintf (stderr, "%s: out of memory\n", name);
        return 0;
    }
    a->adapter.name = name;
    a->usbdev = usb_open (dev);
    if (! a->usbdev) {
        fprintf (stderr, "%s: usb_open() failed\n", a->adapter.name);
        free (a);
        return 0;
    }
    usb_claim_interface (a->usbdev, 0);
    printf ("adapter: %s, id %04x:%04x\n", a->adapter.name,
        dev->descriptor.idVendor, dev->descriptor.idProduct);

    /* Reset the ftdi device. */
    if (usb_control_msg (a->usbdev,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
        SIO_RESET, 0, 1, 0, 0, 1000) != 0) {
        if (errno == EPERM)
            fprintf (stderr, "%s: superuser privileges needed.\n", a->adapter.name);
        else
            fprintf (stderr, "%s: FTDI reset failed\n", a->adapter.name);
failed:
        usb_release_interface (a->usbdev, 0);
        usb_close (a->usbdev);
        free (a);
        return 0;
    }

    /* MPSSE mode. */
    if (usb_control_msg (a->usbdev,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
        SIO_SET_BITMODE, 0x20b, 1, 0, 0, 1000) != 0) {
        fprintf (stderr, "%s: can't set sync mpsse mode\n", a->adapter.name);
        goto failed;
    }

    /* Optimal rate is 0.5 MHz.
     * Divide base oscillator 6 MHz by 12. */
    unsigned divisor = 12 - 1;
    unsigned char latency_timer = 2;
    if (dev->descriptor.idProduct == OLIMEX_ARM_USB_TINY_H)
        latency_timer = 0;

    if (usb_control_msg (a->usbdev,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
        SIO_SET_LATENCY_TIMER, latency_timer, 1, 0, 0, 1000) != 0) {
        fprintf (stderr, "%s: unable to set latency timer\n", a->adapter.name);
        goto failed;
    }
    if (usb_control_msg (a->usbdev,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
        SIO_GET_LATENCY_TIMER, 0, 1, (char*) &latency_timer, 1, 1000) != 1) {
        fprintf (stderr, "%s: unable to get latency timer\n", a->adapter.name);
        goto failed;
    }
    if (debug_level) {
    	fprintf (stderr, "%s: divisor: %u\n", a->adapter.name, divisor);
    	fprintf (stderr, "%s: latency timer: %u usec\n", a->adapter.name, latency_timer);
    }

    /* Light a LED. */
    mpsse_reset (a, 0, 0, 1);

    if (debug_level) {
        int baud = 6000000 / (divisor + 1);
        fprintf (stderr, "%s: speed %d samples/sec\n", a->adapter.name, baud);
    }
    mpsse_speed (a, divisor);

    /* Disable TDI to TDO loopback. */
    unsigned char enable_loopback[] = "\x85";
    bulk_write (a, enable_loopback, 1);

    /* Reset the JTAG TAP controller. */
    mpsse_send (a, 6, 31, 0, 0, 0);             /* TMS 1-1-1-1-1-0 */

    /* Check status. */
    mpsse_send (a, 1, 1, 5, TAP_SW_MTAP, 0);
    mpsse_send (a, 1, 1, 5, MTAP_COMMAND, 0);
    mpsse_send (a, 0, 0, 8, MCHP_STATUS, 1);
    unsigned status = mpsse_recv (a);
    if (debug_level > 0)
        fprintf (stderr, "%s: status %04x\n", a->adapter.name, status);
    if (status & MCHP_STATUS_DEVRST)
        fprintf (stderr, "%s: processor is in reset mode\n", a->adapter.name);
    if ((status & ~MCHP_STATUS_DEVRST) !=
        (MCHP_STATUS_CPS | MCHP_STATUS_CFGRDY | MCHP_STATUS_FAEN))
    {
        fprintf (stderr, "%s: invalid status = %04x\n", a->adapter.name, status);
        free (a);
        return 0;
    }

    /* Leave it in ETAP mode. */
    mpsse_send (a, 1, 1, 5, TAP_SW_ETAP, 0);
    mpsse_flush_output (a);
    a->control = CONTROL_ROCC | CONTROL_PRACC |
                 CONTROL_PROBEN | CONTROL_PROBTRAP;

    /* User functions. */
    a->adapter.close = mpsse_close;
    a->adapter.get_idcode = mpsse_get_idcode;
    a->adapter.cpu_stopped = mpsse_cpu_stopped;
    a->adapter.stop_cpu = mpsse_stop_cpu;
    a->adapter.reset_cpu = mpsse_reset_cpu;
    a->adapter.exec = mpsse_exec;
    return &a->adapter;
}
