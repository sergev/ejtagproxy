/*
 * Interface to MIPS target platform.
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
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <getopt.h>
#include <usb.h>

#include "gdbproxy.h"
#include "target.h"
#include "mips.h"

/*
 * Description of MIPS32 architecture.
 * Register numbering should match GDB.
 */
#define RP_MIPS_MIN_ADDRESS             0x0U
#define RP_MIPS_MAX_ADDRESS             0xFFFFFFFF
#define RP_MIPS_NUM_REGS                72
#define RP_MIPS_REG_BYTES               (RP_MIPS_NUM_REGS*sizeof(unsigned))
#define RP_MIPS_REGNUM_SP               29 /* Stack pointer */
#define RP_MIPS_REGNUM_FP               30 /* Frame pointer */
#define RP_MIPS_REGNUM_STATUS           32 /* Processor status */
#define RP_MIPS_REGNUM_LO               33
#define RP_MIPS_REGNUM_HI               34
#define RP_MIPS_REGNUM_BADVADDR         35
#define RP_MIPS_REGNUM_CAUSE            36
#define RP_MIPS_REGNUM_PC               37 /* Program counter */
#define RP_MIPS_REGNUM_FP0              38 /* FPU registers */
#define RP_MIPS_REGNUM_FCSR             70 /* FPU status */
#define RP_MIPS_REGNUM_FIR              71 /* FPU implementation information */

/*
 * Primitives of selected target (EJTAG).
 */
static int  mips_open (int argc, char * const argv[],
                        const char *prog_name, log_func log_fn);
static void mips_close (void);
static int  mips_connect (char *status_string,
                        size_t status_string_size, int *can_restart);
static int  mips_disconnect (void);
static void mips_kill (void);
static int  mips_restart (void);
static void mips_stop (void);
static int  mips_set_gen_thread (rp_thread_ref *thread);
static int  mips_set_ctrl_thread (rp_thread_ref *thread);
static int  mips_is_thread_alive (rp_thread_ref *thread, int *alive);
static int  mips_read_registers (uint8_t *data_buf, uint8_t *avail_buf,
                        size_t buf_size, size_t *read_size);
static int  mips_write_registers (uint8_t *data_buf, size_t write_size);
static int  mips_read_single_register (unsigned int reg_no,
                        uint8_t *data_buf, uint8_t *avail_buf,
                        size_t buf_size, size_t *read_size);
static int  mips_write_single_register (unsigned int reg_no,
                        uint8_t *data_buf, size_t write_size);
static int  mips_read_mem (uint64_t addr, uint8_t *data_buf,
                        size_t req_size, size_t *actual_size);
static int  mips_write_mem (uint64_t addr, uint8_t *data_buf,
                        size_t req_sise);
static int  mips_resume_from_current (int step, int sig);
static int  mips_resume_from_addr (int step, int sig, uint64_t addr);
static int  mips_go_waiting (int sig);
static int  mips_wait_partial (int first, char *status_string,
                        size_t status_string_len, out_func out,
                        int *implemented, int *more);
static int  mips_wait (char *status_string, size_t status_string_len,
                        out_func out, int *implemented);
static int  mips_process_query (unsigned int *mask,
                        rp_thread_ref *arg, rp_thread_info *info);
static int  mips_list_query (int first, rp_thread_ref *arg,
                        rp_thread_ref *result, size_t max_num,
                        size_t *num, int *done);
static int  mips_current_thread_query (rp_thread_ref *thread);
static int  mips_offsets_query (uint64_t *text,
                        uint64_t *data, uint64_t *bss);
static int  mips_crc_query (uint64_t addr, size_t len, uint32_t *val);
static int  mips_raw_query (char *in_buf, char *out_buf,
                        size_t out_buf_size);
static int  mips_remcmd (char *in_buf, out_func of, data_func df);
static int  mips_add_break (int type, uint64_t addr, unsigned int len);
static int  mips_remove_break (int type, uint64_t addr, unsigned int len);

/*
 * Remove commands, specific for a target platform.
 */
static int mips_rcmd_help (int argc, char *argv[], out_func of, data_func df);

#define RCMD(name, hlp) {#name, mips_rcmd_##name, hlp}  //table entry generation

/*
 * Data structure for remote commands.
 */
typedef struct {
    const char *name;                                   // command name
    int (*function)(int, char**, out_func, data_func);  // function to call
    const char *help;                                   // one line of help text
} RCMD_TABLE;

/*
 * Global descriptor of target platform.
 */
rp_target mips_target = {
    NULL,       /* next */
    "mips",
    "MIPS processor with EJTAG port",
    NULL,       /* help */
    mips_open,
    mips_close,
    mips_connect,
    mips_disconnect,
    mips_kill,
    mips_restart,
    mips_stop,
    mips_set_gen_thread,
    mips_set_ctrl_thread,
    mips_is_thread_alive,
    mips_read_registers,
    mips_write_registers,
    mips_read_single_register,
    mips_write_single_register,
    mips_read_mem,
    mips_write_mem,
    mips_resume_from_current,
    mips_resume_from_addr,
    mips_go_waiting,
    mips_wait_partial,
    mips_wait,
    mips_process_query,
    mips_list_query,
    mips_current_thread_query,
    mips_offsets_query,
    mips_crc_query,
    mips_raw_query,
    mips_remcmd,
    mips_add_break,
    mips_remove_break
};

static struct {
    /* Start up parameters, set by mips_open */
    log_func    log;
    target_t    *device;
} target;

/* Local functions */
static char *mips_out_treg (char *in, unsigned int reg_no);

#if 1
#define DEBUG_OUT(...)
#else
#define DEBUG_OUT printf
#endif

/*
 * Target method.
 * Parse user supplied options.
 * Called at start and on every gdb disconnect.
 */
static int mips_open(int argc,
                         char * const argv[],
                         const char *prog_name,
                         log_func log_fn)
{
    /* Option descriptors */
    static struct option long_options[] =
    {
        /* Options setting flag */
        {NULL, 0, 0, 0}
    };

    assert (prog_name != NULL);
    assert (log_fn != NULL);

    /* Set log */
    target.log = log_fn;

    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_open()",
                        mips_target.name);

    /* Process options */
    for (;;) {
        int c;
        int option_index;

        c = getopt_long(argc, argv, "+", long_options, &option_index);
        if (c == EOF)
            break;
        switch (c) {
        case 0:
            /* Long option which just sets a flag */
            break;
        default:
            target.log(RP_VAL_LOGLEVEL_NOTICE,
                                "%s: Use `%s --help' to see a complete list of options",
                                mips_target.name,
                                prog_name);
            return RP_VAL_TARGETRET_ERR;
        }
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Called when debugger disconnects from the proxy.
 * Resume a processor and close adapter connection.
 */
static void mips_close()
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_close()",
                        mips_target.name);

    if (target.device != 0) {
        target_resume (target.device);
        target_close (target.device, 0);
        target.device = 0;
    }
}

/*
 * Target method.
 * Called when debugger connects to the proxy.
 * Should stop a processor.
 */
static int mips_connect(char *status_string,
                            size_t status_string_len,
                            int *can_restart)
{
    char *cp;

    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_connect()",
                        mips_target.name);
    assert (status_string != NULL);
    assert (status_string_len >= 34);
    assert (can_restart != NULL);

    *can_restart = TRUE;

    if (! target.device) {
        /* First time - connect to adapter.
         * Must stop a processor! */
        target.device = target_open();
        if (! target.device) {
            target.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: target board not found.  Check cable connection!",
                            mips_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    /* Fill out the the status string */
    sprintf(status_string, "T%02d", RP_SIGNAL_BREAKPOINT);

    cp = mips_out_treg(&status_string[3], RP_MIPS_REGNUM_PC);
    cp = mips_out_treg(cp, RP_MIPS_REGNUM_FP);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Called on debugger detach command.
 */
static int mips_disconnect()
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_disconnect()",
                        mips_target.name);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Kill the target debug session.
 */
static void mips_kill()
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_kill()",
                        mips_target.name);
}

static int mips_restart()
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_restart()",
                        mips_target.name);
    assert (target.device != 0);

    target_restart (target.device);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method: Stop (i,e, break) the target program.
 */
static void mips_stop()
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_stop()",
                        mips_target.name);
    assert (target.device != 0);

    target_stop (target.device);
}

static int mips_set_gen_thread(rp_thread_ref *thread)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_set_gen_thread()",
                        mips_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method
 */
static int mips_set_ctrl_thread(rp_thread_ref *thread)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_set_ctrl_thread()",
                        mips_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method
 */
static int mips_is_thread_alive(rp_thread_ref *thread, int *alive)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_is_thread_alive()",
                        mips_target.name);

    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Read values of all registers.
 */
static int mips_read_registers(uint8_t *data_buf,
                                 uint8_t *avail_buf,
                                 size_t buf_size,
                                 size_t *read_size)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_read_registers()",
                        mips_target.name);
    assert (target.device != 0);
    assert (data_buf != NULL);
    assert (avail_buf != NULL);
    assert (buf_size >= RP_MIPS_REG_BYTES);
    assert (read_size != NULL);

    /* If target is running, stop it to get registers. */
    int was_running = ! target_is_stopped (target.device, 0);
    if (was_running) {
        target_stop (target.device);
    }

    /* Show only CPU and CP0 registers.
     * A debugger will ask for FPU registers separately. */
    int i;
    for (i=0; i<RP_MIPS_REGNUM_FP0; i++) {
        unsigned val = target_read_register (target.device, i);
        memcpy (data_buf + i*sizeof(unsigned), &val, sizeof(unsigned));
        memset (avail_buf + i*sizeof(unsigned), 1, sizeof(unsigned));
    }
    *read_size = RP_MIPS_REG_BYTES;

    if (was_running) {
        target_resume (target.device);
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Write the registers to the target.
 */
static int mips_write_registers(uint8_t *buf, size_t write_size)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_write_registers()",
                        mips_target.name);
    assert (target.device != 0);
    assert (buf != NULL);
    assert (write_size > 0);
    assert (write_size <= RP_MIPS_REG_BYTES);

    int i;
    for (i=0; i<RP_MIPS_NUM_REGS; i++) {
        unsigned offset = i * sizeof(unsigned);
        if (offset + sizeof(unsigned) > write_size)
            break;
        unsigned val = *(unsigned*) (buf + offset);
        target_write_register (target.device, i, val);
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Read a value of single register.
 */
static int mips_read_single_register(unsigned int reg_no,
                                         uint8_t *data_buf,
                                         uint8_t *avail_buf,
                                         size_t buf_size,
                                         size_t *read_size)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_read_single_register (%d)",
                        mips_target.name, reg_no);
    assert (target.device != 0);
    assert (data_buf != NULL);
    assert (avail_buf != NULL);
    assert (buf_size >= RP_MIPS_REG_BYTES);
    assert (read_size != NULL);

    if (reg_no >= RP_MIPS_NUM_REGS ||
        ! target_is_stopped (target.device, 0))
    {
        /* Register is not readable. */
        memset (data_buf, 0, sizeof(unsigned));
        memset (avail_buf, 0, sizeof(unsigned));
        *read_size = sizeof(unsigned);
        return RP_VAL_TARGETRET_OK;
    }

    unsigned val = target_read_register (target.device, reg_no);

    memcpy (data_buf, &val, sizeof(unsigned));
    memset (avail_buf, 1, sizeof(unsigned));
    *read_size = sizeof(unsigned);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 */
static int mips_write_single_register(unsigned int reg_no,
                                          uint8_t *buf,
                                          size_t write_size)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_write_single_register (%d, 0x%X)",
                        mips_target.name,
                        reg_no, *(unsigned*) buf);
    assert (target.device != 0);
    assert (buf != NULL);
    assert (write_size == 4);

    if (reg_no < 0 || reg_no >= RP_MIPS_NUM_REGS)
        return RP_VAL_TARGETRET_ERR;

    unsigned val = *(unsigned*) buf;
//target.log(RP_VAL_LOGLEVEL_DEBUG, "  write 0x%X to reg%d\n", val, reg_no);
    target_write_register (target.device, reg_no, val);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Read a requested number of bytes from device memory.
 */
static int mips_read_mem(uint64_t addr,
                             uint8_t *buf,
                             size_t req_size,
                             size_t *actual_size)
{
//    target.log(RP_VAL_LOGLEVEL_DEBUG,
//        "%s: mips_read_mem(0x%llX, ptr, %d, ptr)",
//        mips_target.name, addr, req_size);
    assert (target.device != 0);
    assert (buf != NULL);
    assert (req_size > 0);
    assert (actual_size != NULL);

    if (addr > RP_MIPS_MAX_ADDRESS) {
        target.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            mips_target.name,
                            addr);

        return RP_VAL_TARGETRET_ERR;
    }

    if (addr + req_size > RP_MIPS_MAX_ADDRESS + 1ULL)
        req_size = RP_MIPS_MAX_ADDRESS + 1ULL - addr;
    *actual_size = req_size;

    unsigned offset = (addr & 3);
    if (offset != 0) {
        /* Address is not aligned. */
        unsigned data;
        unsigned nbytes = 4 - offset;
        if (nbytes > req_size)
            nbytes = req_size;
        data = target_read_word (target.device, addr & ~3);
        switch (offset) {
        case 1:
            buf[0] = data >> 8;
            if (nbytes > 1)
                buf[1] = data >> 16;
            if (nbytes > 2)
                buf[2] = data >> 24;
            break;
        case 2:
            buf[0] = data >> 16;
            if (nbytes > 1)
                buf[1] = data >> 24;
            break;
        case 3:
            buf[0] = data >> 24;
            break;
        }
        req_size -= nbytes;
        if (req_size <= 0)
            return RP_VAL_TARGETRET_OK;
        addr += nbytes;
        buf += nbytes;
    }
    if (req_size >= 4) {
        /* Array of words. */
        unsigned nwords = req_size/4;
        target_read_block (target.device, addr, nwords, (unsigned*) buf);
        req_size &= 3;
        if (req_size <= 0)
            return RP_VAL_TARGETRET_OK;
        addr += nwords * 4;
        buf += nwords * 4;
    }
    if (req_size > 0) {
        /* Last word, partial. */
        unsigned data = target_read_word (target.device, addr);
        memcpy (buf, (unsigned char*) &data, req_size);
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Write a requested number of bytes to device memory.
 */
static int mips_write_mem(uint64_t addr,
                              uint8_t *buf,
                              size_t write_size)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_write_mem(0x%llX, ptr, %d)",
                        mips_target.name,
                        addr,
                        write_size);
    assert (target.device != 0);
    assert (buf != NULL);

    /* GDB does zero length writes for some reason. Treat them harmlessly. */
    if (write_size == 0)
        return RP_VAL_TARGETRET_OK;

    if (addr > RP_MIPS_MAX_ADDRESS)
    {
        target.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            mips_target.name,
                            addr);
        return RP_VAL_TARGETRET_ERR;
    }

    if ((addr + write_size - 1) > RP_MIPS_MAX_ADDRESS)
    {
        target.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address/write_size 0x%llx/0x%x",
                            mips_target.name,
                            addr,
                            write_size);
        return RP_VAL_TARGETRET_ERR;
    }
    unsigned offset = (addr & 3);

    if (offset == 0 && write_size == 4 &&
        target_is_rom_address (target.device, addr))
    {
        /* Software breakpoint: use hardware breakpoint instead. */
        unsigned opcode = *(uint32_t*) buf;

        if ((opcode & MIPS_BREAK_MASK) == MIPS_BREAK)
            target_add_break (target.device, addr, 'b');
        else
            target_remove_break (target.device, addr);
        return RP_VAL_TARGETRET_OK;
    }
    if (offset != 0) {
        /* Nonaligned address.
         * Read a word and construct the value. */
        unsigned data;
        unsigned nbytes = 4 - offset;
        if (nbytes > write_size)
            nbytes = write_size;
        data = target_read_word (target.device, addr & ~3);
        switch (offset) {
        case 1:
            data &= ~0x0000ff00;
            data |= buf[0] << 8;
            if (nbytes > 1) {
                data &= ~0x00ff0000;
                data |= buf[1] << 16;
            }
            if (nbytes > 2) {
                data &= ~0xff000000;
                data |= buf[2] << 24;
            }
            break;
        case 2:
            data &= ~0x00ff0000;
            data |= buf[0] << 16;
            if (nbytes > 1) {
                data &= ~0xff000000;
                data |= buf[1] << 24;
            }
            break;
        case 3:
            data &= ~0xff000000;
            data |= buf[0] << 24;
            break;
        }
        target_write_word (target.device, addr & ~3, data);
        write_size -= nbytes;
        if (write_size <= 0)
            return RP_VAL_TARGETRET_OK;
        addr += nbytes;
        buf += nbytes;
    }
    if (write_size >= 4) {
        /* Array of words. */
        unsigned nwords = write_size/4;
        target_write_block (target.device, addr, nwords, (unsigned*) buf);
        write_size &= 3;
        if (write_size <= 0)
            return RP_VAL_TARGETRET_OK;
        addr += nwords * 4;
        buf += nwords * 4;
    }
    if (write_size > 0) {
        /* Last word, partial. */
        unsigned data = target_read_word (target.device, addr);
        memcpy ((unsigned char*) &data, buf, write_size);
        target_write_word (target.device, addr, data);
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 */
static int mips_resume_from_current(int step, int sig)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_resume_from_current(%s, %d)",
                        mips_target.name,
                        (step)  ?  "step"  :  "run",
                        sig);
    assert (target.device != 0);

    if (step) {
        /* Single step the target */
        target_step (target.device);
    } else {
        /* Run the target to a breakpoint, or until we stop it. */
        target_resume (target.device);
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Run the target from the new PC address.
 */
static int mips_resume_from_addr(int step, int sig, uint64_t addr)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_resume_from_addr(%s, %d, 0x%llX)",
                        mips_target.name,
                        (step)  ?  "step"  :  "run",
                        sig,
                        addr);
    assert (target.device != 0);

    target_run (target.device, addr);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 */
static int mips_go_waiting(int sig)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_go_waiting()",
                        mips_target.name);
    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method.
 * Test the target state (i.e. running/stopped) without blocking.
 */
static int mips_wait_partial(int first, char *status_string,
    size_t status_string_len, out_func of,
    int *implemented, int *more)
{
    char *cp;
    int sig;

    assert (target.device != 0);
    assert (status_string != NULL);
    assert (status_string_len >= 34);
    assert (of != NULL);
    assert (implemented != NULL);
    assert (more != NULL);

    *implemented = TRUE;

    int is_aborted;
    if (! target_is_stopped (target.device, &is_aborted)) {
        *more = TRUE;
//        target.log(RP_VAL_LOGLEVEL_DEBUG,
//                        "%s: mips_wait_partial() cpu is running",
//                        mips_target.name);
//fprintf (stderr, "."); fflush (stderr);
        return RP_VAL_TARGETRET_OK;
    }

    if (is_aborted) {
        sig = RP_SIGNAL_ABORTED;
        target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_wait_partial() cpu is aborted",
                        mips_target.name);
    } else {
        sig = RP_SIGNAL_BREAKPOINT;
        target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_wait_partial() cpu stopped on breakpoint",
                        mips_target.name);
    }

    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    cp = mips_out_treg(&status_string[3], RP_MIPS_REGNUM_PC);
    cp = mips_out_treg(cp, RP_MIPS_REGNUM_FP);
    *more = FALSE;
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 * Wait with blocking - non needed.
 */
static int mips_wait(char *status_string,
                         size_t status_string_len,
                         out_func of,
                         int *implemented)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_wait() - not implemented",
                        mips_target.name);

    *implemented = FALSE;
    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method.
 */
static int mips_process_query(unsigned int *mask,
                                  rp_thread_ref *arg,
                                  rp_thread_info *info)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_process_query()",
                        mips_target.name);

    /* Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method.
 */
static int mips_list_query(int first,
                               rp_thread_ref *arg,
                               rp_thread_ref *result,
                               size_t max_num,
                               size_t *num,
                               int *done)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_list_query()",
                        mips_target.name);

    /* Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method.
 */
static int mips_current_thread_query(rp_thread_ref *thread)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_current_thread_query()",
                        mips_target.name);

    /* Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/*
 * Target method.
 */
static int mips_offsets_query(uint64_t *text, uint64_t *data, uint64_t *bss)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_offsets_query()",
                        mips_target.name);
    assert (target.device != 0);
    assert (text != NULL);
    assert (data != NULL);
    assert (bss != NULL);

    /* Is this what *your* target really needs? */
    *text = 0;
    *data = 0;
    *bss = 0;
    return RP_VAL_TARGETRET_OK;
}

/*
 * Target method.
 */
static int mips_crc_query(uint64_t addr, size_t len, uint32_t *val)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_crc_query()",
                        mips_target.name);
    assert (target.device != 0);

    if (addr > RP_MIPS_MAX_ADDRESS ||
        addr + len > RP_MIPS_MAX_ADDRESS + 1) {
        target.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            mips_target.name,
                            addr);
        return RP_VAL_TARGETRET_ERR;
    }

    target.log(RP_VAL_LOGLEVEL_ERR,
                        "%s: crc not implemented", mips_target.name);
    return RP_VAL_TARGETRET_ERR;
}

/*
 * Target method.
 */
static int mips_raw_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_raw_query(\"%s\")",
                        mips_target.name, in_buf);

    return RP_VAL_TARGETRET_NOSUPP;
}

static int tohex(char *s, const char *t)
{
    int i;
    static char hex[] = "0123456789abcdef";

    i = 0;
    while (*t)
    {
        *s++ = hex[(*t >> 4) & 0x0f];
        *s++ = hex[*t & 0x0f];
        t++;
        i++;
    }
    *s = '\0';
    return i;
}

/*
 * Command: reset to boot vector.
 */
static int mips_rcmd_reset(int argc, char *argv[], out_func of, data_func df)
{
    char buf[1000 + 1];

    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_rcmd_reset()",
                        mips_target.name);
    if (! target.device)
        return RP_VAL_TARGETRET_OK;

    tohex(buf, "Resetting target processor... ");
    of(buf);

    /* Reset the processor. */
    target_restart (target.device);

    tohex(buf, "Done.\n");
    of(buf);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Command: list of registers.
 */
static int mips_rcmd_reg(int argc, char *argv[], out_func of, data_func df)
{
    char buf[1000 + 1];
    char buf2[1000 + 1];

    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_rcmd_reg()",
                        mips_target.name);
    if (! target.device)
        return RP_VAL_TARGETRET_OK;

    sprintf (buf, "               t0= %8x   s0= %8x   t8= %8x        lo= %8x\n",
            target_read_register (target.device, 8),
            target_read_register (target.device, 16),
            target_read_register (target.device, 24),
            target_read_register (target.device, 33));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "at= %8x   t1= %8x   s1= %8x   t9= %8x        hi= %8x\n",
            target_read_register (target.device, 1),
            target_read_register (target.device, 9),
            target_read_register (target.device, 17),
            target_read_register (target.device, 25),
            target_read_register (target.device, 34));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "v0= %8x   t2= %8x   s2= %8x   k0= %8x    status= %8x\n",
            target_read_register (target.device, 2),
            target_read_register (target.device, 10),
            target_read_register (target.device, 18),
            target_read_register (target.device, 26),
            target_read_register (target.device, 32));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "v1= %8x   t3= %8x   s3= %8x   k1= %8x     cause= %8x\n",
            target_read_register (target.device, 3),
            target_read_register (target.device, 11),
            target_read_register (target.device, 19),
            target_read_register (target.device, 27),
            target_read_register (target.device, 36));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "a0= %8x   t4= %8x   s4= %8x   gp= %8x  badvaddr= %8x\n",
            target_read_register (target.device, 4),
            target_read_register (target.device, 12),
            target_read_register (target.device, 20),
            target_read_register (target.device, 28),
            target_read_register (target.device, 35));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "a1= %8x   t5= %8x   s5= %8x   sp= %8x        pc= %8x\n",
            target_read_register (target.device, 5),
            target_read_register (target.device, 13),
            target_read_register (target.device, 21),
            target_read_register (target.device, 29),
            target_read_register (target.device, 37));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "a2= %8x   t6= %8x   s6= %8x   s8= %8x\n",
            target_read_register (target.device, 6),
            target_read_register (target.device, 14),
            target_read_register (target.device, 22),
            target_read_register (target.device, 30));
    tohex(buf2, buf);
    of(buf2);
    sprintf (buf, "a3= %8x   t7= %8x   s7= %8x   ra= %8x\n",
            target_read_register (target.device, 7),
            target_read_register (target.device, 15),
            target_read_register (target.device, 23),
            target_read_register (target.device, 31));
    tohex(buf2, buf);
    of(buf2);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Table of commands.
 */
static const RCMD_TABLE remote_commands[] =
{
    RCMD (help,     "This help text"),
    RCMD (reset,    "Reset the target processor"),
    RCMD (reg,      "List of integer registers and their contents"),
    { 0 }
};

/*
 * Help function, generate help text from command table.
 */
static int mips_rcmd_help(int argc, char *argv[], out_func of, data_func df)
{
    char buf[1000 + 1];
    char buf2[1000 + 1];
    int i = 0;

    tohex(buf, "List of ejtagproxy monitor commands:\n\n");
    of(buf);
    for (i = 0; remote_commands[i].name; i++)
    {
        sprintf(buf2, "%s -- %s\n", remote_commands[i].name, remote_commands[i].help);
        tohex(buf, buf2);
        of(buf);
    }
    return RP_VAL_TARGETRET_OK;
}

/*
 * Decode nibble.
 */
static int remote_decode_nibble(const char *in, unsigned int *nibble)
{
    unsigned int nib;

    if ((nib = rp_hex_nibble(*in)) >= 0)
    {
        *nibble = nib;
        return TRUE;
    }
    return FALSE;
}

/*
 * Decode byte.
 */
static int remote_decode_byte(const char *in, unsigned int *byte)
{
    unsigned int ls_nibble;
    unsigned int ms_nibble;

    if (! remote_decode_nibble(in, &ms_nibble))
        return FALSE;

    if (! remote_decode_nibble(in + 1, &ls_nibble))
        return FALSE;

    *byte = (ms_nibble << 4) + ls_nibble;
    return TRUE;
}

/*
 * Target method.
 */
#define MAXARGS 4

static int mips_remcmd(char *in_buf, out_func of, data_func df)
{
    int count = 0;
    int i;
    char *args[MAXARGS];
    char *ptr;
    unsigned int ch;
    char buf[1000 + 1];
    char *s;

    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_remcmd()",
                        mips_target.name);
    DEBUG_OUT("command '%s'\n", in_buf);

    if (strlen(in_buf))
    {
        /* There is something to process.
         * Handle target specific commands, such as reset,
         * flash erase, JTAG control, etc. */

        /* Turn the hex into ASCII */
        ptr = in_buf;
        s = buf;
        while (*ptr)
        {
            if (remote_decode_byte(ptr, &ch) == 0)
                return RP_VAL_TARGETRET_ERR;
            *s++ = ch;
            ptr += 2;
        }
        *s = '\0';
        DEBUG_OUT("command '%s'\n", buf);

        /* Split string into separate arguments */
        ptr = buf;
        args[count++] = ptr;
        while (*ptr)
        {
            /* Search to the end of the string */
            if (*ptr == ' ')
            {
                /* Space is the delimiter */
                *ptr = 0;
                if (count >= MAXARGS)
                    return RP_VAL_TARGETRET_ERR;
                args[count++] = ptr + 1;
            }
            ptr++;
        }
        /* Search the command table, and execute the function if found */
        DEBUG_OUT("executing target dependant command '%s'\n", args[0]);

        for (i = 0; remote_commands[i].name; i++)
        {
            if (strcmp(args[0], remote_commands[i].name) == 0)
                return remote_commands[i].function(count, args, of, df);
        }
        return RP_VAL_TARGETRET_NOSUPP;
    }
    return mips_rcmd_help(0, 0, of, df);
}

/*
 * Target method.
 */
static int mips_add_break(int type, uint64_t addr, unsigned int len)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
        "%s: mips_add_break(%d, 0x%llx, %d)",
        mips_target.name, type, addr, len);
    assert (target.device != 0);
    switch (type) {
    case 1:             /* hardware-breakpoint */
        target_add_break (target.device, addr, 'b');
        return RP_VAL_TARGETRET_OK;
    case 2:             /* write watchpoint */
        target_add_break (target.device, addr, 'w');
        return RP_VAL_TARGETRET_OK;
    case 3:             /* read watchpoint */
        target_add_break (target.device, addr, 'r');
        return RP_VAL_TARGETRET_OK;
    case 4:             /* access watchpoint */
        target_add_break (target.device, addr, 'a');
        return RP_VAL_TARGETRET_OK;
    default:
        return RP_VAL_TARGETRET_NOSUPP;
    }
}

/*
 * Target method.
 */
static int mips_remove_break(int type, uint64_t addr, unsigned int len)
{
    target.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: mips_remove_break(%d, 0x%llx, %d)",
                        mips_target.name,
                        type,
                        addr,
                        len);
    assert (target.device != 0);
    target_remove_break (target.device, addr);
    return RP_VAL_TARGETRET_OK;
}

/*
 * Output registers in the format suitable
 * for TAAn:r...;n:r...; format.
 */
static char *mips_out_treg(char *in, unsigned int reg_no)
{
    static const char hex[] = "0123456789abcdef";
    uint32_t reg_val;

    if (in == NULL)
        return NULL;

    assert (reg_no < RP_MIPS_NUM_REGS);

    *in++ = hex[(reg_no >> 4) & 0x0f];
    *in++ = hex[reg_no & 0x0f];
    *in++ = ':';

    reg_val = target_read_register (target.device, reg_no);
    //fprintf (stderr, "reg%d = %08x\n", reg_no, reg_val);

    /* The register goes into the buffer in little-endian order */
    *in++ = hex[(reg_val >> 4) & 0x0f];  *in++ = hex[reg_val & 0x0f];
    *in++ = hex[(reg_val >> 12) & 0x0f]; *in++ = hex[(reg_val >> 8) & 0x0f];
    *in++ = hex[(reg_val >> 20) & 0x0f]; *in++ = hex[(reg_val >> 16) & 0x0f];
    *in++ = hex[(reg_val >> 28) & 0x0f]; *in++ = hex[(reg_val >> 24) & 0x0f];
    *in++ = ';';
    *in   = '\0';

    return in;
}
