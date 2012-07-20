/* Copyright (C) 2002 Chris Liechti and Steve Underwood

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. The name of the author may not be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
   EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   Implementation of a dummy `skeleton' target for the GDB proxy server.

   Exported Data:
     skeletone_target            - target descriptor of the `skeleton' target

   Imported Data:
     None

   Static Data:
     skeleton_XXXX               - static data representing status and
                                   parameters of the target

   Global Functions:
     None

   Static Functions:
     skeleton_XXXX              - methods comprising the `skeleton' target.
                                  A description is in file gdbproxy.h

     skeleton_                  - local finctions
     skeleton_command

   $Id: target_skeleton.c,v 1.5 2005/08/21 06:51:56 coppice Exp $ */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <getopt.h>

#include "gdbproxy.h"


/* Note: we are using prefix 'skeleton' for static stuff in
   order to simplify debugging of the target code itself */

/* TODO: Put the correct values for the real target in these macros */
#define RP_SKELETON_MIN_ADDRESS             0x0U
#define RP_SKELETON_MAX_ADDRESS             0xFFFFU
#define RP_SKELETON_REG_DATATYPE            uint16_t
#define RP_SKELETON_REG_BYTES               (16*sizeof(uint16_t))
#define RP_SKELETON_NUM_REGS                16
#define RP_SKELETON_REGNUM_PC               0  /* Program counter reg. number */
#define RP_SKELETON_REGNUM_SP               1  /* Stack pointer reg. number */
#define RP_SKELETON_REGNUM_FP               4  /* Frame pointer reg. number */

#define RP_SKELETON_MAX_BREAKPOINTS         10

/* Some example states a real target might support. */
#define RP_SKELETON_TARGET_STATE_RUNNING                    0
#define RP_SKELETON_TARGET_STATE_STOPPED                    1
#define RP_SKELETON_TARGET_STATE_SINGLE_STEP_COMPLETE       2
#define RP_SKELETON_TARGET_STATE_BREAKPOINT_HIT             3

/*
 * Target methods, static
 */
static void  skeleton_help(const char *prog_name);
static int   skeleton_open(int argc,
                           char * const argv[],
                           const char *prog_name,
                           log_func log_fn);
static void  skeleton_close(void);
static int   skeleton_connect(char *status_string,
                              size_t status_string_size,
                              int *can_restart);
static int   skeleton_disconnect(void);
static void  skeleton_kill(void);
static int   skeleton_restart(void);
static void  skeleton_stop(void);
static int   skeleton_set_gen_thread(rp_thread_ref *thread);
static int   skeleton_set_ctrl_thread(rp_thread_ref *thread);
static int   skeleton_is_thread_alive(rp_thread_ref *thread, int *alive);
static int   skeleton_read_registers(uint8_t *data_buf,
                                     uint8_t *avail_buf,
                                     size_t buf_size,
                                     size_t *read_size);
static int   skeleton_write_registers(uint8_t *data_buf, size_t write_size);
static int   skeleton_read_single_register(unsigned int reg_no,
                                           uint8_t *data_buf,
                                           uint8_t *avail_buf,
                                           size_t buf_size,
                                           size_t *read_size);
static int   skeleton_write_single_register(unsigned int reg_no,
                                            uint8_t *data_buf,
                                            size_t write_size);
static int   skeleton_read_mem(uint64_t addr,
                               uint8_t *data_buf,
                               size_t req_size,
                               size_t *actual_size);
static int   skeleton_write_mem(uint64_t addr,
                                uint8_t *data_buf,
                                size_t req_sise);
static int   skeleton_resume_from_current(int step, int sig);
static int   skeleton_resume_from_addr(int step,
                                       int sig,
                                       uint64_t addr);
static int   skeleton_go_waiting(int sig);
static int   skeleton_wait_partial(int first,
                                   char *status_string,
                                   size_t status_string_len,
                                   out_func out,
                                   int *implemented,
                                   int *more);
static int   skeleton_wait(char *status_string,
                           size_t status_string_len,
                           out_func out,
                           int *implemented);
static int   skeleton_process_query(unsigned int *mask,
                                    rp_thread_ref *arg,
                                    rp_thread_info *info);
static int   skeleton_list_query(int first,
                                 rp_thread_ref *arg,
                                 rp_thread_ref *result,
                                 size_t max_num,
                                 size_t *num,
                                 int *done);
static int   skeleton_current_thread_query(rp_thread_ref *thread);
static int   skeleton_offsets_query(uint64_t *text,
                                    uint64_t *data,
                                    uint64_t *bss);
static int   skeleton_crc_query(uint64_t addr,
                                size_t len,
                                uint32_t *val);
static int   skeleton_raw_query(char *in_buf,
                                char *out_buf,
                                size_t out_buf_size);
static int   skeleton_remcmd(char *in_buf, out_func of, data_func df);
static int   skeleton_add_break(int type, uint64_t addr, unsigned int len);
static int   skeleton_remove_break(int type, uint64_t addr, unsigned int len);

//Table of remote commands following
static int skeleton_rcmd_help(int argc, char *argv[], out_func of, data_func df);  //proto for table entry

static uint32_t crc32(uint8_t *buf, size_t len, uint32_t crc);

#define RCMD(name, hlp) {#name, skeleton_rcmd_##name, hlp}  //table entry generation

//Table entry definition
typedef struct
{
    const char *name;                                   // command name
    int (*function)(int, char**, out_func, data_func);  // function to call
    const char *help;                                   // one line of help text
} RCMD_TABLE;

/*
 * Global target descriptor
 */
rp_target skeleton_target =
{
    NULL,      /* next */
    "skeleton",
    "skeleton target to demonstrate the GDB proxy server",
    skeleton_help,
    skeleton_open,
    skeleton_close,
    skeleton_connect,
    skeleton_disconnect,
    skeleton_kill,
    skeleton_restart,
    skeleton_stop,
    skeleton_set_gen_thread,
    skeleton_set_ctrl_thread,
    skeleton_is_thread_alive,
    skeleton_read_registers,
    skeleton_write_registers,
    skeleton_read_single_register,
    skeleton_write_single_register,
    skeleton_read_mem,
    skeleton_write_mem,
    skeleton_resume_from_current,
    skeleton_resume_from_addr,
    skeleton_go_waiting,
    skeleton_wait_partial,
    skeleton_wait,
    skeleton_process_query,
    skeleton_list_query,
    skeleton_current_thread_query,
    skeleton_offsets_query,
    skeleton_crc_query,
    skeleton_raw_query,
    skeleton_remcmd,
    skeleton_add_break,
    skeleton_remove_break
};

struct skeleton_status_s
{
    /* Start up parameters, set by skeleton_open */
    log_func    log;
    int         is_open;

    /* Tell wait_xxx method the notion of whether or not
       previously called resume is supported */
    int         target_running;
    int         target_interrupted;
    RP_SKELETON_REG_DATATYPE    registers[RP_SKELETON_NUM_REGS];
    uint64_t                    breakpoints[RP_SKELETON_MAX_BREAKPOINTS];
};

static struct skeleton_status_s skeleton_status =
{
    NULL,
    FALSE,
    FALSE,
    FALSE,
    {0},
    {0}
};

/* Local functions */
static char *skeleton_out_treg(char *in, unsigned int reg_no);
static int refresh_registers(void);

/* Target method */

#ifdef NDEBUG
#define DEBUG_OUT(...)
#else
static void DEBUG_OUT(const char *string,...)
{
    va_list args;
    va_start (args, string);
    fprintf (stderr, "debug: ");
    vfprintf (stderr, string, args);
    fprintf (stderr, "\n");
    va_end (args);
}
#endif

static void skeleton_help(const char *prog_name)
{
    printf("This is the skeleton target for the GDB proxy server. Usage:\n\n");
    printf("  %s [options] %s [skeleton-options] [port]\n",
           prog_name,
           skeleton_target.name);
    printf("\nOptions:\n\n");
    printf("  --debug              run %s in debug mode\n", prog_name);
    printf("  --help               `%s --help %s'  prints this message\n",
           prog_name,
           skeleton_target.name);
    printf("  --port=PORT          use the specified TCP port\n");
    printf("\nskeleton-options:\n\n");

    printf("\n");

    return;
}

/* Target method */
static int skeleton_open(int argc,
                         char * const argv[],
                         const char *prog_name,
                         log_func log_fn)
{
    /* TODO: This example assumes a target attached to a parallel port, as many
             JTAG controlled devices are.*/
#if WIN32
    const char *port = "1";
#elif defined(__linux__)
    const char *port = "/dev/parport0";
#else
    const char *port ="/dev/ppi0";
#endif
    /* Option descriptors */
    static struct option long_options[] =
    {
        /* Options setting flag */
        {NULL, 0, 0, 0}
    };

    assert(!skeleton_status.is_open);
    assert(prog_name != NULL);
    assert(log_fn != NULL);

    /* Set log */
    skeleton_status.log = log_fn;

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_open()",
                        skeleton_target.name);

    /* Process options */
    for (;;)
    {
        int c;
        int option_index;

        c = getopt_long(argc, argv, "+", long_options, &option_index);
        if (c == EOF)
            break;
        switch (c)
        {
        case 0:
            /* Long option which just sets a flag */
            break;
        default:
            skeleton_status.log(RP_VAL_LOGLEVEL_NOTICE,
                                "%s: Use `%s --help %s' to see a complete list of options",
                                skeleton_target.name,
                                prog_name,
                                skeleton_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    if (optind == (argc - 1))
    {
        port = argv[optind];
    }
    else if (optind != argc)
    {
        /* Bad number of arguments */
        skeleton_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad number of arguments",
                            skeleton_target.name);
        skeleton_target.help(prog_name);

        return RP_VAL_TARGETRET_ERR;
    }

    if (!skeleton_status.is_open)
    {
        /* TODO: initialise the target interface */
    }

    /* TODO: Perform any initial target configuration. Perhaps check if the
       target is actually there. */

    /* Set up initial default values */
    skeleton_status.target_running = FALSE;
    skeleton_status.target_interrupted = FALSE;
    memset (skeleton_status.registers, 0, sizeof(skeleton_status.registers));
    memset (skeleton_status.breakpoints, 0, sizeof(skeleton_status.breakpoints));

    skeleton_status.is_open = TRUE;

    return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void skeleton_close(void)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_close()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    /* TODO: Tidy up things and shut down. */

    skeleton_status.is_open = FALSE;
}

/* Target method */
static int skeleton_connect(char *status_string,
                            size_t status_string_len,
                            int *can_restart)
{
    char *cp;

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_connect()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(can_restart != NULL);

    *can_restart = TRUE;

    /* Fill out the the status string */
    sprintf(status_string, "T%02d", RP_SIGNAL_ABORTED);

    if (refresh_registers())
        return RP_VAL_TARGETRET_ERR;

    cp = skeleton_out_treg(&status_string[3], RP_SKELETON_REGNUM_PC);
    cp = skeleton_out_treg(cp, RP_SKELETON_REGNUM_FP);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int skeleton_disconnect(void)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_disconnect()",
                        skeleton_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static void skeleton_kill(void)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_kill()",
                        skeleton_target.name);

    /* TODO: Kill the target debug session. */
}

static int skeleton_restart(void)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_restart()",
                        skeleton_target.name);

    /* Just stop it. The actual restart will be done
       when connect is called again */
    skeleton_stop();

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void skeleton_stop(void)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_stop()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    /* TODO: Steop (i,e, break) the target program. */

    skeleton_status.target_interrupted = TRUE;
}

static int skeleton_set_gen_thread(rp_thread_ref *thread)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_set_gen_thread()",
                        skeleton_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_set_ctrl_thread(rp_thread_ref *thread)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_set_ctrl_thread()",
                        skeleton_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_is_thread_alive(rp_thread_ref *thread, int *alive)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_is_thread_alive()",
                        skeleton_target.name);

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_read_registers(uint8_t *data_buf,
                                 uint8_t *avail_buf,
                                 size_t buf_size,
                                 size_t *read_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_read_registers()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= RP_SKELETON_REG_BYTES);
    assert(read_size != NULL);

    if (refresh_registers())
        return RP_VAL_TARGETRET_ERR;

    memcpy(data_buf, skeleton_status.registers, RP_SKELETON_REG_BYTES);
    memset(avail_buf, 1, RP_SKELETON_REG_BYTES);
    *read_size = RP_SKELETON_REG_BYTES;
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_write_registers(uint8_t *buf, size_t write_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_write_registers()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(buf != NULL);
    assert(write_size > 0);
    assert(write_size <= RP_SKELETON_REG_BYTES);

    memcpy(skeleton_status.registers, buf, write_size);

    /* TODO: Write the registers to the target. */

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_read_single_register(unsigned int reg_no,
                                         uint8_t *data_buf,
                                         uint8_t *avail_buf,
                                         size_t buf_size,
                                         size_t *read_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_read_single_register()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= RP_SKELETON_REG_BYTES);
    assert(read_size != NULL);

    if (reg_no < 0  ||  reg_no > RP_SKELETON_NUM_REGS)
        return RP_VAL_TARGETRET_ERR;

    if (refresh_registers())
        return RP_VAL_TARGETRET_ERR;

    memcpy(data_buf, &skeleton_status.registers[reg_no], sizeof(skeleton_status.registers[reg_no]));
    memset(avail_buf, 1, sizeof(skeleton_status.registers[reg_no]));
    *read_size = sizeof(skeleton_status.registers[reg_no]);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_write_single_register(unsigned int reg_no,
                                          uint8_t *buf,
                                          size_t write_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_write_single_register(%d, 0x%X)",
                        skeleton_target.name,
                        reg_no,
                        ((RP_SKELETON_REG_DATATYPE *) buf)[0]);

    assert(skeleton_status.is_open);

    assert(buf != NULL);
    assert(write_size == 2);

    if (reg_no < 0  ||  reg_no > RP_SKELETON_NUM_REGS)
        return RP_VAL_TARGETRET_ERR;

    skeleton_status.registers[reg_no] = ((RP_SKELETON_REG_DATATYPE *) buf)[0];

    /* TODO: Write the register to the target. */

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_read_mem(uint64_t addr,
                             uint8_t *buf,
                             size_t req_size,
                             size_t *actual_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_read_mem(0x%llX, ptr, %d, ptr)",
                        skeleton_target.name,
                        addr,
                        req_size);

    assert(skeleton_status.is_open);

    assert(buf != NULL);
    assert(req_size > 0);
    assert(actual_size != NULL);

    if (addr > RP_SKELETON_MAX_ADDRESS)
    {
        skeleton_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            skeleton_target.name,
                            addr);

        return RP_VAL_TARGETRET_ERR;
    }

    if (addr + req_size > RP_SKELETON_MAX_ADDRESS + 1)
        *actual_size = RP_SKELETON_MAX_ADDRESS + 1 - addr;
    else
        *actual_size = req_size;

    /* TODO: Read the required block of memory from the target. */

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_write_mem(uint64_t addr,
                              uint8_t *buf,
                              size_t write_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_write_mem(0x%llX, ptr, %d)",
                        skeleton_target.name,
                        addr,
                        write_size);

    assert(skeleton_status.is_open);
    assert(buf != NULL);

    /* GDB does zero length writes for some reason. Treat them harmlessly. */
    if (write_size == 0)
        return RP_VAL_TARGETRET_OK;

    if (addr > RP_SKELETON_MAX_ADDRESS)
    {
        skeleton_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            skeleton_target.name,
                            addr);
        return RP_VAL_TARGETRET_ERR;
    }

    if ((addr + write_size - 1) > RP_SKELETON_MAX_ADDRESS)
    {
        skeleton_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address/write_size 0x%llx/0x%x",
                            skeleton_target.name,
                            addr,
                            write_size);
        return RP_VAL_TARGETRET_ERR;
    }

    /* TODO: Write to the target. */

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_resume_from_current(int step, int sig)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_resume_from_current(%s, %d)",
                        skeleton_target.name,
                        (step)  ?  "step"  :  "run",
                        sig);

    assert(skeleton_status.is_open);

    if (step)
        /* TODO: Single step the target */;
    else
        /* TODO: Run the target to a breakpoint, or until we stop it. */;

    skeleton_status.target_running = TRUE;
    skeleton_status.target_interrupted = FALSE;
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_resume_from_addr(int step, int sig, uint64_t addr)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_resume_from_addr(%s, %d, 0x%llX)",
                        skeleton_target.name,
                        (step)  ?  "step"  :  "run",
                        sig,
                        addr);

    assert(skeleton_status.is_open);

    skeleton_status.registers[RP_SKELETON_REGNUM_PC] = addr;

    /* TODO: Update the PC register in the target */

    /* TODO: Run the target from the new PC address. */

    skeleton_status.target_running = TRUE;
    skeleton_status.target_interrupted = FALSE;
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_go_waiting(int sig)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_go_waiting()",
                        skeleton_target.name);
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_wait_partial(int first,
                                 char *status_string,
                                 size_t status_string_len,
                                 out_func of,
                                 int *implemented,
                                 int *more)
{
    int state;
    char *cp;
    int sig;

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_wait_partial()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);
    assert(more != NULL);

    *implemented = TRUE;

    if (!skeleton_status.target_running)
        return RP_VAL_TARGETRET_NOSUPP;

#ifdef WIN32
    sleep((first)  ?  500  :  100);
#else
    usleep((first)  ?  500000  :  100000);
#endif
    /* TODO: Test the target state (i.e. running/stopped) without blocking */
    /* If the target only supports a blocking form of test return no support,
       and the blocking version of this test will be called instead. That is
       not so nice, as the system is less interactive using a blocking test. */
    state = RP_SKELETON_TARGET_STATE_RUNNING;

    if (state == RP_SKELETON_TARGET_STATE_RUNNING)
    {
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    }

    switch (state)
    {
    case RP_SKELETON_TARGET_STATE_STOPPED:
        if (skeleton_status.target_interrupted)
            sig = RP_SIGNAL_INTERRUPT;
        else
            sig = RP_SIGNAL_ABORTED;
        break;
    case RP_SKELETON_TARGET_STATE_RUNNING:
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    case RP_SKELETON_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_BREAKPOINT;
        break;
    case RP_SKELETON_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_BREAKPOINT;
        break;
    default:
        skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the SKELETON",
                            skeleton_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    if (refresh_registers())
        return RP_VAL_TARGETRET_ERR;

    cp = skeleton_out_treg(&status_string[3], RP_SKELETON_REGNUM_PC);
    cp = skeleton_out_treg(cp, RP_SKELETON_REGNUM_FP);

    *more = FALSE;

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int skeleton_wait(char *status_string,
                         size_t status_string_len,
                         out_func of,
                         int *implemented)
{
    int state;
    char *cp;
    int sig;

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_wait()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);

    *implemented = TRUE;

    if (!skeleton_status.target_running)
        return RP_VAL_TARGETRET_NOSUPP;

    /* TODO: Wait for the target to stop */
    state = RP_SKELETON_TARGET_STATE_STOPPED;

    switch (state)
    {
    case RP_SKELETON_TARGET_STATE_STOPPED:
        sig = RP_SIGNAL_ABORTED;
        break;
    case RP_SKELETON_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_BREAKPOINT;
        break;
    case RP_SKELETON_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_BREAKPOINT;
        break;
    default:
        skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the SKELETON",
                            skeleton_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    if (refresh_registers())
        return RP_VAL_TARGETRET_ERR;

    cp = skeleton_out_treg(&status_string[3], RP_SKELETON_REGNUM_PC);
    cp = skeleton_out_treg(cp, RP_SKELETON_REGNUM_FP);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int skeleton_process_query(unsigned int *mask,
                                  rp_thread_ref *arg,
                                  rp_thread_info *info)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_process_query()",
                        skeleton_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_list_query(int first,
                               rp_thread_ref *arg,
                               rp_thread_ref *result,
                               size_t max_num,
                               size_t *num,
                               int *done)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_list_query()",
                        skeleton_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_current_thread_query(rp_thread_ref *thread)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_current_thread_query()",
                        skeleton_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_offsets_query(uint64_t *text, uint64_t *data, uint64_t *bss)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_offsets_query()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    assert(text != NULL);
    assert(data != NULL);
    assert(bss != NULL);

    /* TODO: Is this what *your* target really needs? */
    *text = 0;
    *data = 0;
    *bss = 0;
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_crc_query(uint64_t addr, size_t len, uint32_t *val)
{
    uint8_t buf[1] = {0};

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_crc_query()",
                        skeleton_target.name);

    assert(skeleton_status.is_open);

    if (addr > RP_SKELETON_MAX_ADDRESS  ||  addr + len > RP_SKELETON_MAX_ADDRESS + 1)
    {
        skeleton_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            skeleton_target.name,
                            addr);

        return RP_VAL_TARGETRET_ERR;
    }

    /* TODO: Read the target memory,and use the crc32 routine to calculate
       the CRC value to be returned. */
    /* Note: The CRC can be calculated in chunks. The first call to crc32
       should set the current CRC value to all 1's, as this is the priming
       value for CRC32. Subsequent calls should set the current CRC to the
       value returned by the previous call, until all the data has been
       processed. */

    *val = crc32(buf, sizeof(buf), 0xFFFFFFFF);

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int skeleton_raw_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_raw_query()",
                        skeleton_target.name);

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

/* command: erase flash */
static int skeleton_rcmd_erase(int argc, char *argv[], out_func of, data_func df)
{
    char buf[1000 + 1];

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_rcmd_erase()",
                        skeleton_target.name);
    tohex(buf, "Erasing target flash - ");
    of(buf);

    /* TODO: perform the erase. */

    tohex(buf, " Erased OK\n");
    of(buf);
    return RP_VAL_TARGETRET_OK;
}

/* Table of commands */
static const RCMD_TABLE remote_commands[] =
{
    RCMD(help,      "This help text"),

    RCMD(erase,     "Erase target flash memory"),
    {0,0,0}     //sentinel, end of table marker
};

/* Help function, generate help text from command table */
static int skeleton_rcmd_help(int argc, char *argv[], out_func of, data_func df)
{
    char buf[1000 + 1];
    char buf2[1000 + 1];
    int i = 0;

    tohex(buf, "Remote command help:\n");
    of(buf);
    for (i = 0;  remote_commands[i].name;  i++)
    {
#ifdef WIN32
        sprintf(buf2, "%-10s %s\n", remote_commands[i].name, remote_commands[i].help);
#else
        snprintf(buf2, 1000, "%-10s %s\n", remote_commands[i].name, remote_commands[i].help);
#endif
        tohex(buf, buf2);
        of(buf);
    }
    return RP_VAL_TARGETRET_OK;
}

/* Decode nibble */
static int remote_decode_nibble(const char *in, unsigned int *nibble)
{
    unsigned int nib;

    if ((nib = rp_hex_nibble(*in)) >= 0)
    {
        *nibble = nib;
        return  TRUE;
    }

    return  FALSE;
}


/* Decode byte */
static int remote_decode_byte(const char *in, unsigned int *byte)
{
    unsigned int ls_nibble;
    unsigned int ms_nibble;

    if (!remote_decode_nibble(in, &ms_nibble))
        return  FALSE;

    if (!remote_decode_nibble(in + 1, &ls_nibble))
        return  FALSE;

    *byte = (ms_nibble << 4) + ls_nibble;

    return  TRUE;
}


/* Target method */
#define MAXARGS 4
static int skeleton_remcmd(char *in_buf, out_func of, data_func df)
{
    int count = 0;
    int i;
    char *args[MAXARGS];
    char *ptr;
    unsigned int ch;
    char buf[1000 + 1];
    char *s;

    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_remcmd()",
                        skeleton_target.name);
    DEBUG_OUT("command '%s'", in_buf);

    if (strlen(in_buf))
    {
        /* There is something to process */
        /* TODO: Handle target specific commands, such as flash erase, JTAG
                 control, etc. */
        /* A single example "flash erase" command is partially implemented
           here as an example. */

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
        DEBUG_OUT("command '%s'", buf);

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
        DEBUG_OUT("executing target dependant command '%s'", args[0]);

        for (i = 0;  remote_commands[i].name;  i++)
        {
            if (strcmp(args[0], remote_commands[i].name) == 0)
                return remote_commands[i].function(count, args, of, df);
        }
        return RP_VAL_TARGETRET_NOSUPP;
    }
    return RP_VAL_TARGETRET_ERR;
}


/* Target method */
static int skeleton_add_break(int type, uint64_t addr, unsigned int len)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_add_break(%d, 0x%llx, %d)",
                        skeleton_target.name,
                        type,
                        addr,
                        len);
    /* TODO: Handle whichever types of breakpoint the target can support, and
       report no support for the others. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int skeleton_remove_break(int type, uint64_t addr, unsigned int len)
{
    skeleton_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: skeleton_remove_break(%d, 0x%llx, %d)",
                        skeleton_target.name,
                        type,
                        addr,
                        len);
    /* TODO: Handle whichever types of breakpoint the target can support, and
       report no support for the others. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Output registers in the format suitable
   for TAAn:r...;n:r...;  format */
static char *skeleton_out_treg(char *in, unsigned int reg_no)
{
    static const char hex[] = "0123456789abcdef";
    int16_t reg_val;

    if (in == NULL)
        return NULL;

    assert(reg_no < RP_SKELETON_NUM_REGS);

    *in++ = hex[(reg_no >> 4) & 0x0f];
    *in++ = hex[reg_no & 0x0f];
    *in++ = ':';

    reg_val = skeleton_status.registers[reg_no];
    /* The register goes into the buffer in little-endian order */
    *in++ = hex[(reg_val >> 4) & 0x0f];
    *in++ = hex[reg_val & 0x0f];
    *in++ = hex[(reg_val >> 12) & 0x0f];
    *in++ = hex[(reg_val >> 8) & 0x0f];
    *in++ = ';';
    *in   = '\0';

    return in;
}

/* Table used by the crc32 function to calcuate the checksum. */
static uint32_t crc32_table[256] =
{
    0,
    0
};

static uint32_t crc32(uint8_t *buf, size_t len, uint32_t crc)
{
    if (!crc32_table[1])
    {
        /* Initialize the CRC table and the decoding table. */
        int i;
        int j;
        unsigned int c;

        for (i = 0; i < 256; i++)
	{
	    for (c = i << 24, j = 8; j > 0; --j)
	        c = c & 0x80000000 ? (c << 1) ^ 0x04c11db7 : (c << 1);
	    crc32_table[i] = c;
	}
    }

    while (len--)
    {
        crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ *buf) & 255];
        buf++;
    }
    return crc;
}

static int refresh_registers(void)
{
    int i;

    /* TODO: Grab the real register values from the target */
    for (i = 0;  i < RP_SKELETON_NUM_REGS;  i++)
        skeleton_status.registers[i] = 42;
    return  FALSE;
}
