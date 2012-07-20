/*
 * Remote proxy definitions.
 *
 * Copyright (C) 1999-2001 Quality Quorum, Inc.
 * Copyright (C) 2002 Chris Liechti and Steve Underwood
 * Copyright (C) 2010-2012 Serge Vakulenko
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
 *
 * QQI can be contacted as qqi@world.std.com
 */
#ifndef _PROXY_H_
#define _PROXY_H_

/*=================  Global Parameters ================================ */

/* Size of data buffer  */
#define RP_PARAM_DATABYTES_MAX (0x10000)


/* Size of input and out buffers */
#define RP_PARAM_INOUTBUF_SIZE (2*RP_PARAM_DATABYTES_MAX+32)

/* First socket port to try */
#define RP_PARAM_SOCKPORT_MIN (2000)


/*================= Debugger Parameters ================================*/

/* These values have to be in synch with corresponding values used
   by debugger, fortunately, they are not going to change any
   time soon */

/* Target signals, the only definition we need at this level is one which
   specifies 'no signal' signal. This definition is good one as long
   as gdb's TARGET_SIGNAL_0 is 0 */
#define RP_VAL_TARGETSIG_0 (0)

/* Acceptable packet size for target and for gdb !!!!!
   a lot of gdb_versions will ask for 200 bytes of memory
   and will choke on response. It seems like it only
   affects read memory requests */
#define RP_VAL_DBG_PBUFSIZ  (400)

#if RP_VAL_PBUFSIZ > RP_PARAM_INOUTBUF_SIZE
 #error "Unexpected value of RP_VAL_PBUFSIZ"
#endif /* RP_VAL_PBUFSIZ > RP_PARAM_INOUTBUF_SIZE */

/* Thread related definitions */
typedef struct __rp_thread_ref
{
    uint64_t val;
} rp_thread_ref;

/*
 * Thread value meaning all threads
 */
#define RP_VAL_THREADID_ALL (0ULL)

typedef struct __rp_thread_nfo
{
    rp_thread_ref thread_id;
    int           exists;
    char          display[256];
    char          thread_name[32];
    char          more_display[256];
}
rp_thread_info;

/* Function to do console output from wait methods */
typedef void (*out_func)(const char *string);

/* Function to transefer data received as qRcmd response */
typedef void (*data_func)(const char *string);

/* Function to do logging */
typedef void (*log_func)(int level, const char *string, ...);

/* Log level definitions */
#define RP_VAL_LOGLEVEL_EMERG   (0x1234)
#define RP_VAL_LOGLEVEL_ALERT   (0x4321)
#define RP_VAL_LOGLEVEL_CRIT    (0xabcd)
#define RP_VAL_LOGLEVEL_ERR     (0xdcba)
#define RP_VAL_LOGLEVEL_WARNING (0x5678)
#define RP_VAL_LOGLEVEL_NOTICE  (0x8765)
#define RP_VAL_LOGLEVEL_INFO    (0x1111)
#define RP_VAL_LOGLEVEL_DEBUG   (0x2222)
#define RP_VAL_LOGLEVEL_DEBUG2  (0x3333)
#define RP_VAL_LOGLEVEL_DEBUG3  (0x4444)


/* Target, all functions return boolen values */
typedef struct rp_target_s rp_target;

struct rp_target_s
{
    rp_target *next;

    const char *name; /* Unique ASCII name of the target */

    const char *desc; /* Short description */

    /*======================   Help/Debug  =======================*/

    /* Help, argument is a pointer to itself */
    void (*help)(const char *prog_name);

    /*=========   Open/Close/Connect/Disconnect  ==============*/

    /* Start target stub and provide run time parameters
       in time tested manner, does not assume actually
       connecting to target.  */
    int (*open)(int argc,
                char * const agrv[],
                const char *prog_name,
                log_func log_fn);

    /* Close target stub: if target is still connected disconnect and
       leave it running */
    void (*close)(void);

    /* Actually connect to a target and return status string; */
    int (*connect)(char *status_string,
                   size_t status_string_size,
                   int *can_restart);

    /* Disconnect from a target a leave it running */
    int (*disconnect)(void);


    /*=================== Start/Stop =========================*/

    /* Kill target: disconnect from a target and leave it waiting
       for a command. Target output is ignored.

       Restart: start target all over again.

       Stop: break into running target

       Note these commands are used in following sequences only

       1. kill, close, terminate proxy
       2. kill, restart, connect
       3. restart, connect
       4. stop, wait */


    /* Kill target: disconnect from a target and leave it waiting
       for a command. It is expected that either close or wait or
       connect will follow after kill to get last status_string */
    void (*kill)(void);

    /* Restart target and return status string */
    int (*restart)(void);

    /* Stop target. E.g. send ^C or BREAK to target - note
       it has to be followed either by wait or connect in order to
       to get last status_string */
    void (*stop)(void);

    /*============== Thread Control ===============================*/

    /* Set generic thread */
    int (*set_gen_thread)(rp_thread_ref *thread);

    /* Set control thread */
    int (*set_ctrl_thread)(rp_thread_ref *thread);

    /* Get thread status */
    int (*is_thread_alive)(rp_thread_ref *thread, int *alive);

    /*============= Register Access ================================*/

    /* Read all registers. buf is 4-byte aligned and it is in
       target byte order. If  register is not available
       corresponding bytes in avail_buf are 0, otherwise
       avail buf is 1 */
    int (*read_registers)(unsigned char *data_buf,
                          unsigned char *avail_buf,
                          size_t buf_size,
                          size_t *read_size);

    /* Write all registers. buf is 4-byte aligned and it is in target
       byte order */
    int (*write_registers)(unsigned char *buf, size_t write_size);

    /* Read one register. buf is 4-byte aligned and it is in
       target byte order. If  register is not available
       corresponding bytes in avail_buf are 0, otherwise
       avail buf is 1 */
    int (*read_single_register)(unsigned int reg_no,
                                unsigned char *buf,
                                unsigned char *avail_buf,
                                size_t buf_size,
                                size_t *read_size);

    /* Write one register. buf is 4-byte aligned and it is in target byte
       order */
    int (*write_single_register)(unsigned int reg_no,
                                 unsigned char *buf,
                                 size_t write_size);

    /*=================== Memory Access =====================*/

    /* Read memory, buf is 4-bytes aligned and it is in target
       byte order */
    int (*read_mem)(uint64_t addr,
                    unsigned char *buf,
                    size_t req_size,
                    size_t *actual_size);

    /* Write memory, buf is 4-bytes aligned and it is in target
       byte order */
    int (*write_mem)(uint64_t addr,
                     unsigned char *buf,
                     size_t req_size);

    /*================ Resume/Wait  ============================*/

    /* Resume from current address, if not supported it
       has to be figured out by wait */
    int (*resume_from_current)(int step, int sig);

    /* Resume from specified address, if not supported it
       has to be figured out by wait */
    int (*resume_from_addr)(int step, int sig, uint64_t addr);

    /* Allow threads which are not stopped already to continue */
    int (*go_waiting)(int sig);

    /* Wait function, wait_partial is called by the proxy with one
       tick intervals, so it allows to break into running
       target */

    /* Check for event and return. It allows proxy server to
       check messages from gdb allowing gdb to stop/kill target.
       Break and kill commands are generated by a human being so,
       the process can wait inside wait_partial with some substantial
       timeouts. It seems like 1s time will be highest acceptable value.

       In this case return value RP_TARGETRET_NOSUPP means, that
       response to previous resume was - 'not supported'. If this operation
       is not implemented by target, then it will return OK and
       implemeted will be 0.

       status_string is unchanged unless return value is OK and
       implemented is non 0 */
    int (*wait_partial)(int first,
                        char *status_string,
                        size_t status_string_len,
                        out_func out,
                        int *implemented,
                        int *more);

    /* Wait for event, fill (null-terminated) status_string upon successful
       returm, if there is not enough space for 'TAA... string' use
       'SAA' instead, status_sting_len is always > 3

       In this case return value RP_TARGETRET_NOSUPP means, that
       response to previous resume was - 'not supported'. If this operation
       is not implemented by target, then it will return OK and
       implemeted will be 0

       status_string is unchanged unless return value is OK and
       implemented is non 0 */
    int (*wait)(char *status_string,
                size_t status_string_len,
                out_func out,
                int *implemented);

    /*============= Queries ===============================*/

    /* Bits of mask determine set of information about thread
       to be retrieved, results are put into info.  */
    int (*process_query)(unsigned int *mask,
                         rp_thread_ref *arg,
                         rp_thread_info *info);

    /* List threads. If first is non-zero then start from the first thread,
       otherwise start from arg, result points to array of threads to be
       filled out, result size is number of elements in the result,
       num points to the actual number of threads found, done is
       set if all threads are processed.  */
    int (*list_query)(int first,
                      rp_thread_ref *arg,
                      rp_thread_ref *result,
                      size_t max_num,
                      size_t *num,
                      int *done);

    /* Query current thread id */
    int (*current_thread_query)(rp_thread_ref *thread);

    /* Query offset of major sections in memory */
    int (*offsets_query)(uint64_t *text, uint64_t *data, uint64_t *bss);

    /* Query crc32 of memory area */
    int (*crc_query)(uint64_t addr, size_t len, uint32_t *val);

    /* Raw query, see gdb-XXXX/gdb/remote.c. we got buffer
       call this function. It is a responsibility of the target
       to fill out out_buf correctly in case of success.

       It is planned to have more more specific queries in
       the nearest future.  */
    int (*raw_query)(char *in_buf, char *out_buf, size_t out_buf_size);

    /* Remote command */
    int (*remcmd)(char *in_buf, out_func of, data_func df);

    /*============ Breakpoints ===========================*/

    int (*add_break)(int type, uint64_t addr, unsigned int length);
    int (*remove_break)(int type, uint64_t addr, unsigned int length);
};


/* Return values of target functions */
#define RP_VAL_TARGETRET_OK     (0x345) /* Success */
#define RP_VAL_TARGETRET_ERR    (0x678) /* Error */
#define RP_VAL_TARGETRET_NOSUPP (0x9ab) /* Operation is not supported */

/* Bits of process_query mask */
#define RP_BIT_PROCQMASK_THREADID    (1)
#define RP_BIT_PROCQMASK_EXISTS      (2)
#define RP_BIT_PROCQMASK_DISPLAY     (4)
#define RP_BIT_PROCQMASK_THREADNAME  (8)
#define RP_BIT_PROCQMASK_MOREDISPLAY (16)

/* Break types. Note: a software break point does nothing else than
   notify target about planned action to patch memory with a breakpoint
   instruction. It is not associated with any actions to be taken */
#define RP_VAL_BREAKTYPE_MIN   (0)
#define RP_VAL_BREAKTYPE_MAX   (4)

/* Signal types */

#define RP_SIGNAL_HANGUP                    1
#define RP_SIGNAL_INTERRUPT                 2
#define RP_SIGNAL_QUIT                      3
#define RP_SIGNAL_ILLEGAL_INSTRUCTION       4
#define RP_SIGNAL_BREAKPOINT                5
#define RP_SIGNAL_ABORTED                   6
#define RP_SIGNAL_BUS_ERROR                 7
#define RP_SIGNAL_FLOATING_POINT_EXCEPTION  8
#define RP_SIGNAL_KILLED                    9
#define RP_SIGNAL_SEGMENT_VIOLATION         11
#define RP_SIGNAL_TERMINATED                15
#define RP_SIGNAL_STK_FLT                   16

/* Breakpoint types */

#define RP_BP_TYPE_SOFTWARE             0
#define RP_BP_TYPE_HARDWARE             1
#define RP_BP_TYPE_READ_WATCH           2
#define RP_BP_TYPE_WRITE_WATCH          3
#define RP_BP_TYPE_ACCESS_WATCH         4

#if !defined(FALSE)
#define FALSE 0
#endif
#if !defined(TRUE)
#define TRUE (!FALSE)
#endif

#define ACK                             '+'
#define NAK                             '-'

#define PACKET_BUFF_SIZE                8192

extern int debug_level;
extern rp_target mips_target;

/* Initialization function in init.c */
rp_target *rp_init(void);

/* Functions to display warranty and copying information */
void rp_show_copying(void);
void rp_show_warranty(void);

int rp_hex_nibble(char in);

void dbg_sock_init(void);
void dbg_sock_cleanup(void);
int  dbg_sock_open(unsigned int *port);
void dbg_sock_close(void);
int  dbg_sock_accept(void);
int  dbg_sock_readchar(int ms);

/* Return values for readchar: either character
   code or one of the following*/
#define RP_VAL_MISCREADCHARRET_TMOUT (-2)
#define RP_VAL_MISCREADCHARRET_ERR   (-1)

void dbg_sock_putchar(int c);
int  dbg_sock_write(unsigned char *buf, size_t len);

log_func rp_env_init(char *name, int do_daemon);

#endif /* _PROXY_H_ */
