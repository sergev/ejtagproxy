/*
 * Socket functions and log functions for gdb proxy.
 *
 * Copyright (C) 1999-2001 Quality Quorum, Inc.
 * Copyright (C) 2002 Chris Liechti and Steve Underwood.
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
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#if defined(WIN32)
#   include <windows.h>
#   include <winsock.h>
#   include <sys/types.h>
#   include <sys/timeb.h>
#else
#   include <string.h>
#   include <stdlib.h>
#   include <fcntl.h>
#   include <stdarg.h>
#   include <errno.h>
#   include <netdb.h>
#   include <signal.h>
#   include <unistd.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <syslog.h>
#   include <sys/time.h>
#   include <sys/socket.h>
#   include <netinet/in.h>
#   include <netinet/tcp.h>
#endif

#include "gdbproxy.h"

/*
 * Static Data:
 *   dbg_sock        - socket to be used for communication
 *   dbg_listen_sock - socket to be used for communication
 */
#if defined(WIN32)
static int dbg_sock        = INVALID_SOCKET;
static int dbg_listen_sock = INVALID_SOCKET;

/* winsock seems to omit the follow standard definition
 * (actually a lot of older Unix header do, too). */
typedef int socklen_t;
#else
static int dbg_sock        = -1;
static int dbg_listen_sock = -1;
#endif

/*
 * Log functions
 */
static void rp_log_local(int level, const char *fmt, ...);

#if !defined(WIN32)
static void rp_log_system(int level, const char *fmt, ...);
#endif

/*
 * Global Functions:
 *   dbg_sock_xxxx   - communication with debugger
 */

/* Initialize debugger connection */
void dbg_sock_init(void)
{
#if defined(WIN32)
    WSADATA data;
    int ret;

    assert(dbg_sock == INVALID_SOCKET);

    if ((ret = WSAStartup(MAKEWORD(2, 1), &data)) != 0)
    {
        WSACleanup();
        assert(0);
    }
#else
    assert(dbg_sock == -1);
#endif
}

void dbg_sock_cleanup(void)
{
    dbg_sock_close();
#if defined(WIN32)
    WSACleanup();
#endif
}

/* Open socket in a mode expected by gdb */
int dbg_sock_open(unsigned int *port)
{
    struct sockaddr_in sa;
    int tmp;
    int ret;
    int p;

    assert(port == NULL  ||  *port < 0x10000);

#if defined(WIN32)
    assert(dbg_sock == INVALID_SOCKET  &&  dbg_listen_sock == INVALID_SOCKET);

    if ((dbg_listen_sock = socket(PF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
        return  FALSE;
#else
    assert(dbg_sock == -1  &&  dbg_listen_sock == -1);

    if ((dbg_listen_sock = socket(PF_INET, SOCK_STREAM, 0)) < 0)
        return  FALSE;
#endif

    tmp = 1;
    setsockopt(dbg_listen_sock,
    	       SOL_SOCKET,
               SO_REUSEADDR,
               (char *) &tmp,
               sizeof(tmp));

    sa.sin_family = PF_INET;
    sa.sin_addr.s_addr = INADDR_ANY;

    if (port == NULL  ||  *port != 0)
    {
        if (port == NULL)
            sa.sin_port = 0;
        else
            sa.sin_port = htons((unsigned short int) *port);

#if defined(WIN32)
        if ((ret = bind(dbg_listen_sock, (struct sockaddr *) &sa, sizeof (sa))) == SOCKET_ERROR)
            return  FALSE;
#else
        if ((ret = bind(dbg_listen_sock, (struct sockaddr *) &sa, sizeof (sa))) != 0)
            return  FALSE;
#endif
    }
    else
    {
        /* We have to try for an available port */
        for (p = RP_PARAM_SOCKPORT_MIN;  p < 0x10000;  p++)
        {
            sa.sin_port = htons(p);

#if defined(WIN32)
            if ((ret = bind(dbg_listen_sock, (struct sockaddr *) &sa, sizeof (sa))) != SOCKET_ERROR)
                break;
#else
            if ((ret = bind(dbg_listen_sock, (struct sockaddr *) &sa, sizeof (sa))) == 0)
                break;
#endif
        }

        if (p == 0x10000)
        {
            /* No sockets available */
            return  FALSE;
        }

        *port = p;
    }

    return  TRUE;
}

/* Accept incoming connection and set mode expected by gdb */
int dbg_sock_accept(void)
{
    int ret;
    socklen_t tmp;
    struct sockaddr_in sa;
    struct protoent *pe;

#if defined(WIN32)
    assert(dbg_sock == INVALID_SOCKET);
    assert(dbg_listen_sock != INVALID_SOCKET);

    if ((ret = listen(dbg_listen_sock, 1)) == SOCKET_ERROR)
        return  FALSE;

    tmp = sizeof(sa);
    if ((dbg_sock = accept(dbg_listen_sock, (struct sockaddr *) &sa, &tmp))
        == INVALID_SOCKET)
    {
        return  FALSE;
    }

    if ((pe = getprotobyname("tcp")) == NULL)
        return  FALSE;
#else
    assert(dbg_sock < 0);
    assert(dbg_listen_sock >= 0);

    if ((ret = listen(dbg_listen_sock, 1)) != 0)
        return  FALSE;

    tmp = sizeof(sa);
    if ((dbg_sock = accept(dbg_listen_sock, (struct sockaddr *) &sa, &tmp)) < 0)
        return  FALSE;

    if ((pe = getprotobyname("tcp")) == NULL)
        return  FALSE;
#endif

    tmp = 1;
    setsockopt(dbg_listen_sock,
    	       SOL_SOCKET,
	       SO_KEEPALIVE,
               (char *) &tmp,
	       sizeof(tmp));

    tmp = 1;
    setsockopt(dbg_sock,
               pe->p_proto,
               TCP_NODELAY,
               (char *) &tmp,
               sizeof(tmp));

#if defined(WIN32)
    closesocket(dbg_listen_sock);
    dbg_listen_sock = dbg_listen_sock;
#else
    close(dbg_listen_sock);
    dbg_listen_sock = -1;

    /* If we don't do this, then proxy simply
       exits when the remote side dies. */
    signal (SIGPIPE, SIG_IGN);
#endif

    return  TRUE;
}

/* Close connection to debugger side */
void dbg_sock_close(void)
{
#if defined(WIN32)
    if (dbg_sock != INVALID_SOCKET)
    {
        closesocket(dbg_sock);
        dbg_sock = INVALID_SOCKET;
    }

    if (dbg_listen_sock != INVALID_SOCKET)
    {
        closesocket(dbg_listen_sock);
        dbg_listen_sock = INVALID_SOCKET;
    }
#else
    if (dbg_sock >= 0)
    {
        close(dbg_sock);
        dbg_sock = -1;
    }

    if (dbg_listen_sock >= 0)
    {
        close(dbg_listen_sock);
        dbg_listen_sock = -1;
    }
#endif
}

/* Send character to debugger */
void dbg_sock_putchar(int c)
{
#if !defined(WIN32)
    int  ret;
#endif
    char b[4];

#if defined(WIN32)
    assert(dbg_sock != INVALID_SOCKET);
#endif

    b[0] = c & 0xff;

#if defined(WIN32)
    send(dbg_sock, b, 1, 0);
#else
    do
    {
        ret = send(dbg_sock, b, 1, 0);
    }
    while (ret < 0  &&  errno == EINTR);
#endif
}

/* Read character with timeout, return charcter code, if timeout,
   or error return RP_VAL_MISC....  ms  is timeout in millisaconds
   or -1 for no timeout */
int dbg_sock_readchar(int ms)
{
    static uint8_t buf[PACKET_BUFF_SIZE];
    static int count = 0;
    static uint8_t *p;
    int ret;
    fd_set rset;

#if defined(WIN32)
    assert(dbg_sock != INVALID_SOCKET);
#else
    assert(dbg_sock >= 0);
#endif
    assert(ms == -1  ||  ms >= 0);

    if (count > 0)
    {
        ret = *p++ & 0xff;  /* 8-bit transparent */
        count--;
        return  ret;
    }

    FD_ZERO(&rset);
    FD_SET(dbg_sock, &rset);

#if defined(WIN32)
    if (ms != -1)
    {
        struct timeval tv;
        /* Let us calculate timeout */
        tv.tv_sec = ms/1000;
        tv.tv_usec = (ms%1000)*1000;

        ret = select(dbg_sock + 1, &rset, NULL, NULL, &tv);
    }
    else
    {
        ret = select(dbg_sock + 1, &rset, NULL, NULL, NULL);
    }

    if (ret == SOCKET_ERROR)
        return RP_VAL_MISCREADCHARRET_ERR;

    if (ret == 0)
        return RP_VAL_MISCREADCHARRET_TMOUT;

    assert(FD_ISSET(dbg_sock, &rset));
#else
    if (ms != -1)
    {
        struct timeval tv;
        struct timeval end;
        struct timeval cur;
        struct timezone tz;

        /* It is quite possible that targets might install
           some signal handlers, so we have to ignore EINTR
           in all cases */

        /* Let us calculate the timeout */
        tv.tv_sec = ms/1000;
        tv.tv_usec = (ms%1000)*1000;

        /* Let us get current time */
        if ((ret = gettimeofday(&cur, &tz)) < 0)
            return RP_VAL_MISCREADCHARRET_ERR;

        /* Let us figure out end time */
        end.tv_sec  = cur.tv_sec + tv.tv_sec;
        end.tv_usec = cur.tv_usec + tv.tv_usec;

        if (end.tv_usec >= 1000000)
        {
            assert(end.tv_usec < 2000000);
            end.tv_sec++;
            end.tv_usec -= 1000000;
        }

        for (;;)
        {
            FD_ZERO(&rset);
            FD_SET(dbg_sock, &rset);

            if ((ret = select(dbg_sock + 1, &rset, NULL, NULL, &tv)) > 0)
            {
                assert(FD_ISSET(dbg_sock, &rset));
                break;
            }

            if (ret < 0  &&  errno != EINTR)
                return RP_VAL_MISCREADCHARRET_ERR;

            if (ret == 0)
            {
                /* Timeout */
                return RP_VAL_MISCREADCHARRET_TMOUT;
            }

            /* We have been interrupted by a signal */

            /* We have to recalculate the remaining timeout */
            if ((ret = gettimeofday(&cur, &tz)) < 0)
                return RP_VAL_MISCREADCHARRET_ERR;

            if (cur.tv_sec > end.tv_sec
                ||
                (cur.tv_sec == end.tv_sec  &&  cur.tv_usec >= end.tv_usec))
            {
                tv.tv_sec = 0;
                tv.tv_usec = 0;
                continue;
            }

            tv.tv_sec = end.tv_sec - cur.tv_sec;

            if (cur.tv_usec <= end.tv_usec)
            {
                tv.tv_usec = end.tv_usec - cur.tv_usec;
            }
            else
            {
                assert(tv.tv_sec > 0);

                tv.tv_sec--;
                tv.tv_usec = end.tv_usec + 1000000 - cur.tv_usec;
            }
        }
    }
    else
    {
        do
        {
            ret = select(dbg_sock + 1, &rset, NULL, NULL, NULL);
        }
        while (ret < 0  &&  errno == EINTR);
        if (ret < 0)
            return RP_VAL_MISCREADCHARRET_ERR;
    }
#endif

#if defined(WIN32)
    count = recv(dbg_sock, (char*) buf, sizeof (buf), 0);
    if (count == 0  ||  count == SOCKET_ERROR)
#else
    /* We need to ignore EINTR */
    do
    {
        count = recv(dbg_sock, buf, sizeof (buf), 0);
    }
    while (count < 0  &&  errno == EINTR);
    if (count <= 0)
#endif
    {
        /* Either eof or error: in all cases return an error */
        return RP_VAL_MISCREADCHARRET_ERR;
    }

    ret = buf[0] & 0xff;
    p = &buf[1];
    count--;

    return ret;
}

/* Send buffer to debugger */
int dbg_sock_write(unsigned char *data, size_t len)
{
    int ret;

    assert(data != NULL);
    assert(len > 0);

#if defined(WIN32)
    ret = send(dbg_sock, (char*) data, len, 0);
    if (ret == SOCKET_ERROR  ||  ret != (int) len)
        return  FALSE;
#else
    do
    {
        ret = send(dbg_sock, data, len, 0);
    }
    while (ret < 0  &&  errno == EINTR);
    if (ret != (int) len)
        return  FALSE;
#endif

    return  TRUE;
}

#if defined(WIN32)
log_func rp_env_init(char *name, int do_daemon)
{
    assert(name != NULL);
    assert(!do_daemon);

    return rp_log_local;
}
#else
log_func rp_env_init(char *name, int do_daemon)
{
    pid_t childpid;
    int fd;

    if (!do_daemon)
	return  rp_log_local;

    /* `fork()' so the parent can exit, this returns control to the command
       line or shell invoking your program.  This step is required so that
       the new process is guaranteed not to be a process group leader. The
       next step, `setsid()', fails if you're a process group leader. */

    if ((childpid = fork ()) < 0)
    {
        perror ("Can't fork daemon process");
        exit (1);
    }
    /*endif*/
    if (childpid > 0)
    {
        /* Now in parent.  Kill if off */
        exit (0);
    }
    /*endif*/

    /* `setsid()' to become a process group and session group leader. Since a
        controlling terminal is associated with a session, and this new
        session has not yet acquired a controlling terminal our process now
        has no controlling terminal, which is a Good Thing for daemons. */
    if (setsid () == -1)
    {
        perror ("Can't create a new session");
        exit (1);
    }
    /*endif*/

    /* `fork()' again so the parent, (the session group leader), can exit.
       This means that we, as a non-session group leader, can never regain a
       controlling terminal. */
    if ((childpid = fork ()) < 0)
    {
        perror ("Can't fork daemon process");
        exit (1);
    }
    /*endif*/
    if (childpid > 0)
    {
        /* Now in parent.  Kill if off */
        exit (0);
    }
    /*endif*/

    /* `chdir("/")' to ensure that our process doesn't keep any directory in
       use. Failure to do this could make it so that an administrator
       couldn't unmount a filesystem, because it was our current directory.
       If the caller wants a different working directory they can always
       set their own later */
    if (chdir ("/") < 0)
    {
        perror ("Can't chdir to /");
        exit (1);
    }

    /* `umask(0)' so that we have complete control over the permissions of
       anything we write. We don't know what umask we may have inherited.
       [This step is not essential, but seems a good idea.] */
    umask(0);

    /* Close fds 0, 1, and 2. This releases the standard in, out, and
       error we inherited from our parent process. We have no way of knowing
       where these fds might have been redirected to. Note that many daemons
       use `sysconf()' to determine the limit `_SC_OPEN_MAX'.  `_SC_OPEN_MAX'
       tells you the maximun open files/process. Then in a loop, the daemon
       can close all possible file descriptors. You have to decide if you
       need to do this or not.  If you think that there might be
       file-descriptors open you should close them, since there's a limit on
       number of concurrent file descriptors. */

    /* Establish new open descriptors for stdin, stdout and stderr. Even if
       you don't plan to use them, it is still a good idea to have them open.
       The precise handling of these is a matter of taste; if you have a
       logfile, for example, you might wish to open it as stdout or stderr,
       and open `/dev/null' as stdin; alternatively, you could open
       `/dev/console' as stderr and/or stdout, and `/dev/null' as stdin, or
       any other combination that makes sense for your particular daemon.
       A number of systems treat file descriptors 0, 1 and 2 specially, and
       can cause a lot of trouble if they are left to be used as ordinary
       files. */
    if ((fd = open ("/dev/null", O_RDWR, 0)) != -1)
    {
        dup2 (fd, 0);
        dup2 (fd, 1);
        dup2 (fd, 2);
        if (fd > 2)
            close (fd);
        /*endif*/
    }
    /*endif*/
    umask (0);

    openlog(name, LOG_PID, LOG_DAEMON);

    return  rp_log_system;
}
#endif

static void rp_log_local(int level, const char *fmt, ...)
{
    va_list args;

    /* Convert our log level values to system ones */
    switch (level) {
    case RP_VAL_LOGLEVEL_DEBUG:
        if (debug_level < 1)
            return;
        break;
    case RP_VAL_LOGLEVEL_DEBUG2:
        if (debug_level < 2)
            return;
        break;
    case RP_VAL_LOGLEVEL_DEBUG3:
        if (debug_level < 3)
            return;
        break;
    }

    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);

    fprintf(stderr, "\n");
    fflush(stderr);
}

#if !defined(WIN32)
static void rp_log_system(int level, const char *fmt, ...)
{
    va_list args;
    int priority;

    /* Convert our log level values to system ones */
    switch (level) {
    case RP_VAL_LOGLEVEL_EMERG:
        priority = LOG_EMERG;
        break;
    case RP_VAL_LOGLEVEL_ALERT:
        priority = LOG_ALERT;
        break;
    case RP_VAL_LOGLEVEL_CRIT:
        priority = LOG_CRIT;
        break;
    case RP_VAL_LOGLEVEL_ERR:
        priority = LOG_ERR;
        break;
    case RP_VAL_LOGLEVEL_WARNING:
        priority = LOG_WARNING;
        break;
    case RP_VAL_LOGLEVEL_NOTICE:
        priority = LOG_NOTICE;
        break;
    case RP_VAL_LOGLEVEL_INFO:
        priority = LOG_INFO;
        break;
    case RP_VAL_LOGLEVEL_DEBUG:
        if (debug_level < 1)
            return;
        priority = LOG_DEBUG;
        break;
    case RP_VAL_LOGLEVEL_DEBUG2:
        if (debug_level < 2)
            return;
        priority = LOG_DEBUG;
        break;
    case RP_VAL_LOGLEVEL_DEBUG3:
        if (debug_level < 3)
            return;
        priority = LOG_DEBUG;
        break;
    default:
        assert(0);
        priority = LOG_WARNING;
        break;
    }

    va_start(args, fmt);
    vsyslog(priority, fmt, args);
    va_end(args);
}
#endif
