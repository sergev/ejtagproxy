
EJTAGproxy is a utility for debugging PIC32 microcontrollers with GDB
via JTAG or ICSP adapter.  Supported adapters:

 * Microchip PICkit2
 * Microchip PICkit3 with scripting firmware
 * Olimex ARM-USB-Tiny
 * Olimex ARM-USB-Tiny-H

Usage:
    ejtagproxy [options]

Options:
    -d, --daemon        run as daemon
    -p, --port=PORT     use the specified TCP port (default 2000)
    -D, --debug         output debug messages

To start a debug session:
1) Run ejtagproxy.
2) Attach a USB adapter to target board.
3) Run gdb and connect to the PIC32 target:
    set remote hardware-breakpoint-limit 6
    set remote hardware-watchpoint-limit 2
    target remote localhost:2000

When gdb session is closed, ejtagproxy disconnects from the target board.
You can use other tools, like pic32prog, to update the target software
and then start a new gdb session.  No need to restart ejtagproxy between
sessions.  You can safely run it as a daemon.

To build on Linux or Mac OS X, run:
    make
    make install

To build on Windows using MINGW compiler, run:
    gmake -f make-mingw

Based on sources of:
 * GDBproxy project by Steve Underwood: http://gdbproxy.sourceforge.net/
 * PIC32prog project by Serge Vakulenko: http://code.google.com/p/pic32prog/

List of supported processors:
* PIC32MX1xx/2xx family:
    mx110f016b, mx110f016c, mx110f016d, mx120f032b, mx120f032c, mx120f032d,
    mx130f064b, mx130f064c, mx130f064d, mx150f128b, mx150f128c, mx150f128d,
    mx210f016b, mx210f016c, mx210f016d, mx220f032b, mx220f032c, mx220f032d,
    mx230f064b, mx230f064c, mx230f064d, mx250f128b, mx250f128c, mx250f128d.
 * PIC32MX3xx/4xx family:
    mx320f032h, mx320f064h, mx320f128h, mx320f128l, mx340f128h, mx340f128l,
    mx340f256h, mx340f512h, mx360f256l, mx360f512l, mx420f032h, mx440f128h,
    mx440f128l, mx440f256h, mx440f512h, mx460f256l, mx460f512l.
* PIC32MX5xx/6xx/7xx family:
    mx534f064h, mx534f064l, mx564f064h, mx564f064l, mx564f128h, mx564f128l,
    mx575f256h, mx575f256l, mx575f512h, mx575f512l, mx664f064h, mx664f064l,
    mx664f128h, mx664f128l, mx675f256h, mx675f256l, mx675f512h, mx675f512l,
    mx695f512h, mx695f512l, mx764f128h, mx764f128l, mx775f256h, mx775f256l,
    mx775f512h, mx775f512l, mx795f512h, mx795f512l.
