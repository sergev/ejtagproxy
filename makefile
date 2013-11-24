CC		= gcc

SVNVERS         = $(shell svnversion)
CFLAGS		= -Wall -g -O -I/opt/local/include -Ihidapi -DSVNVERSION='"$(SVNVERS)"'
LDFLAGS		=

# Linux
ifneq (,$(wildcard /lib/i386-linux-gnu))
    LIBS        += -lusb-1.0 -lpthread
    HIDSRC      = hidapi/hid-libusb.c
endif

# Mac OS X
ifneq (,$(wildcard /System/Library/Frameworks/CoreFoundation.framework))
    LIBS        += -framework IOKit -framework CoreFoundation
    HIDSRC      = hidapi/hid-mac.c
endif

OBJS		= gdbproxy.o rpmisc.o target-ejtag.o hid.o \
                  proxy-mips.o adapter-pickit2.o

# Olimex ARM-USB-Tiny JTAG adapter.
CFLAGS          += -DUSE_MPSSE
OBJS            += adapter-mpsse.o
LIBS            += -L/opt/local/lib -lusb

all:		ejtagproxy

ejtagproxy:	$(OBJS)
		$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

hid.o:          $(HIDSRC)
		$(CC) $(CFLAGS) -c -o $@ $<

clean:
		rm -f *~ *.o core ejtagproxy

install:	ejtagproxy
		install -c -s ejtagproxy /usr/local/bin/ejtagproxy
###
adapter-mpsse.o: adapter-mpsse.c adapter.h mips.h ejtag.h
adapter-pickit2.o: adapter-pickit2.c adapter.h hidapi/hidapi.h pickit2.h \
    mips.h ejtag.h
gdbproxy.o: gdbproxy.c gdbproxy.h
proxy-mips.o: proxy-mips.c gdbproxy.h target.h mips.h
proxy-skeleton.o: proxy-skeleton.c gdbproxy.h
rpmisc.o: rpmisc.c gdbproxy.h
target-ejtag.o: target-ejtag.c target.h adapter.h mips.h ejtag.h
hid.o: hidapi/hid-linux.c hidapi/hid-mac.c hidapi/hidapi.h
