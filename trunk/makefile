CC		= gcc

ifneq (,$(wildcard .svn/entries))
SVNVERS         = $(shell head -n4 .svn/entries | tail -n1)
else
SVNVERS         = $(shell svnversion)
endif
UNAME           = $(shell uname)
CFLAGS		= -Wall -g -O -Ihidapi -DSVNVERSION='"$(SVNVERS)"'
LDFLAGS		=

# Linux
ifeq ($(UNAME),Linux)
    LIBS        += -lusb-1.0 -lpthread
    HIDSRC      = hidapi/hid-libusb.c
endif

# Mac OS X
ifeq ($(UNAME),Darwin)
    LIBS        += -framework IOKit -framework CoreFoundation
    HIDSRC      = hidapi/hid-mac.c
endif

OBJS		= gdbproxy.o rpmisc.o target-ejtag.o hid.o \
                  proxy-mips.o adapter-pickit2.o

# Olimex ARM-USB-Tiny JTAG adapter.
CFLAGS          += -DUSE_MPSSE
OBJS            += adapter-mpsse.o
ifeq ($(UNAME),Linux)
    # Use 'sudo port install libusb-0.1-dev'
    LIBS        += -lusb
endif
ifeq ($(UNAME),Darwin)
    # Use 'sudo port install libusb-legacy'
    CFLAGS      += -I/opt/local/include/libusb-legacy
    LIBS        += -L/opt/local/lib/libusb-legacy -lusb-legacy
endif

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
