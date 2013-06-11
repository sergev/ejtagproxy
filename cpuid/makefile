CC		= gcc

SVNVERS         = $(shell svnversion)
CFLAGS		= -Wall -g -O -I/opt/local/include -I.. -DSVNVERSION='"$(SVNVERS)"'
LDFLAGS		= -g

# Linux
ifneq (,$(wildcard /lib/i386-linux-gnu))
    LIBS        += -lusb-1.0
    HIDSRC      = ../hidapi/hid-libusb.c
endif

# Mac OS X
ifneq (,$(wildcard /System/Library/Frameworks/CoreFoundation.framework))
    LIBS        += -framework IOKit -framework CoreFoundation
    HIDSRC      = ../hidapi/hid-mac.c
endif

OBJS		= hid.o cpuid.o adapter-mpsse.o

# FT2232 JTAG adapter.
CFLAGS          += -DUSE_MPSSE
LIBS            += -L/opt/local/lib -lusb

all:		cpuid

cpuid:          $(OBJS)
		$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

hid.o:          $(HIDSRC)
		$(CC) $(CFLAGS) -c -o $@ $<

adapter-mpsse.o: ../adapter-mpsse.c
		$(CC) $(CFLAGS) -c -o $@ $<

clean:
		rm -f *~ *.o core cpuid

install:	cpuid
		install -c -s cpuid /usr/local/bin/cpuid
###
adapter-mpsse.o: ../adapter-mpsse.c ../adapter.h ../mips.h ../ejtag.h
cpuid.o: cpuid.c ../target.h ../adapter.h ../mips.h ../ejtag.h
hid.o: ../hidapi/hid-linux.c ../hidapi/hid-mac.c ../hidapi/hidapi.h
