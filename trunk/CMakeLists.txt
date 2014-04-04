project (ejtagproxy C)
cmake_minimum_required (VERSION 2.4)

#
# Extract Subversion revision number
#
include (FindSubversion)
Subversion_WC_INFO (.${SOURCE_DIR} MY)
cmake_policy (SET CMP0005 OLD)

message ("Build ejtagproxy revision r${MY_WC_REVISION} on ${CMAKE_SYSTEM_NAME}")
message ("Compiler is ${CMAKE_C_COMPILER}")

# Compilation flags
set (CMAKE_C_FLAGS_DEBUG "-Wall")
#set (CMAKE_BUILD_TYPE DEBUG)
set (CMAKE_BUILD_TYPE release)

#
# Mac OS X: need IOKit
#
if (APPLE)
    set (HID_SRC hidapi/hid-mac.c)
    find_library (IOKIT_LIB IOKit)
    find_library (COREFOUNDATION_LIB CoreFoundation)
    message (STATUS "Found IOKit framework: ${IOKIT_LIB}")
#
# Linux: need libusb-1.0
#
elseif (UNIX)
    set (HID_SRC hidapi/hid-libusb.c)
    find_library (USB_LIB usb-1.0)
    find_library (PTHREAD_LIB pthread)
    message (STATUS "Found USB library: ${USB_LIB}")
endif()

#
# FT2232-based adapters: Olimex ARM-USB-Tiny and others.
# Need legacy libusb-0.1 library.
#
find_path (USBLEGACY_H usb.h
    /usr/include
    /opt/local/include/libusb-legacy
)
find_library (USBLEGACY_LIB
    NAMES usb usb-legacy
    PATHS /usr/lib
          /opt/local/lib/libusb-legacy
)
if (USBLEGACY_LIB AND USBLEGACY_H)
    add_definitions (-DUSE_MPSSE)
    set (MPSSE_SRC adapter-mpsse.c)
    message (STATUS "Found legacy USB library: ${USBLEGACY_LIB}")
else()
    message (STATUS "Legacy USB library not found: support for FTDI adapters disabled")
endif()

#
# Configure the target executable
#
add_definitions (-DSVNVERSION='"${MY_WC_REVISION}"')
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/hidapi ${USBLEGACY_H})

add_executable (ejtagproxy
    gdbproxy.c
    rpmisc.c
    target-ejtag.c
    ${HID_SRC}
    proxy-mips.c
    adapter-pickit2.c
    ${MPSSE_SRC}
)

target_link_libraries (ejtagproxy
    ${USB_LIB}
    ${USBLEGACY_LIB}
    ${PTHREAD_LIB}
    ${IOKIT_LIB}
    ${COREFOUNDATION_LIB}
)
