all : flash

TARGET:=usbhost_msc_main
TARGET_MCU:=CH32V203

# Shared enumeration core (Phase A/B/C) + MSC/BOT class driver
# (Phase D) + Petit FatFs (Phase E).
#
# ch32fun.mk adds $(TARGET).c automatically (= usbhost_msc_main.c), so
# we only list the *additional* sources here.  PFF lives in its own
# subfolder but gets compiled in the same translation unit via -I.
ADDITIONAL_C_FILES:=usbhost_hw.c usbhost_xfer.c usbhost_std.c \
                    usbhost_msc.c \
                    pff/pff.c pff/diskio.c

# Petit FatFs header is pff/pff.h, so add the subfolder to the include
# path. ch32fun.mk appends EXTRA_CFLAGS to its CFLAGS line, so this
# is the ch32fun-approved way to add per-project -I.
EXTRA_CFLAGS:=-I pff

include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean
