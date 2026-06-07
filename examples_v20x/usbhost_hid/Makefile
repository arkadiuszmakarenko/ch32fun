all : flash

TARGET:=usbhost_hid
TARGET_MCU:=CH32V203

# Shared enumeration core (Phase A/B/C) + HID class driver (Phase D).
# Note: usbhost_enum.c is the project-local main() that overrides
# the weak USBH_OnEnumSuccess() callback to bring up HID.
# ch32fun.mk adds $(TARGET).c automatically (= usbhost_hid.c), so we
# only list the *additional* sources here.
ADDITIONAL_C_FILES:=usbhost_hw.c usbhost_xfer.c usbhost_std.c usbhost_enum.c

include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean
