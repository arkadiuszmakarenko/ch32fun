// usbhost_hid.h -- minimal USB HID class driver on top of usbhost_xfer.
//
// Scope: claim the first HID interface, switch to boot protocol, and
// hand callers a one-shot "read the next report" function. Keyboard
// and mouse boot reports are the only thing decoded (boot keyboard
// scancodes -> ASCII in main()).
//
// This driver is intentionally tiny: it does NOT parse the HID Report
// Descriptor. Boot protocol uses a fixed layout per device class:
//   - Boot keyboard: 8-byte report - modifier byte, reserved byte,
//     up to 6 keycodes, in USB HID usage order.
//   - Boot mouse:    3-byte report - buttons, X, Y (relative).
// Anything else (non-boot HID devices, report-protocol HID) will
// fail the init phase and the caller will see USBH_ERR_USB_HID_NOBOOT.

#ifndef USBHOST_HID_H
#define USBHOST_HID_H

#include <stdint.h>
#include "usbhost_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// 8-byte boot keyboard report: [mod, reserved, k0, k1, k2, k3, k4, k5].
typedef struct {
	uint8_t modifier;
	// bit 0=LCTRL, 1=LSHIFT, 2=LALT, 3=LGUI,
	// bit 4=RCTRL, 5=RSHIFT, 6=RALT, 7=RGUI
	uint8_t reserved;
	uint8_t keycode[6];
	// USB HID usage IDs (0x04 = 'a', 0x05 = 'b', ...)
} USBH_HidKbdReport;

typedef struct {
	uint8_t buttons;
	// bit 0=L, 1=R, 2=M
	int8_t  x;
	int8_t  y;
} USBH_HidMouseReport;

// Set up the attached HID device:
//   - find the first HID interface
//   - SET_IDLE 0
//   - SET_PROTOCOL boot
//   - SET_CONFIGURATION is assumed to have been done by the enumerator
// On success returns USBH_ERR_SUCCESS and fills *prep_len (the boot
// report size, 8 for keyboard, 3 for mouse) and the interrupt-IN
// endpoint address, max packet size, and polling interval.
uint8_t USBH_HidInit( const uint8_t *cfg_desc, uint16_t cfg_len,
	uint8_t *prep_len,
	uint8_t *pin_ep,
	uint16_t *pin_maxp,
	uint8_t *pin_interval );

// Read the next boot-protocol report from the interrupt-IN endpoint
// (blocking, with retries; returns USBH_ERR_USB_DISCON on detach).
//   ep   - endpoint number (low 4 bits)
//
// The caller passes a per-endpoint toggle byte (initialise to 0).
uint8_t USBH_HidReadReport( uint8_t ep, uint8_t *ptog, uint16_t maxp,
	void *report, uint8_t report_size );

// Map a USB HID boot keyboard usage ID to ASCII. Returns 0 for keys
// with no printable equivalent (modifiers, F-keys, etc). Shift is
// handled separately by the caller (we expose the modifier byte so
// the caller can decide).
uint8_t USBH_HidKeyToAscii( uint8_t usage, uint8_t shift );

#ifdef __cplusplus
}
#endif

#endif // USBHOST_HID_H
