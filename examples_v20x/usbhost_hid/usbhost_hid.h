// usbhost_hid.h
//
// USB HID class driver for the ch32fun USB host stack.
//
// Supports three device classes:
//   - Boot-protocol keyboard   (8-byte fixed report)
//   - Boot-protocol mouse      (3-byte fixed report)
//   - Report-protocol mouse / joystick / gamepad / multi-button mouse
//     with a HID Report Descriptor we parse dynamically. Axes, up to
//     12 buttons, a hat, and a wheel are extracted.
//
// Xbox 360 controllers (VID 0x045E, PID 0x028E, plus a few clones)
// are special-cased: they present a vendor-specific interface
// (class 0xFF, subclass 0x5D, protocol 0x01) with a fixed 20-byte
// input report, not a HID Report Descriptor.

#ifndef USBHOST_HID_H
#define USBHOST_HID_H

#include <stdint.h>
#include "usbhost_defs.h"
#include "usbhost_reportparser.h"

#ifdef __cplusplus
extern "C" {
#endif

// Class driver kinds (returned by USBH_HidInit).
#define USBH_HID_KIND_NONE      0
#define USBH_HID_KIND_KEYBOARD  1
#define USBH_HID_KIND_MOUSE     2
#define USBH_HID_KIND_GAMEPAD   3
#define USBH_HID_KIND_XBOX360   4

// Maximum HID report length we handle. The WCH SIE's full-speed
// bulk/interrupt-IN FIFO is 64 bytes; everything in scope (mice,
// keyboards, consumer gamepads, Xbox 360 wired controller) fits.
#define USBH_HID_REPORT_MAX     64

// Decoded input reports. The HID driver fills one of these from
// each interrupt-IN transfer.

// Boot keyboard: [mod, reserved, k0..k5]. 8 bytes.
typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} USBH_HidKbdReport;

// Boot mouse: [buttons, X, Y]. 3 bytes.
typedef struct {
	uint8_t buttons;
	int8_t  x;
	int8_t  y;
} USBH_HidMouseReport;

// Generic decoded input: up to 4 axes (X / Y / Rx / Ry), wheel, and
// 12 button bits. Values are the *raw* field values pulled out of
// the report, no centring. The application is expected to apply
// dead-zones and centring using pdesc->axis[c].logical.{min,max}.
typedef struct {
	int16_t axis[4];      // axis[0]=X, [1]=Y, [2]=Z/Rx, [3]=Rz/Ry
	int16_t wheel;
	uint8_t buttons;       // primary 8 buttons (1 bit each)
	uint8_t buttons_extra; // buttons 8..11 packed, 0 if not used
} USBH_HidInputReport;

// Xbox 360 wired controller, 20-byte input report.
typedef struct {
	uint8_t  buttons_low;   // byte 2 (dpad, start, back, L3, R3)
	uint8_t  buttons_high;  // byte 3 (LB, RB, guide, A, B, X, Y)
	uint8_t  lt;            // byte 4 (left trigger, 0..255)
	uint8_t  rt;            // byte 5 (right trigger, 0..255)
	int16_t  lx, ly;        // bytes 6..9  (left stick, signed 16-bit LE)
	int16_t  rx, ry;        // bytes 10..13 (right stick, signed 16-bit LE)
} USBH_HidX360Report;

// Set up the attached HID device:
//   - find the first HID interface (boot- or report-protocol)
//   - issue SET_IDLE 0
//   - issue SET_PROTOCOL boot (only for boot-protocol devices)
//   - for non-boot-protocol: GET_DESCRIPTOR(HID_REPORT) + parse it
//   - for Xbox 360: leave it uninitialised and just hand back
//     a fixed-size 20-byte report buffer
//
// On success returns USBH_ERR_SUCCESS and fills:
//   *pkind      – one of USBH_HID_KIND_*
//   *prep_len   – the report length the device will send, in bytes
//   *pin_ep     – interrupt-IN endpoint number (low 4 bits)
//   *pin_maxp   – endpoint wMaxPacketSize
//   *pin_interval – endpoint bInterval
//   *pdesc      – parsed HID Report Descriptor (only meaningful when
//                 *pkind is MOUSE/GAMEPAD and not Xbox 360)
//
// *pdesc may be NULL on input if the caller doesn't need it.
//
// For boot-protocol devices the descriptor parser is skipped and
// *pdesc is left zeroed.
uint8_t USBH_HidInit( const uint8_t *dev_desc,
                      const uint8_t *cfg_desc, uint16_t cfg_len,
                      uint8_t *pkind,
                      uint8_t *prep_len,
                      uint8_t *pin_ep,
                      uint16_t *pin_maxp,
                      uint8_t *pin_interval,
                      hid_report_t *pdesc );

// Read the next report from the interrupt-IN endpoint (blocking,
// with retries; returns USBH_ERR_USB_DISCON on detach). The caller
// supplies the report buffer; the SIE fills it with up to max_len
// bytes. After the call, *pnread holds the number of bytes actually
// received.
uint8_t USBH_HidReadReport( uint8_t ep, uint8_t *ptog, uint16_t maxp,
                            void *report, uint8_t max_len,
                            uint8_t *pnread );

// Map a USB HID boot keyboard usage ID to ASCII. Returns 0 for keys
// with no printable equivalent (modifiers, F-keys, etc). Shift is
// handled separately by the caller (we expose the modifier byte so
// the caller can decide).
uint8_t USBH_HidKeyToAscii( uint8_t usage, uint8_t shift );

// Decode a raw HID report into a uniform USBH_HidInputReport using
// the parsed descriptor. Safe to call with a zeroed pdesc (all
// fields are returned as 0 in that case).
//
// `kind` may be USBH_HID_KIND_MOUSE or USBH_HID_KIND_GAMEPAD. The
// Xbox 360 path uses a separate function.
void USBH_HidDecodeGeneric( const hid_report_t *pdesc, uint8_t kind,
                            const uint8_t *raw, uint8_t raw_len,
                            USBH_HidInputReport *out );

// Decode the 20-byte Xbox 360 wired-controller report. Returns 0 on
// success, non-zero if the report header isn't 0x00 0x14.
int USBH_HidDecodeX360( const uint8_t *raw, uint8_t raw_len,
                        USBH_HidX360Report *out );

#ifdef __cplusplus
}
#endif

#endif // USBHOST_HID_H
