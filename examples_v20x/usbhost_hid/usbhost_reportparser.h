// usbhost_reportparser.h
//
// HID Report Descriptor parser for the ch32fun USB host stack.
//
// Loosely adapted from the reference firmware's
// `USB_Host/usb_hid_reportparser.{c,h}`.  Differences from the
// reference:
//   - `report_id` and the offsets are stored in plain types (no
//     bitfields compressed into a packed struct) so they're usable
//     from the host code on a CH32V203 without worrying about
//     the toolchain's packing rules.
//   - Supports 12 buttons (matching the reference parser) and a
//     single hat/wheel; multiple logical collections are tolerated.
//
// Usage:
//   hid_report_t conf;
//   if (parse_report_descriptor(rep, rep_size, &conf)) {
//       // conf.report_id   -- 0 if no Report ID, else 1..255
//       // conf.report_size -- total report length in bytes (incl. id)
//       // conf.type        -- REPORT_TYPE_*
//       // conf.joystick_mouse.axis[0..1]  -- X / Y
//       // conf.joystick_mouse.button[0..11] -- up to 12 buttons
//       // conf.joystick_mouse.hat        -- hat (joysticks only)
//       // conf.joystick_mouse.wheel      -- wheel (mice only)
//   }

#ifndef USBHOST_REPORTPARSER_H
#define USBHOST_REPORTPARSER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USBH_REPORT_TYPE_NONE     0
#define USBH_REPORT_TYPE_MOUSE    1
#define USBH_REPORT_TYPE_KEYBOARD 2
#define USBH_REPORT_TYPE_JOYSTICK 3

#define USBH_REPORT_AXES     2
#define USBH_REPORT_BUTTONS  12

typedef struct {
	uint16_t offset;
	uint8_t  size;
	struct {
		uint16_t min;
		uint16_t max;
	} logical;
} hid_axis_t;

typedef struct {
	uint16_t offset;
	uint8_t  size;
	struct {
		uint16_t min;
		uint16_t max;
	} logical;
} hid_wheel_t;

typedef struct {
	uint8_t byte_offset;
	uint8_t bitmask;
} hid_button_t;

typedef struct {
	uint8_t     type;        // USBH_REPORT_TYPE_*
	uint8_t     report_id;   // 0 if the descriptor has no Report ID
	uint8_t     report_size; // total report length in bytes (incl. id byte if present)

	hid_axis_t  axis[USBH_REPORT_AXES];
	hid_button_t button[USBH_REPORT_BUTTONS];
	hid_wheel_t hat;
	hid_wheel_t wheel;

	uint8_t     button_count; // number of distinct bits consumed by buttons
} hid_report_t;

// Parse a HID Report Descriptor. Returns 1 if a usable device
// (keyboard, mouse, or joystick/gamepad) was found, 0 otherwise.
// On success, *conf is filled in with offsets and sizes for the
// interesting fields.
int parse_report_descriptor( const uint8_t *rep, uint16_t rep_size, hid_report_t *conf );

// If the last call to parse_report_descriptor() returned 0, this
// returns a diagnostic code: ((type << 8) | tag) of the offending
// short item, or 0xFFFF if the parser walked off the end without
// finding a usable layout. *poff (if non-NULL) receives the byte
// offset of the offending item within the descriptor.
uint16_t usbhost_reportparser_last_err( uint16_t *poff );

// Extract a bit field (signed or unsigned) from a HID report
// (used to read axis / wheel values out of a report buffer).
uint16_t collect_bits( const uint8_t *p, uint16_t offset, uint8_t size, int is_signed );

#ifdef __cplusplus
}
#endif

#endif // USBHOST_REPORTPARSER_H
