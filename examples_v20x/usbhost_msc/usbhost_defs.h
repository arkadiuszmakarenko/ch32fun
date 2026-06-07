// usbhost_defs.h
//
// USB constants ch32fun's ch32v20xhw.h does not define. These come from
// the USB 2.0 spec and the WCH EVT peripheral library; we re-define
// just the ones we need so the host code does not depend on any vendor
// header.
//
// All values are little-endian on the wire.

#ifndef USBHOST_DEFS_H
#define USBHOST_DEFS_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Token PIDs. The WCH SIE expects the 4-bit PID code in the top nibble of
// HOST_EP_PID and the endpoint number in the bottom nibble:
//   HOST_EP_PID = (pid_code << 4) | endpoint
// These values match the WCH ch32v20x_usb.h definitions.
// ---------------------------------------------------------------------------
#define USBH_PID_SETUP   0x0Du  // token PID code for SETUP
#define USBH_PID_OUT     0x01u  // token PID code for OUT
#define USBH_PID_IN      0x09u  // token PID code for IN

// ---------------------------------------------------------------------------
// Handshake/data response codes from INT_ST bits[3:0] after a transaction.
// The SIE stores the raw 4-bit PID code, not the full 8-bit USB PID byte.
// ---------------------------------------------------------------------------
#define USBH_PID_ACK     0x02u  // ACK handshake code
#define USBH_PID_NAK     0x0Au  // NAK handshake code
#define USBH_PID_STALL   0x0Eu  // STALL handshake code
#define USBH_PID_DATA0   0x03u  // DATA0 PID code
#define USBH_PID_DATA1   0x0Bu  // DATA1 PID code
#define USBH_PID_TIMEOUT 0x00u  // our own sentinel, not a USB PID

// ---------------------------------------------------------------------------
// USB request types (byte 0 of an 8-byte SETUP packet).
// ---------------------------------------------------------------------------
#define USBH_REQ_DIR_HOST_TO_DEVICE 0x00u
#define USBH_REQ_DIR_DEVICE_TO_HOST 0x80u

#define USBH_REQ_TYPE_STANDARD   0x00u
#define USBH_REQ_TYPE_CLASS      0x20u
#define USBH_REQ_TYPE_VENDOR     0x40u

#define USBH_REQ_RCPT_DEVICE     0x00u
#define USBH_REQ_RCPT_INTERFACE  0x01u
#define USBH_REQ_RCPT_ENDPOINT   0x02u
#define USBH_REQ_RCPT_OTHER      0x03u

// Pre-baked byte 0 helpers. (Named differently from the request codes
// above to keep the preprocessor from getting confused when both forms
// are used in the same file.)
#define USBH_REQ_GET_DEVICE_DESC_BYTE \
	( USBH_REQ_DIR_DEVICE_TO_HOST | USBH_REQ_TYPE_STANDARD | USBH_REQ_RCPT_DEVICE )
#define USBH_REQ_SET_ADDRESS_BYTE \
	( USBH_REQ_DIR_HOST_TO_DEVICE | USBH_REQ_TYPE_STANDARD | USBH_REQ_RCPT_DEVICE )
#define USBH_REQ_SET_CONFIGURATION_BYTE \
	( USBH_REQ_DIR_HOST_TO_DEVICE | USBH_REQ_TYPE_STANDARD | USBH_REQ_RCPT_DEVICE )

// ---------------------------------------------------------------------------
// Standard request codes (USB 2.0 spec table 9-4).
// ---------------------------------------------------------------------------
#define USBH_REQ_GET_DESCRIPTOR        0x06u
#define USBH_REQ_SET_ADDRESS           0x05u
#define USBH_REQ_SET_CONFIGURATION     0x09u
#define USBH_REQ_CLEAR_FEATURE         0x01u
#define USBH_REQ_SET_INTERFACE         0x0Bu

// ---------------------------------------------------------------------------
// Descriptor types (USB 2.0 spec table 9-5).
// ---------------------------------------------------------------------------
#define USBH_DESC_DEVICE                0x01u
#define USBH_DESC_CONFIGURATION         0x02u
#define USBH_DESC_STRING                0x03u
#define USBH_DESC_INTERFACE             0x04u
#define USBH_DESC_ENDPOINT              0x05u
#define USBH_DESC_DEVICE_QUALIFIER      0x06u
#define USBH_DESC_OTHER_SPEED_CONFIG    0x07u
#define USBH_DESC_HID                   0x21u
#define USBH_DESC_HID_REPORT            0x22u
#define USBH_DESC_HUB                   0x29u

// ---------------------------------------------------------------------------
// Class codes (USB-IF assigned, used in interface descriptors).
// ---------------------------------------------------------------------------
#define USBH_CLASS_PER_INTERFACE   0x00u
#define USBH_CLASS_AUDIO           0x01u
#define USBH_CLASS_COMM            0x02u
#define USBH_CLASS_HID             0x03u
#define USBH_CLASS_PHYSICA         0x05u
#define USBH_CLASS_IMAGE           0x06u
#define USBH_CLASS_PRINTER         0x07u
#define USBH_CLASS_MSC             0x08u
#define USBH_CLASS_HUB             0x09u
#define USBH_CLASS_CDC_DATA        0x0Au
#define USBH_CLASS_VENDOR_SPECIFIC 0xFFu

// ---------------------------------------------------------------------------
// Error codes. USBH_ERR_SUCCESS == 0, other values come from the WCH EVT
// convention so the log lines look familiar.
// ---------------------------------------------------------------------------
enum
{
	USBH_ERR_SUCCESS         = 0x00,
	USBH_ERR_USB_CONNECT     = 0x15,
	USBH_ERR_USB_DISCON      = 0x16,
	USBH_ERR_USB_BUF_OVER    = 0x17,
	USBH_ERR_USB_TRANSFER    = 0x20,
	USBH_ERR_USB_UNKNOWN     = 0xFE,

	USBH_ERR_DEV_DESCR_FAIL  = 0x45,
	USBH_ERR_ADDR_SET_FAIL   = 0x46,
	USBH_ERR_CFG_DESCR_FAIL  = 0x47,

	USBH_ERR_USB_HID_NOBOOT  = 0x55,  // No boot-protocol HID interface found
	USBH_ERR_USB_HID_STALL   = 0x56,  // HID endpoint stalled
	USBH_ERR_USB_MSC_RESET   = 0x60,  // MSC class request failed
	USBH_ERR_USB_MSC_CBW     = 0x61,  // CBW send/receive failed
	USBH_ERR_USB_MSC_CSW     = 0x62,  // CSW indicated error
	USBH_ERR_USB_MSC_SENSE   = 0x63,  // REQUEST SENSE check failed
	USBH_ERR_USB_MSC_CAP     = 0x64,  // READ CAPACITY failed
};

// ---------------------------------------------------------------------------
// USB SETUP packet, 8 bytes, little-endian on the wire. Our chip is
// little-endian, so a plain struct copy works (the SIE reads it as raw
// bytes, not as a struct).
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} USBH_SetupReq;

#endif // USBHOST_DEFS_H
