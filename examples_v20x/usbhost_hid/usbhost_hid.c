// usbhost_hid.c -- minimal boot-protocol HID driver.
//
// Walks the configuration descriptor to find the first HID interface,
// claims it (issues SET_IDLE 0 + SET_PROTOCOL boot), then exposes a
// blocking report-read function. Does NOT parse the HID Report
// Descriptor (the boot protocol has a fixed layout per subclass).

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_hw.h"
#include "usbhost_xfer.h"
#include "usbhost_std.h"
#include "usbhost_defs.h"
#include "usbhost_hid.h"

// Class / subclass / protocol values we care about.
#define USBH_HID_CLASS             0x03
#define USBH_HID_SUBCLASS_BOOTIF   0x01
#define USBH_HID_PROTOCOL_KBD      0x01
#define USBH_HID_PROTOCOL_MOUSE    0x02

#define USBH_HID_REQ_SET_IDLE      0x0Au
#define USBH_HID_REQ_SET_PROTOCOL  0x0Bu
#define USBH_HID_PROTOCOL_BOOT     0x00u
#define USBH_HID_PROTOCOL_REPORT   0x01u

#define USBH_HID_EP_TYPE_INTR      0x03

// Walk a configuration descriptor looking for the first interface with
// bInterfaceClass=03 (HID), bInterfaceSubClass=01 (boot interface),
// and bInterfaceProtocol 1 (keyboard) or 2 (mouse). If found, copies
// the interface record and its interrupt-IN endpoint back.
//
// `ep0_size` is passed through to USBH_CtrlXfer for SET_IDLE/PROTOCOL.
static uint8_t USBH_FindBootHid( const uint8_t *cfg, uint16_t total,
	uint8_t ep0_size,
	uint8_t *pin_ep, uint16_t *pin_maxp,
	uint8_t *pin_interval, uint8_t *prep_len )
{
	uint16_t off = 0;
	while( off + 2 <= total )
	{
		uint8_t len  = cfg[off];
		uint8_t type = cfg[off + 1];
		if( len == 0 ) break;
		if( off + len > total ) break;

		if( type == USBH_DESC_INTERFACE && len >= 9 )
		{
			uint8_t cls = cfg[off + 5];
			uint8_t sub = cfg[off + 6];
			uint8_t proto = cfg[off + 7];
			uint8_t n_ep = cfg[off + 4];

			if( cls == USBH_HID_CLASS &&
			    sub == USBH_HID_SUBCLASS_BOOTIF &&
			    ( proto == USBH_HID_PROTOCOL_KBD ||
			      proto == USBH_HID_PROTOCOL_MOUSE ) )
			{
				*prep_len = ( proto == USBH_HID_PROTOCOL_KBD ) ? 8 : 3;

				// The endpoint descriptors follow the interface
				// descriptor (and any class-specific descriptors
				// like HID Report Descriptor). We scan forward
				// until we run out of bytes or hit the next
				// interface.
				uint16_t ep_off = off + len;
				uint16_t end = off + len;
				while( end + 2 <= total && cfg[end + 1] != USBH_DESC_INTERFACE )
				{
					uint8_t elen = cfg[end];
					if( elen == 0 ) break;
					end += elen;
				}
				(void)ep_off;

				for( uint16_t e = off + len; e < end; e += cfg[e] )
				{
					if( cfg[e] < 7 ) continue;
					if( cfg[e + 1] != USBH_DESC_ENDPOINT ) continue;
					uint8_t eaddr = cfg[e + 2];
					uint8_t eattr = cfg[e + 3];
					uint16_t emaxp = (uint16_t)cfg[e + 4] | ((uint16_t)cfg[e + 5] << 8);
					uint8_t eintv = cfg[e + 6];
					if( ( eaddr & 0x80 ) &&  // IN
					    ( eattr & 0x03 ) == USBH_HID_EP_TYPE_INTR )
					{
						*pin_ep = eaddr & 0x0Fu;
						*pin_maxp = emaxp;
						*pin_interval = eintv;
						return USBH_ERR_SUCCESS;
					}
				}

				printf( "HID: boot iface found (proto=%u) but no INTR-IN endpoint\n",
					proto );
				return USBH_ERR_USB_UNKNOWN;
			}
			(void)n_ep;
		}

		off += len;
	}

	return USBH_ERR_USB_UNKNOWN;
}

// SET_IDLE / SET_PROTOCOL: class-specific requests on EP0, recipient
// is the *interface*. The wValue carries protocol (0=boot) and the
// idle rate (0=infinite - wake us on every report, which is what
// we want for boot protocol polling).
static inline void USBH_Put16_inline( uint8_t *p, uint16_t v )
{
	p[0] = (uint8_t)( v & 0xFFu );
	p[1] = (uint8_t)( (v >> 8) & 0xFFu );
}

static uint8_t USBH_HidClassReq( uint8_t ep0_size, uint8_t iface,
	uint8_t request, uint8_t wvalue )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	req.bmRequestType = 0x21u;  // host-to-device, class, interface
	req.bRequest = request;
	USBH_Put16_inline( (uint8_t*)&req.wValue, wvalue );
	USBH_Put16_inline( (uint8_t*)&req.wIndex, iface );
	USBH_Put16_inline( (uint8_t*)&req.wLength, 0 );

	s = USBH_CtrlXfer( &req, NULL, 0, ep0_size, &got );
	return s;
}

uint8_t USBH_HidInit( const uint8_t *cfg_desc, uint16_t cfg_len,
	uint8_t *prep_len,
	uint8_t *pin_ep,
	uint16_t *pin_maxp,
	uint8_t *pin_interval )
{
	uint8_t s;
	uint8_t iface_num = 0xFF;
	uint8_t ep = 0;
	uint16_t maxp = 0;
	uint8_t interval = 0;
	uint8_t rlen = 0;

	// Re-derive the interface number by scanning the config desc.
	{
		uint16_t off = 0;
		while( off + 2 <= cfg_len )
		{
			uint8_t len = cfg_desc[off];
			uint8_t type = cfg_desc[off + 1];
			if( len == 0 ) break;
			if( off + len > cfg_len ) break;
			if( type == USBH_DESC_INTERFACE && len >= 9 )
			{
				uint8_t cls = cfg_desc[off + 5];
				uint8_t sub = cfg_desc[off + 6];
				uint8_t proto = cfg_desc[off + 7];
				if( cls == USBH_HID_CLASS &&
				    sub == USBH_HID_SUBCLASS_BOOTIF &&
				    ( proto == USBH_HID_PROTOCOL_KBD ||
				      proto == USBH_HID_PROTOCOL_MOUSE ) )
				{
					iface_num = cfg_desc[off + 2];
					break;
				}
			}
			off += len;
		}
	}
	if( iface_num == 0xFF )
	{
		printf( "HID: no boot-protocol interface\n" );
		return USBH_ERR_USB_HID_NOBOOT;
	}

	s = USBH_FindBootHid( cfg_desc, cfg_len, 64,
		&ep, &maxp, &interval, &rlen );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "HID: scan failed\n" );
		return s;
	}

	// SET_IDLE 0 (infinite - wake on any change).
	s = USBH_HidClassReq( 64, iface_num,
		USBH_HID_REQ_SET_IDLE, 0x0000u );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "HID: SET_IDLE failed %02x\n", s );
		return s;
	}

	// SET_PROTOCOL boot (0).
	s = USBH_HidClassReq( 64, iface_num,
		USBH_HID_REQ_SET_PROTOCOL,
		USBH_HID_PROTOCOL_BOOT );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "HID: SET_PROTOCOL failed %02x\n", s );
		return s;
	}

	*prep_len = rlen;
	*pin_ep = ep;
	*pin_maxp = maxp;
	*pin_interval = interval;
	printf( "HID: boot interface #%u, ep=0x%02x maxp=%u interval=%u report=%u\n",
		iface_num, ep, maxp, interval, rlen );
	return USBH_ERR_SUCCESS;
}

// Block on the interrupt-IN endpoint until a report is available, or
// return on detach.  We don't strictly need a per-endpoint toggle for
// boot protocol (most keyboards use 8-byte reports and always start
// with DATA0 after a successful report), but we keep the field so the
// caller can maintain one.
uint8_t USBH_HidReadReport( uint8_t ep, uint8_t *ptog, uint16_t maxp,
	void *report, uint8_t report_size )
{
	uint8_t rx_len = 0;
	uint8_t s;

	(void)maxp;  // unused - SIE clamps to 64

	s = USBH_BulkOrIntrIn( ep, *ptog, (uint8_t*)report, &rx_len, 1000 );
	if( s != USBH_ERR_SUCCESS ) return s;

	if( rx_len < report_size )
	{
		// Short report: zero-pad the rest. (Mice often send 3
		// bytes; some keyboards send 6. Either way, the unread
		// bytes are stale and shouldn't be acted on.)
		memset( (uint8_t*)report + rx_len, 0, report_size - rx_len );
	}

	*ptog ^= 1u;  // host flips expected toggle after TOG_OK
	return USBH_ERR_SUCCESS;
}

// HID usage -> ASCII. We cover the printable subset: a-z, 0-9,
// common punctuation, Enter, Escape, Backspace, Tab. Unhandled keys
// return 0. The caller checks the modifier byte for shift and
// inverts case / picks the shifted glyph.
//
// Usage IDs come from the USB HID Usage Tables spec, "Keyboard/
// Keypad Page (0x07)".
uint8_t USBH_HidKeyToAscii( uint8_t usage, uint8_t shift )
{
	if( usage >= 0x04 && usage <= 0x1D )
	{
		// a-z
		uint8_t base = (uint8_t)( 'a' + ( usage - 0x04 ) );
		return shift ? (uint8_t)( base - 'a' + 'A' ) : base;
	}
	if( usage >= 0x1E && usage <= 0x27 )
	{
		// 1-0
		static const char top[10] = "1234567890";
		return (uint8_t)top[ usage - 0x1E ];
	}
	switch( usage )
	{
		case 0x28: return shift ? '<' : '\n';   // Return; shifted = < on US
		case 0x29: return 0;                    // Escape
		case 0x2A: return shift ? '~' : '`';
		case 0x2B: return shift ? '_' : '-';
		case 0x2C: return shift ? '+' : '=';
		case 0x2D: return shift ? '{' : '[';
		case 0x2E: return shift ? '}' : ']';
		case 0x2F: return shift ? '|' : '\\';
		case 0x30: return shift ? '!' : '1';
		case 0x31: return shift ? '@' : '2';
		case 0x32: return shift ? '#' : '3';
		case 0x33: return shift ? '$' : '4';
		case 0x34: return shift ? '%' : '5';
		case 0x35: return shift ? '^' : '6';
		case 0x36: return shift ? '&' : '7';
		case 0x37: return shift ? '*' : '8';
		case 0x38: return shift ? '(' : '9';
		case 0x39: return shift ? ')' : '0';
		case 0x4C: return 0;                    // Delete
		case 0x4F: return 0;                    // Right
		case 0x50: return 0;                    // Left
		case 0x51: return 0;                    // Down
		case 0x52: return 0;                    // Up
	}
	return 0;
}
