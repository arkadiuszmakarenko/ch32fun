// usbhost_hid.c
//
// USB HID class driver for the ch32fun USB host stack.
//
// Walks the configuration descriptor looking for an interface that is
// either:
//   - a boot-protocol HID (class=3, subclass=1, protocol=1 keyboard
//     or 2 mouse) — we issue SET_PROTOCOL boot, the boot report has a
//     fixed layout, and the caller's report buffer is treated as a
//     USBH_HidKbdReport or USBH_HidMouseReport, OR
//   - a report-protocol HID (class=3, any other subclass/protocol),
//     or a vendor-specific Xbox-360 interface (class=0xFF, subclass=
//     0x5D, protocol=0x01) — we leave the device on report protocol
//     and parse its HID Report Descriptor to learn the layout of
//     axes, buttons, and wheel.
//
// The Xbox 360 wired controller is special-cased: it has no HID
// Report Descriptor and uses a fixed 20-byte input report with
// header 0x00 0x14. We detect it by VID/PID (Microsoft 0x045E/0x028E)
// or by the vendor interface descriptor, and the caller's buffer
// is filled with raw bytes for the Xbox 360 decoder to consume.

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_hw.h"
#include "usbhost_xfer.h"
#include "usbhost_std.h"
#include "usbhost_defs.h"
#include "usbhost_hid.h"

// Class / subclass / protocol values we care about.
#define USBH_HID_CLASS             0x03u
#define USBH_HID_SUBCLASS_BOOTIF   0x01u
#define USBH_HID_PROTOCOL_KBD      0x01u
#define USBH_HID_PROTOCOL_MOUSE    0x02u

// Xbox 360 wired controller — vendor-specific interface.
#define USBH_X360_ITF_CLASS        0xFFu
#define USBH_X360_ITF_SUBCLASS     0x5Du
#define USBH_X360_ITF_PROTOCOL     0x01u
#define USBH_X360_VID              0x045Eu
#define USBH_X360_PID              0x028Eu

// Class requests.
#define USBH_HID_REQ_GET_REPORT    0x01u
#define USBH_HID_REQ_SET_IDLE      0x0Au
#define USBH_HID_REQ_SET_PROTOCOL  0x0Bu
#define USBH_HID_PROTOCOL_BOOT     0x00u
#define USBH_HID_PROTOCOL_REPORT   0x01u

// Endpoint types.
#define USBH_HID_EP_TYPE_INTR      0x03u

// Sizes.
#define USBH_HID_REP_DESC_MAX      512u   // Twin USB / many pads are 250-400 bytes
#define USBH_HID_REP_LEN_BOOT_KBD  8u
#define USBH_HID_REP_LEN_BOOT_MSE  3u
#define USBH_HID_REP_LEN_XBOX360   20u


// ---------------------------------------------------------------------------
// USBH_Put16_inline
// ---------------------------------------------------------------------------
static inline void USBH_Put16_inline( uint8_t *p, uint16_t v )
{
	p[0] = (uint8_t)( v       & 0xFFu );
	p[1] = (uint8_t)( (v >> 8) & 0xFFu );
}


// ---------------------------------------------------------------------------
// Class-specific request helper (SET_IDLE / SET_PROTOCOL).
// ---------------------------------------------------------------------------
static uint8_t USBH_HidClassReq( uint8_t ep0_size, uint8_t iface,
	uint8_t request, uint16_t wvalue )
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


// ---------------------------------------------------------------------------
// Scan the config descriptor for the first matching interface. Returns
// the interface number via *piface, the interrupt-IN endpoint
// details via *pin_ep/*pin_maxp/*pin_interval, the *report length*
// (used only for boot-protocol devices) via *prep_len, and the
// *HID Report Descriptor length* (from the class-specific HID
// descriptor) via *prep_desc_len.
//
// `*pkbd_proto` is filled with 1 for keyboard, 2 for mouse, 0 for
// other (gamepad, multi-button mouse, etc), 0xFF for Xbox 360.
//
// Boot-protocol devices: bInterfaceSubClass=1, bInterfaceProtocol=1 or 2.
// Xbox 360:             bInterfaceClass=0xFF, bInterfaceSubClass=0x5D,
//                       bInterfaceProtocol=0x01.
// Other HID:            bInterfaceClass=3.
// ---------------------------------------------------------------------------
static uint8_t USBH_FindHid( const uint8_t *cfg, uint16_t total,
	const uint8_t *dev_desc,
	uint8_t *piface,
	uint8_t *pkbd_proto,
	uint8_t *pin_ep,
	uint16_t *pin_maxp,
	uint8_t *pin_interval,
	uint8_t *prep_len,
	uint16_t *prep_desc_len )
{
	uint16_t off = 0;
	uint8_t  match_iface = 0xFF;
	uint8_t  match_proto = 0xFFu;
	uint8_t  match_ep    = 0;
	uint16_t match_maxp  = 0;
	uint8_t  match_intv  = 0;
	uint8_t  match_rlen  = 0;
	uint16_t match_dlen  = 0;

	// For Xbox 360 detection we need the device VID/PID; the device
	// descriptor is 18 bytes and the IDs are at bytes 8..11.
	uint16_t vid = 0, pid = 0;
	if( dev_desc )
	{
		vid = (uint16_t)dev_desc[8]  | ( (uint16_t)dev_desc[9]  << 8 );
		pid = (uint16_t)dev_desc[10] | ( (uint16_t)dev_desc[11] << 8 );
	}

	while( off + 2 <= total )
	{
		uint8_t len  = cfg[off];
		uint8_t type = cfg[off + 1];
		if( len == 0 ) break;
		if( off + len > total ) break;

		if( type == USBH_DESC_INTERFACE && len >= 9 )
		{
			uint8_t cls   = cfg[off + 5];
			uint8_t sub   = cfg[off + 6];
			uint8_t proto = cfg[off + 7];

			// First match wins. We prefer boot-protocol devices if
			// both are present, since they're the most common.
			if( match_iface == 0xFF )
			{
				if( cls == USBH_HID_CLASS &&
				    sub == USBH_HID_SUBCLASS_BOOTIF &&
				    ( proto == USBH_HID_PROTOCOL_KBD ||
				      proto == USBH_HID_PROTOCOL_MOUSE ) )
				{
					match_iface = cfg[off + 2];
					match_proto = proto;
					match_rlen  = ( proto == USBH_HID_PROTOCOL_KBD )
					             ? USBH_HID_REP_LEN_BOOT_KBD
					             : USBH_HID_REP_LEN_BOOT_MSE;
				}
				else if( ( vid == USBH_X360_VID && pid == USBH_X360_PID ) ||
				         ( cls == USBH_X360_ITF_CLASS &&
				           sub == USBH_X360_ITF_SUBCLASS &&
				           proto == USBH_X360_ITF_PROTOCOL ) )
				{
					// Xbox 360 — the actual IN endpoint will be
					// picked up below; it lives in the same
					// interface descriptor's EP list.
					match_iface = cfg[off + 2];
					match_proto = 0xFEu;  // sentinel for X360
					match_rlen  = USBH_HID_REP_LEN_XBOX360;
				}
				else if( cls == USBH_HID_CLASS )
				{
					// Generic HID (gamepad, multi-button mouse,
					// consumer control, etc). We accept the
					// first one — the parser then decides if it
					// has axes / buttons / wheel.
					match_iface = cfg[off + 2];
					match_proto = 0x00u;  // report-protocol HID
					match_rlen  = 0;       // filled in by parser
				}
			}
		}
		else if( type == USBH_DESC_ENDPOINT && len >= 7 &&
		         match_iface != 0xFF )
		{
			// Endpoints follow the interface they belong to, until
			// the next interface descriptor. We only collect EPs
			// while we're still inside the matched interface.
			// Simpler approach: collect every interrupt-IN EP and
			// keep the first one (we match on iface afterwards).
			uint8_t eaddr = cfg[off + 2];
			uint8_t eattr = cfg[off + 3];
			uint16_t emaxp = (uint16_t)cfg[off + 4] | ((uint16_t)cfg[off + 5] << 8);
			uint8_t eintv = cfg[off + 6];
			if( ( eaddr & 0x80 ) &&                 // IN
			    ( eattr & 0x03 ) == USBH_HID_EP_TYPE_INTR &&
			    match_ep == 0 )
			{
				match_ep   = eaddr & 0x0Fu;
				match_maxp = emaxp;
				match_intv = eintv;
			}
		}
		else if( type == USBH_DESC_HID && len >= 6 && match_iface != 0xFF )
		{
			// The HID class descriptor's wReportDescriptorLength is
			// at offsets 4..5 of the descriptor (after bcdHID,
			// bCountryCode, bNumDescriptors).
			match_dlen = (uint16_t)cfg[off + 4] | ((uint16_t)cfg[off + 5] << 8);
		}

		off += len;
	}

	if( match_iface == 0xFF || match_ep == 0 ) {
		return USBH_ERR_USB_HID_NOBOOT;
	}

	*piface        = match_iface;
	*pkbd_proto    = match_proto;
	*pin_ep        = match_ep;
	*pin_maxp      = match_maxp;
	*pin_interval  = match_intv;
	*prep_len      = match_rlen;
	*prep_desc_len = match_dlen;
	printf( "HID: find iface=%u proto=%02x ep=0x%02x maxp=%u dlen=%u rlen=%u\n",
		match_iface, match_proto, match_ep, match_maxp, match_dlen, match_rlen );
	return USBH_ERR_SUCCESS;
}


// ---------------------------------------------------------------------------
// USBH_HidInit
//
// Bring up the first matching HID interface. See the header for the
// return-value contract.
// ---------------------------------------------------------------------------
uint8_t USBH_HidInit( const uint8_t *dev_desc,
                      const uint8_t *cfg_desc, uint16_t cfg_len,
                      uint8_t *pkind,
                      uint8_t *prep_len,
                      uint8_t *pin_ep,
                      uint16_t *pin_maxp,
                      uint8_t *pin_interval,
                      hid_report_t *pdesc )
{
	uint8_t  s;
	uint8_t  iface = 0;
	uint8_t  proto = 0xFFu;
	uint8_t  ep = 0;
	uint16_t maxp = 0;
	uint8_t  interval = 0;
	uint8_t  rlen = 0;
	uint16_t dlen = 0;
	uint8_t  kind = USBH_HID_KIND_NONE;
	uint8_t  rep_desc[ USBH_HID_REP_DESC_MAX ];

	if( pdesc ) memset( pdesc, 0, sizeof( *pdesc ) );

	// Use the device's actual bMaxPacketSize0 (from byte 7 of the
	// device descriptor) for control transfers — many low-speed
	// mice only have 8 bytes of EP0 FIFO, and using a larger value
	// will make USBH_CtrlXfer's short-packet check misfire after
	// the first 8-byte chunk. Default to 8 (the minimum) if the
	// descriptor is missing or zero.
	uint8_t ep0_size = 8;
	if( dev_desc && dev_desc[0] >= 8 )
	{
		ep0_size = dev_desc[7];
		if( ep0_size == 0 ) ep0_size = 8;
	}

	s = USBH_FindHid( cfg_desc, cfg_len, dev_desc,
		&iface, &proto, &ep, &maxp, &interval, &rlen, &dlen );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "HID: scan failed\n" );
		return s;
	}
	printf( "HID: ep0_size=%u (from bMaxPacketSize0)\n", ep0_size );

	if( proto == 0xFEu )
	{
		// Xbox 360 wired controller. No class setup needed; just
		// remember the layout and hand back a fixed-size buffer.
		kind = USBH_HID_KIND_XBOX360;
		rlen = USBH_HID_REP_LEN_XBOX360;
		printf( "HID: Xbox 360 iface #%u, ep=0x%02x maxp=%u interval=%u\n",
			iface, ep, maxp, interval );
	}
	else
	{
		// SET_IDLE 0 — wake on every report (and the device can
		// stay silent when nothing's happening).
		s = USBH_HidClassReq( ep0_size, iface,
			USBH_HID_REQ_SET_IDLE, 0x0000u );
		if( s != USBH_ERR_SUCCESS )
		{
			printf( "HID: SET_IDLE failed %02x\n", s );
			return s;
		}

		if( proto == USBH_HID_PROTOCOL_KBD || proto == USBH_HID_PROTOCOL_MOUSE )
		{
			// Boot-capable interface. We prefer *report* protocol
			// even for these — many mice that advertise boot
			// subclass still expose a wheel / extra buttons in
			// their HID Report Descriptor, but boot protocol
			// itself is a fixed 3-byte report (buttons/X/Y) with
			// no room for the wheel. So:
			//
			//   1. Stay on report protocol (the default after
			//      SET_CONFIGURATION). SET_PROTOCOL is a no-op
			//      until we explicitly issue it.
			//   2. Fetch the HID Report Descriptor and parse it.
			//   3. If we get a usable mouse / keyboard layout
			//      (with or without a wheel), use report protocol.
			//   4. Only fall back to boot protocol if the device
			//      has no HID Report Descriptor at all (very old
			//      mice, or the GET_DESCRIPTOR request STALLs).
			kind = ( proto == USBH_HID_PROTOCOL_KBD )
			     ? USBH_HID_KIND_KEYBOARD
			     : USBH_HID_KIND_MOUSE;

			printf( "HID: boot-cap iface=%u, wReportDescriptorLength=%u\n",
				iface, dlen );

			int used_report_proto = 0;
			if( dlen > 0 && dlen <= USBH_HID_REP_DESC_MAX )
			{
				uint16_t want = dlen;
				s = USBH_GetHidReportDesc( ep0_size, iface,
					rep_desc, &want );
				printf( "HID: GET_REPORT_DESC rc=%02x got=%u\n",
					s, want );
				if( s == USBH_ERR_SUCCESS && want > 0 )
				{
					// Hex-dump the first 64 bytes of the
					// report descriptor. With 32-byte
					// chunks, the first dump is enough to
					// see the report ID, usages, etc.
					printf( "HID: report desc:" );
					for( uint16_t k = 0; k < want && k < 96; k++ )
					{
						if( ( k & 15 ) == 0 ) printf( "\n  %04x:", k );
						printf( " %02x", rep_desc[ k ] );
					}
					printf( "\n" );

					if( pdesc )
					{
						int pr = parse_report_descriptor( rep_desc, want, pdesc );
						if( !pr )
						{
							uint16_t ooff = 0;
							uint16_t oerr = usbhost_reportparser_last_err( &ooff );
							printf( "HID: parser FAILED err=%04x off=%u type=%u rlen=%u btns=%u\n",
								oerr, ooff, pdesc->type, pdesc->report_size,
								pdesc->button_count );
						}
						else
						{
							printf( "HID: parser rc=%d type=%u rlen=%u btns=%u "
								"rid=%u X(%u/%u) Y(%u/%u) W(%u/%u) hat=(%u/%u)\n",
								pr, pdesc->type, pdesc->report_size,
								pdesc->button_count, pdesc->report_id,
								pdesc->axis[0].size, pdesc->axis[0].logical.min,
								pdesc->axis[1].size, pdesc->axis[1].logical.min,
								pdesc->wheel.size,  pdesc->wheel.logical.min,
								pdesc->hat.size,    pdesc->hat.logical.min );
							// Dump per-button offset/bitmask so we
							// can verify the layout the decoder will
							// see. 8 buttons fit on one line.
							printf( "HID: btn layout:" );
							for( uint8_t k = 0; k < 12; k++ )
							{
								if( k >= pdesc->button_count ) break;
								printf( " [%u]=%u/%02x",
									k, pdesc->button[k].byte_offset,
									pdesc->button[k].bitmask );
							}
							printf( "\n" );
						}
						if( pr )
						{
							used_report_proto = 1;
							if( pdesc->type == USBH_REPORT_TYPE_MOUSE )
							{
								kind = USBH_HID_KIND_MOUSE;
								rlen = pdesc->report_size;
							}
							else if( pdesc->type == USBH_REPORT_TYPE_JOYSTICK )
							{
								kind = USBH_HID_KIND_GAMEPAD;
								rlen = pdesc->report_size;
							}
							else if( pdesc->type == USBH_REPORT_TYPE_KEYBOARD )
							{
								kind = USBH_HID_KIND_KEYBOARD;
								rlen = pdesc->report_size;
							}
							else
							{
								// Parser succeeded but didn't
								// recognise a usage we know
								// about. Fall through to boot.
								used_report_proto = 0;
							}
						}
					}
				}
			}
			else
			{
				printf( "HID: no HID class descriptor in cfg (dlen=%u)\n", dlen );
			}

			if( !used_report_proto )
			{
				// Either no report descriptor at all, or the
				// parser bailed. Issue SET_PROTOCOL boot so the
				// device sends the standard 3- or 8-byte report.
				s = USBH_HidClassReq( ep0_size, iface,
					USBH_HID_REQ_SET_PROTOCOL,
					USBH_HID_PROTOCOL_BOOT );
				if( s != USBH_ERR_SUCCESS )
				{
					printf( "HID: SET_PROTOCOL boot failed %02x\n", s );
					return s;
				}
				printf( "HID: boot %s iface #%u, ep=0x%02x maxp=%u interval=%u report=%u\n",
					kind == USBH_HID_KIND_KEYBOARD ? "kbd" : "mouse",
					iface, ep, maxp, interval, rlen );
			}
			else
			{
				printf( "HID: report-proto %s iface #%u, ep=0x%02x maxp=%u interval=%u report=%u\n",
					kind == USBH_HID_KIND_MOUSE    ? "mouse"  :
					kind == USBH_HID_KIND_GAMEPAD  ? "gamepad":
					kind == USBH_HID_KIND_KEYBOARD ? "kbd"    : "?",
					iface, ep, maxp, interval, rlen );
			}
		}
		else
		{
			// Report-protocol HID. Fetch and parse the Report
			// Descriptor. The parser tells us what kind of device
			// it is (mouse with wheel, joystick, gamepad, ...).
			kind = USBH_HID_KIND_GAMEPAD;  // optimistic default;
			                                // overridden below if the
			                                // parser finds a mouse.
			if( dlen == 0 || dlen > USBH_HID_REP_DESC_MAX )
			{
				printf( "HID: bad report desc len %u\n", dlen );
				return USBH_ERR_USB_HID_NOBOOT;
			}
			{
				uint16_t want = dlen;
				s = USBH_GetHidReportDesc( ep0_size, iface,
					rep_desc, &want );
				if( s != USBH_ERR_SUCCESS || want == 0 )
				{
					printf( "HID: GET_REPORT_DESC failed %02x\n", s );
					return s;
				}
				if( pdesc &&
				    parse_report_descriptor( rep_desc, want, pdesc ) )
				{
					if( pdesc->type == USBH_REPORT_TYPE_MOUSE )
						kind = USBH_HID_KIND_MOUSE;
					else if( pdesc->type == USBH_REPORT_TYPE_JOYSTICK )
						kind = USBH_HID_KIND_GAMEPAD;
					else if( pdesc->type == USBH_REPORT_TYPE_KEYBOARD )
						kind = USBH_HID_KIND_KEYBOARD;
					// Otherwise leave the default.
					rlen = pdesc->report_size;
				}
				else
				{
					printf( "HID: report desc unparseable, falling back\n" );
				}
			}
			printf( "HID: report-proto %s iface #%u, ep=0x%02x maxp=%u interval=%u report=%u\n",
				kind == USBH_HID_KIND_MOUSE    ? "mouse"  :
				kind == USBH_HID_KIND_GAMEPAD  ? "gamepad":
				kind == USBH_HID_KIND_KEYBOARD ? "kbd"    : "?",
				iface, ep, maxp, interval, rlen );
		}
	}

	*pkind        = kind;
	*prep_len     = rlen;
	*pin_ep       = ep;
	*pin_maxp     = maxp;
	*pin_interval = interval;
	return USBH_ERR_SUCCESS;
}


// ---------------------------------------------------------------------------
// USBH_HidReadReport
//
// One interrupt-IN transfer. Caller passes a buffer that's at least
// `max_len` bytes long; we fill it with up to `max_len` bytes (the
// SIE's full-speed IN FIFO is 64 bytes) and write the actual byte
// count to *pnread. Toggle bookkeeping matches usbhost_xfer.c:
// the SIE has auto-toggle OFF for IN, so the caller flips *ptog
// after a successful TOG_OK reply.
// ---------------------------------------------------------------------------
uint8_t USBH_HidReadReport( uint8_t ep, uint8_t *ptog, uint16_t maxp,
                            void *report, uint8_t max_len,
                            uint8_t *pnread )
{
	uint8_t  rx_len = 0;
	uint8_t  s;

	(void)maxp;

	if( pnread ) *pnread = 0;
	if( max_len == 0 || report == NULL ) return USBH_ERR_USB_UNKNOWN;

	s = USBH_BulkOrIntrIn( ep, *ptog, (uint8_t*)report, &rx_len, 1000 );
	if( s != USBH_ERR_SUCCESS ) return s;

	if( rx_len > max_len ) rx_len = max_len;
	if( pnread ) *pnread = rx_len;
	*ptog ^= 1u;
	return USBH_ERR_SUCCESS;
}


// ---------------------------------------------------------------------------
// USBH_HidDecodeGeneric
//
// Read a raw HID report through the parsed descriptor and produce
// a USBH_HidInputReport. Handles signed axes (a logical_minimum >
// logical_maximum in the descriptor is HID's way of saying "this
// field is signed"; we honor that) and multi-bit buttons.
//
// All axis values are returned *raw* — the application is expected
// to apply centring and dead-zones using
// pdesc->axis[c].logical.{min,max}.
//
// `kind` may be USBH_HID_KIND_MOUSE or USBH_HID_KIND_GAMEPAD. The
// Xbox 360 path uses a separate function.
// ---------------------------------------------------------------------------
void USBH_HidDecodeGeneric( const hid_report_t *pdesc, uint8_t kind,
                            const uint8_t *raw, uint8_t raw_len,
                            USBH_HidInputReport *out )
{
	if( !out ) return;
	memset( out, 0, sizeof( *out ) );
	if( !pdesc || !raw || raw_len == 0 ) return;

	// Skip the Report ID byte if the descriptor has one. Many
	// devices omit it; some don't.
	uint8_t off = ( pdesc->report_id ) ? 1 : 0;
	if( off >= raw_len ) return;
	const uint8_t *p = raw + off;
	uint8_t plen = (uint8_t)( raw_len - off );

	(void)kind;

	// ---- Axes (up to 4) ----
	// Read each axis as a raw field. HID sign convention:
	// logical_minimum > logical_maximum means "signed" (some
	// devices use this; we follow it).
	for( uint8_t c = 0; c < 4; c++ )
	{
		if( pdesc->axis[c].size == 0 ) continue;
		int is_signed = ( pdesc->axis[c].logical.min >
		                  pdesc->axis[c].logical.max );
		uint16_t v = collect_bits( p, pdesc->axis[c].offset,
		                           pdesc->axis[c].size, is_signed );
		out->axis[c] = (int16_t)v;
	}
	(void)plen;

	// ---- Wheel (mice only) ----
	if( pdesc->wheel.size > 0 )
	{
		int is_signed = ( pdesc->wheel.logical.min >
		                  pdesc->wheel.logical.max );
		uint16_t v = collect_bits( p, pdesc->wheel.offset,
		                           pdesc->wheel.size, is_signed );
		out->wheel = (int16_t)v;
	}

	// ---- Buttons (up to 12 bits) ----
	// The parser stores byte_offset and bitmask for each button
	// directly in the descriptor. We index into `p` (which has
	// the report ID byte already stripped) using the same offset;
	// because the parser computed offsets from the start of the
	// data (after the report ID), this is consistent.
	for( uint8_t b = 0; b < USBH_REPORT_BUTTONS; b++ )
	{
		if( b >= pdesc->button_count ) break;
		if( pdesc->button[b].bitmask == 0 ) continue;  // not present
		if( pdesc->button[b].byte_offset >= plen ) continue;
		if( p[ pdesc->button[b].byte_offset ] & pdesc->button[b].bitmask )
		{
			if( b < 8 ) out->buttons       |= (uint8_t)( 1u << b );
			else        out->buttons_extra |= (uint8_t)( 1u << ( b - 8 ) );
		}
	}
}


// ---------------------------------------------------------------------------
// USBH_HidDecodeX360
//
// Xbox 360 wired-controller 20-byte report. Layout (Microsoft public
// protocol): 0x00 0x14 BTN_LO BTN_HI LT RT LX(LE16) LY(LE16) RX(LE16)
// RY(LE16) ... (remainder unused here).
// ---------------------------------------------------------------------------
int USBH_HidDecodeX360( const uint8_t *raw, uint8_t raw_len,
                        USBH_HidX360Report *out )
{
	if( !out ) return -1;
	memset( out, 0, sizeof( *out ) );
	if( !raw || raw_len < 20 ) return -1;
	if( raw[0] != 0x00u || raw[1] != 0x14u ) return -1;

	out->buttons_low  = raw[2];
	out->buttons_high = raw[3];
	out->lt           = raw[4];
	out->rt           = raw[5];
	out->lx = (int16_t)( (uint16_t)raw[6]  | ( (uint16_t)raw[7]  << 8 ) );
	out->ly = (int16_t)( (uint16_t)raw[8]  | ( (uint16_t)raw[9]  << 8 ) );
	out->rx = (int16_t)( (uint16_t)raw[10] | ( (uint16_t)raw[11] << 8 ) );
	out->ry = (int16_t)( (uint16_t)raw[12] | ( (uint16_t)raw[13] << 8 ) );
	return 0;
}


// ---------------------------------------------------------------------------
// USBH_HidKeyToAscii
// ---------------------------------------------------------------------------
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
		case 0x28: return shift ? '<' : '\n';
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
