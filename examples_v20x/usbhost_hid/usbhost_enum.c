// usbhost_enum.c -- project-local main for usbhost_hid.
//
// Carries the same enumeration state machine as the usbhost_enum
// reference, but at the end of a successful enumeration we call
// USBH_HidInit() and then loop on USBH_HidReadReport(), printing
// decoded keyboard characters, mouse deltas + scroll + extra buttons,
// joystick axes, and Xbox 360 button states.

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_hw.h"
#include "usbhost_xfer.h"
#include "usbhost_std.h"
#include "usbhost_defs.h"
#include "usbhost_enum.h"
#include "usbhost_hid.h"

// Standard address assigned to the device after SET_ADDRESS.
#define USBH_DEVICE_ADDR   0x02u

// Buffers for parsed descriptors.
static uint8_t DevDesc[ 18 ];
static uint8_t CfgDesc[ USBH_CFG_DESC_BUFFER_SIZE ];

// HID endpoint state. Stays valid until detach; main() resets it.
static uint8_t  g_hid_ep;
static uint16_t g_hid_maxp;
static uint8_t  g_hid_interval;
static uint8_t  g_hid_kind;        // one of USBH_HID_KIND_*
static uint8_t  g_hid_rep_len;
static uint8_t  g_hid_tog;
static hid_report_t g_hid_desc;   // parsed HID report descriptor

// Pretty-printers (subset of the reference project's).
static const char *USBH_ClassName( uint8_t class_code )
{
	switch( class_code )
	{
		case USBH_CLASS_HID:     return "HID";
		case USBH_CLASS_MSC:     return "Mass Storage";
		case USBH_CLASS_HUB:     return "Hub";
		case USBH_CLASS_VENDOR_SPECIFIC: return "Vendor-specific";
		case USBH_CLASS_PER_INTERFACE:    return "(per-iface)";
		default: return "?";
	}
}

static void USBH_PrintDeviceDesc( const uint8_t *p )
{
	uint16_t vid = (uint16_t)p[8]  | ( (uint16_t)p[9]  << 8 );
	uint16_t pid = (uint16_t)p[10] | ( (uint16_t)p[11] << 8 );
	uint16_t bcd = (uint16_t)p[2]  | ( (uint16_t)p[3]  << 8 );
	printf( "  VID=%04x PID=%04x bcdUSB=%04x Class=%02x(%s) SubClass=%02x Proto=%02x ep0=%u\n",
		vid, pid, bcd, p[4], USBH_ClassName( p[4] ), p[5], p[6], p[7] );
}

static void USBH_PrintConfigDesc( const uint8_t *p, uint16_t total )
{
	printf( "  config: %u byte(s), %u interface(s)\n", total, p[4] );
	uint16_t off = 0;
	while( off + 2 <= total )
	{
		uint8_t len = p[off];
		uint8_t type = p[off + 1];
		if( len == 0 ) break;
		if( off + len > total ) break;

		if( type == USBH_DESC_INTERFACE && len >= 9 )
		{
			printf( "    iface #%u  alt=%u  Class=%02x(%s) SubClass=%02x Proto=%02x  %u endpoint(s)\n",
				p[off + 2], p[off + 3],
				p[off + 5], USBH_ClassName( p[off + 5] ),
				p[off + 6], p[off + 7],
				p[off + 4] );
		}
		else if( type == USBH_DESC_ENDPOINT && len >= 7 )
		{
			uint8_t  ep_addr = p[off + 2];
			uint8_t  ep_attr = p[off + 3];
			uint16_t ep_size = (uint16_t)p[off + 4] | ( (uint16_t)p[off + 5] << 8 );
			uint8_t  ep_intv = p[off + 6];
			const char *dir = ( ep_addr & 0x80 ) ? "IN" : "OUT";
			const char *kind = ( ep_attr & 0x03 ) == 0x01 ? "isoch"
			                 : ( ep_attr & 0x03 ) == 0x02 ? "bulk"
			                 : ( ep_attr & 0x03 ) == 0x03 ? "intr"
			                 : "ctrl";
			printf( "      EP %s addr=0x%02x maxp=%u interval=%u (%s)\n",
				dir, ep_addr, ep_size, ep_intv, kind );
		}

		off += len;
	}
}

uint8_t USBH_Enumerate( uint8_t speed )
{
	uint8_t  ep0_size = 0;
	uint16_t cfg_len = 0;
	uint8_t  s;

	USBH_SetSelfSpeed( speed );

	s = USBH_GetDeviceDesc( &ep0_size, DevDesc );
	if( s != USBH_ERR_SUCCESS ) { printf( "GET_DESCRIPTOR(DEVICE) err %02x\n", s ); return s; }
	USBH_PrintDeviceDesc( DevDesc );
	printf( "  ep0=%u\n", ep0_size );

	s = USBH_SetAddress( ep0_size, USBH_DEVICE_ADDR );
	if( s != USBH_ERR_SUCCESS ) { printf( "SET_ADDRESS err %02x\n", s ); return s; }
	USBH_SetSelfAddr( USBH_DEVICE_ADDR );
	Delay_Ms( 3 );

	s = USBH_GetConfigDesc( ep0_size, CfgDesc, sizeof CfgDesc, &cfg_len );
	if( s != USBH_ERR_SUCCESS ) { printf( "GET_DESCRIPTOR(CONFIG) err %02x\n", s ); return s; }
	USBH_PrintConfigDesc( CfgDesc, cfg_len );

	s = USBH_SetConfig( ep0_size, CfgDesc[5] );
	if( s != USBH_ERR_SUCCESS ) { printf( "SET_CONFIGURATION err %02x\n", s ); return s; }

	printf( "enumeration complete\n" );
	USBH_OnEnumSuccess( speed, DevDesc, CfgDesc, cfg_len );
	return USBH_ERR_SUCCESS;
}

// HID bring-up + report loop.
void USBH_OnEnumSuccess( uint8_t speed,
	const uint8_t *dev_desc,
	const uint8_t *cfg_desc,
	uint16_t       cfg_len )
{
	(void)speed;

	memset( &g_hid_desc, 0, sizeof g_hid_desc );
	g_hid_kind = USBH_HID_KIND_NONE;

	uint8_t s = USBH_HidInit( dev_desc, cfg_desc, cfg_len,
		&g_hid_kind, &g_hid_rep_len,
		&g_hid_ep,   &g_hid_maxp,
		&g_hid_interval,
		&g_hid_desc );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[hid] init failed %02x (not a boot-protocol HID?)\n", s );
		return;
	}
	g_hid_tog = 0;

	const char *kind_str = "?";
	switch( g_hid_kind )
	{
		case USBH_HID_KIND_KEYBOARD: kind_str = "keyboard";   break;
		case USBH_HID_KIND_MOUSE:    kind_str = "mouse";      break;
		case USBH_HID_KIND_GAMEPAD:  kind_str = "gamepad/joy"; break;
		case USBH_HID_KIND_XBOX360:  kind_str = "xbox360";    break;
	}
	printf( "[hid] ready: %s (report=%u, report_id=%u, axes=([0]%u/%u, [1]%u/%u), wheel=%u/%u, buttons=%u)\n",
		kind_str, g_hid_rep_len, g_hid_desc.report_id,
		g_hid_desc.axis[0].size, g_hid_desc.axis[0].logical.min,
		g_hid_desc.axis[1].size, g_hid_desc.axis[1].logical.min,
		g_hid_desc.wheel.size,   g_hid_desc.wheel.logical.min,
		g_hid_desc.button_count );
}

// Print new keypresses from a boot-keyboard report (skip autorepeats).
static uint8_t prev_kbd_keys[6] = {0};
static void USBH_HidKbdHandle( const USBH_HidKbdReport *r )
{
	uint8_t shift = ( r->modifier & 0x22 ) ? 1 : 0;  // LSHIFT | RSHIFT

	for( uint8_t i = 0; i < 6; i++ )
	{
		uint8_t k = r->keycode[i];
		if( k == 0 ) continue;

		uint8_t was_held = 0;
		for( uint8_t j = 0; j < 6; j++ )
		{
			if( prev_kbd_keys[j] == k ) { was_held = 1; break; }
		}
		if( was_held ) continue;

		uint8_t c = USBH_HidKeyToAscii( k, shift );
		if( c ) putchar( c );
	}
	memcpy( prev_kbd_keys, r->keycode, 6 );
}

static void USBH_HidMouseHandle( const USBH_HidInputReport *r )
{
	if( r->buttons || r->axis[0] || r->axis[1] || r->wheel ||
	    r->buttons_extra )
	{
		printf( "[mouse] btn=%02x btnX=%02x dx=%d dy=%d wheel=%d\n",
			r->buttons, r->buttons_extra,
			(int)r->axis[0], (int)r->axis[1], (int)r->wheel );
	}
}

// Prev-state for the gamepad, so we can fire "press" / "release"
// events rather than spamming a line per report. Reset on detach
// (handled by main()).
static struct {
	uint8_t  buttons;
	uint8_t  buttons_extra;
	int16_t  axis[4];
	uint8_t  hat;     // 0 = centre, 1..8 = N..NW, 15 = null
} g_pad_prev;

static const char *USBH_HatName( uint8_t hat )
{
	switch( hat )
	{
		case 0:  return "-";
		case 1:  return "N";
		case 2:  return "NE";
		case 3:  return "E";
		case 4:  return "SE";
		case 5:  return "S";
		case 6:  return "SW";
		case 7:  return "W";
		case 8:  return "NW";
		case 15: return "null";
		default: return "?";
	}
}

// Centre an unsigned axis on its midpoint and return the signed
// delta clamped to int8_t range. The midpoint is `(min + max) / 2`,
// matching the integer truncation the device uses internally: a
// 0..255 axis centres at 127, so the at-rest value 127 produces a
// delta of 0 (no quantisation error).
//
// Signed axes (declared with `min > max` in the descriptor) are
// passed through unchanged.
static int16_t USBH_AxisCentre( int16_t raw, uint16_t lmin, uint16_t lmax,
                                uint8_t size )
{
	// Skip 1-bit axes (booleans / padding) and degenerate ranges.
	if( size == 0 || size == 1 ) return 0;
	if( lmin == lmax )           return 0;
	if( lmin == 0 && lmax == 1 ) return 0;  // boolean / hat-bit

	if( lmin > lmax )
	{
		// Signed axis — pass through (the descriptor used the
		// min > max convention to declare a signed value).
		return raw;
	}
	// Truncated midpoint: matches what the device sends.
	int32_t mid = ( (int32_t)lmin + (int32_t)lmax ) / 2;
	int32_t v   = (int32_t)raw - mid;
	if( v >  127 ) v =  127;
	if( v < -128 ) v = -128;
	return (int16_t)v;
}

static void USBH_HidGamepadHandle( const USBH_HidInputReport *r )
{
	// Print on any state change. We *always* print something so
	// the user sees releases (going back to centre) and sticks
	// moving past a small dead-zone.
	uint8_t btn_now  = r->buttons;
	uint8_t btnX_now = r->buttons_extra;
	uint8_t hat_now  = r->hat;

	uint8_t btn_press   = (uint8_t)( btn_now  & ~g_pad_prev.buttons );
	uint8_t btn_release = (uint8_t)( ~btn_now  &  g_pad_prev.buttons );
	uint8_t btnX_press  = (uint8_t)( btnX_now & ~g_pad_prev.buttons_extra );
	uint8_t btnX_release= (uint8_t)( ~btnX_now &  g_pad_prev.buttons_extra );

	// Centre each axis on its midpoint using the parsed logical
	// range. The 4 axes are X / Y / Z(or Rx) / Rz(or Ry) in
	// usage order from the parser.
	int16_t ax[4];
	for( uint8_t i = 0; i < 4; i++ )
	{
		ax[i] = USBH_AxisCentre( r->axis[i],
			g_hid_desc.axis[i].logical.min,
			g_hid_desc.axis[i].logical.max,
			g_hid_desc.axis[i].size );
	}

	int moved = 0;
	for( uint8_t i = 0; i < 4; i++ )
	{
		if( ax[i] != g_pad_prev.axis[i] ) { moved = 1; break; }
	}

	int hat_changed = ( hat_now != g_pad_prev.hat );

	if( btn_press || btn_release || btnX_press || btnX_release || moved || hat_changed )
	{
		printf( "[gamepad] btn=%02x%s btnX=%02x%s hat=%2u(%s) "
			"X=%4d Y=%4d Rx=%4d Ry=%4d\n",
			btn_now,  btn_press   ? "+" : ( btn_release   ? "-" : "" ),
			btnX_now, btnX_press  ? "+" : ( btnX_release  ? "-" : "" ),
			(unsigned)hat_now, USBH_HatName( hat_now ),
			(int)ax[0], (int)ax[1], (int)ax[2], (int)ax[3] );
	}

	g_pad_prev.buttons       = btn_now;
	g_pad_prev.buttons_extra = btnX_now;
	for( uint8_t i = 0; i < 4; i++ ) g_pad_prev.axis[i] = ax[i];
	g_pad_prev.hat = hat_now;
}

static const char *USBH_XboxBtnName( uint8_t b )
{
	switch( b )
	{
		case 0x01: return "UP";     case 0x02: return "DOWN";
		case 0x04: return "LEFT";   case 0x08: return "RIGHT";
		case 0x10: return "START";  case 0x20: return "BACK";
		case 0x40: return "L3";     case 0x80: return "R3";
		default:   return NULL;
	}
}

static const char *USBH_XboxBtnNameH( uint8_t b )
{
	switch( b )
	{
		case 0x10: return "A";     case 0x20: return "B";
		case 0x40: return "X";     case 0x80: return "Y";
		case 0x01: return "LB";    case 0x02: return "RB";
		case 0x04: return "GUIDE";
		default:   return NULL;
	}
}

static void USBH_HidX360Handle( const USBH_HidX360Report *r )
{
	static uint8_t prev_low = 0, prev_high = 0;
	uint8_t now_low  = r->buttons_low;
	uint8_t now_high = r->buttons_high;

	// Rising-edge print for the basic buttons (only fire on press).
	if( now_low != prev_low )
	{
		uint8_t diff = (uint8_t)( now_low & ~prev_low );
		for( uint8_t b = 0; b < 8; b++ )
		{
			if( diff & ( 1u << b ) )
			{
				const char *n = USBH_XboxBtnName( (uint8_t)( 1u << b ) );
				if( n ) printf( "[x360] dpad/btn: %s\n", n );
			}
		}
		prev_low = now_low;
	}
	if( now_high != prev_high )
	{
		uint8_t diff = (uint8_t)( now_high & ~prev_high );
		for( uint8_t b = 0; b < 8; b++ )
		{
			if( diff & ( 1u << b ) )
			{
				const char *n = USBH_XboxBtnNameH( (uint8_t)( 1u << b ) );
				if( n ) printf( "[x360] btn: %s\n", n );
			}
		}
		prev_high = now_high;
	}

	// Sticks: only print on a meaningful change (1% threshold).
	static int16_t plx = 0, ply = 0, prx = 0, pry = 0;
	if( ( r->lx > plx + 1000 ) || ( r->lx < plx - 1000 ) ||
	    ( r->ly > ply + 1000 ) || ( r->ly < ply - 1000 ) ||
	    ( r->rx > prx + 1000 ) || ( r->rx < prx - 1000 ) ||
	    ( r->ry > pry + 1000 ) || ( r->ry < pry - 1000 ) ||
	    r->lt > 0 || r->rt > 0 )
	{
		printf( "[x360] LX=%d LY=%d RX=%d RY=%d LT=%u RT=%u\n",
			(int)r->lx, (int)r->ly, (int)r->rx, (int)r->ry,
			(unsigned)r->lt, (unsigned)r->rt );
		plx = r->lx; ply = r->ly; prx = r->rx; pry = r->ry;
	}
}

// Main loop
int main( void )
{
	SystemInit();

	USBH_ClockInit();
	printf( "[clk] CFGR0=%08lx USBPRE=%lu\n",
		(unsigned long)RCC->CFGR0,
		(unsigned long)( ( RCC->CFGR0 >> 22 ) & 3u ) );
	USBH_GpioInit();
	USBH_HostInit();

	printf( "\n[usbhost_hid] CH32V203 USBFS host, HCLK=%lu Hz\n",
		(unsigned long)USBH_GetHclk() );
	printf( "[usbhost_hid] waiting for device...\n" );

	uint8_t last_state = 0xFF;
	uint8_t speed = USB_SPEED_UNKNOWN;
	uint8_t enumerated = 0;
	uint32_t last_print_ms = 0;

	for( ;; )
	{
		uint8_t state = USBH_PortCheckStatus();
		uint32_t now = funSysTick32();

		if( state != last_state )
		{
			switch( state )
			{
				case USBH_PORT_DETACHED:
					printf( "[usbhost_hid] device removed\n" );
					speed = USB_SPEED_UNKNOWN;
					enumerated = 0;
					g_hid_kind = USBH_HID_KIND_NONE;
					memset( &g_pad_prev, 0, sizeof g_pad_prev );
					g_pad_prev.hat = 0xFFu;  // force a "null → centre" print on next attach
					break;

				case USBH_PORT_ATTACHED:
					printf( "[usbhost_hid] device attached, resetting bus...\n" );
					Delay_Ms( 100 );
					USBH_BusReset( 0 );
					{
						uint8_t rc = USBH_PortEnable( &speed );
						if( rc == USBH_PORT_ENABLED )
						{
							printf( "[usbhost_hid] port enabled, speed=%s\n",
								speed == USB_SPEED_LOW  ? "low (1.5 Mbps)" :
								speed == USB_SPEED_FULL ? "full (12 Mbps)" :
								                                            "unknown" );
						}
						else printf( "[usbhost_hid] port enable failed rc=%u\n", rc );
					}
					break;

				case USBH_PORT_ENABLED:
					break;
			}
			last_state = state;
		}

		// Enumeration: same retry pattern as the reference example.
		if( state == USBH_PORT_ENABLED && !enumerated )
		{
			enumerated = 1;
			uint8_t s = USBH_ERR_USB_UNKNOWN;
			for( uint8_t etry = 0; etry < 5 && USBH_IsDevicePresent(); etry++ )
			{
				if( etry > 0 )
				{
					printf( "[usbhost_hid] enum err=%02x, reset+retry (%u/5)\n",
						s, (unsigned)(etry + 1) );
					Delay_Ms( 100 );
					USBH_BusReset( 0 );
					if( USBH_PortEnable( &speed ) != USBH_PORT_ENABLED ) break;
				}
				s = USBH_Enumerate( speed );
				if( s == USBH_ERR_SUCCESS ) break;
			}
		}

		// HID report loop. Runs only when the device is configured
		// and the HID driver was brought up. We don't sleep
		// aggressively - the WCH SIE will NAK if no report is ready,
		// and the BulkOrIntrIn retry budget is 10 * 1 ms ~= 10 ms.
		if( state == USBH_PORT_ENABLED && g_hid_kind != USBH_HID_KIND_NONE )
		{
			uint8_t  raw[ USBH_HID_REPORT_MAX ];
			uint8_t  nread = 0;
			uint8_t  r = USBH_HidReadReport( g_hid_ep, &g_hid_tog, g_hid_maxp,
				raw, sizeof raw, &nread );
			if( r == USBH_ERR_SUCCESS )
			{
				switch( g_hid_kind )
				{
					case USBH_HID_KIND_KEYBOARD:
					{
						// Both boot- and report-protocol keyboards
						// share the [mod,res,k0..k5] layout.
						if( nread >= 8 ) {
							USBH_HidKbdHandle( (USBH_HidKbdReport*)raw );
						}
						break;
					}
					case USBH_HID_KIND_MOUSE:
					{
						if( g_hid_rep_len == 3 )
						{
							// Boot mouse: 3 bytes, fixed layout.
							USBH_HidMouseReport m = {
								.buttons = raw[0],
								.x = (int8_t)raw[1],
								.y = (int8_t)raw[2]
							};
							USBH_HidInputReport ir;
							memset( &ir, 0, sizeof ir );
							ir.axis[0] = m.x;
							ir.axis[1] = m.y;
							ir.buttons = m.buttons;
							USBH_HidMouseHandle( &ir );
						}
						else
						{
							USBH_HidInputReport ir;
							USBH_HidDecodeGeneric( &g_hid_desc,
								g_hid_kind, raw, nread, &ir );
							USBH_HidMouseHandle( &ir );
						}
						break;
					}
					case USBH_HID_KIND_GAMEPAD:
					{
						USBH_HidInputReport ir;
						USBH_HidDecodeGeneric( &g_hid_desc,
							g_hid_kind, raw, nread, &ir );
						// If we don't see any button change after a
						// while, dump the raw report bytes once to
						// help diagnose layout issues. The dump is
						// throttled to once per N reports.
						{
							static uint32_t dump_counter = 0;
							static uint8_t  last_dumped[16] = {0};
							uint8_t changed = 0;
							for( uint8_t i = 0; i < nread && i < 16; i++ )
							{
								if( raw[i] != last_dumped[i] ) { changed = 1; break; }
							}
							if( changed || ( dump_counter & 0xff ) == 0 )
							{
								printf( "[hid] raw[%u]:", nread );
								for( uint8_t i = 0; i < nread && i < 16; i++ )
									printf( " %02x", raw[i] );
								printf( "\n" );
								memcpy( last_dumped, raw, nread < 16 ? nread : 16 );
							}
							dump_counter++;
						}
						USBH_HidGamepadHandle( &ir );
						break;
					}
					case USBH_HID_KIND_XBOX360:
					{
						USBH_HidX360Report xr;
						if( USBH_HidDecodeX360( raw, nread, &xr ) == 0 )
							USBH_HidX360Handle( &xr );
						break;
					}
					default:
						break;
				}
			}
			else if( r == USBH_ERR_USB_DISCON )
			{
				g_hid_kind = USBH_HID_KIND_NONE;
			}
			else
			{
				// Other error: small backoff so we don't spin 100%
				// CPU on a noisy bus.
				Delay_Ms( 5 );
			}
		}
		else
		{
			if( ( now - last_print_ms ) > 12000000 )
			{
				last_print_ms = now;
				if( state == USBH_PORT_DETACHED ) printf( "." );
			}
		}
	}
}
