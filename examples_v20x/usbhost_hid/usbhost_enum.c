// usbhost_enum.c -- project-local main for usbhost_hid.
//
// Carries the same enumeration state machine as the usbhost_enum
// reference, but at the end of a successful enumeration we call
// USBH_HidInit() and then loop on USBH_HidReadReport(), printing
// decoded keyboard characters and mouse deltas.

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
static uint8_t  g_hid_rep_len;
static uint8_t  g_hid_tog;
static uint8_t  g_hid_kind;   // 0=none, 1=keyboard, 2=mouse

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
	(void)speed; (void)dev_desc;

	g_hid_kind = 0;
	uint8_t s = USBH_HidInit( cfg_desc, cfg_len,
		&g_hid_rep_len, &g_hid_ep,
		&g_hid_maxp,    &g_hid_interval );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[hid] init failed %02x (not a boot-protocol HID?)\n", s );
		return;
	}
	g_hid_tog = 0;
	// Heuristic: 8-byte report -> keyboard, 3-byte -> mouse.
	g_hid_kind = ( g_hid_rep_len == 8 ) ? 1 : ( g_hid_rep_len == 3 ? 2 : 0 );
	printf( "[hid] ready: %s (report=%u)\n",
		g_hid_kind == 1 ? "keyboard" :
		g_hid_kind == 2 ? "mouse"    : "unknown",
		g_hid_rep_len );
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

static void USBH_HidMouseHandle( const USBH_HidMouseReport *r )
{
	if( r->buttons || r->x || r->y )
	{
		printf( "[mouse] btn=%02x dx=%d dy=%d\n",
			r->buttons, (int)r->x, (int)r->y );
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
					g_hid_kind = 0;
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
		if( state == USBH_PORT_ENABLED && g_hid_kind != 0 )
		{
			uint8_t rep[ 8 ] = {0};
			uint8_t r = USBH_HidReadReport( g_hid_ep, &g_hid_tog, g_hid_maxp,
				rep, g_hid_rep_len );
			if( r == USBH_ERR_SUCCESS )
			{
				if( g_hid_kind == 1 ) USBH_HidKbdHandle( (USBH_HidKbdReport*)rep );
				else                  USBH_HidMouseHandle( (USBH_HidMouseReport*)rep );
			}
			else if( r == USBH_ERR_USB_DISCON )
			{
				g_hid_kind = 0;
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
