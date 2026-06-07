// usbhost_enum.c -- project-local main for usbhost_msc.
//
// Carries the same enumeration state machine as the usbhost_enum
// reference, but at the end of a successful enumeration we call
// USBH_MscAttach() (which brings up the BOT layer), then mount the
// FAT volume via Petit FatFs and list the root directory.
//
// If a non-MSC device is attached, MSC init fails and the example
// simply sits idle waiting for the next attach.

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_hw.h"
#include "usbhost_xfer.h"
#include "usbhost_std.h"
#include "usbhost_defs.h"
#include "usbhost_enum.h"
#include "usbhost_msc.h"

// Petit FatFs.
#include "pff.h"
#include "diskio.h"

// Standard address assigned to the device after SET_ADDRESS.
#define USBH_DEVICE_ADDR   0x02u

// Buffers for parsed descriptors.
static uint8_t DevDesc[ 18 ];
static uint8_t CfgDesc[ USBH_CFG_DESC_BUFFER_SIZE ];

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

// MSC + PFF bring-up.
//
// On success: print the FAT type, list the root directory, and read
// the first 4 KiB of "firmware.bin" if it's present. The read loop
// is just a smoke test -- the example doesn't try to do anything
// with the file contents.
static FATFS g_fs;

void USBH_OnEnumSuccess( uint8_t speed,
	const uint8_t *dev_desc,
	const uint8_t *cfg_desc,
	uint16_t       cfg_len )
{
	(void)speed; (void)dev_desc;

	uint8_t s = USBH_MscAttach( cfg_desc, cfg_len, 64 );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[msc] attach failed %02x (not a mass-storage device?)\n", s );
		return;
	}

	// Mount the volume. PFF's pf_mount() returns FR_OK on a valid
	// BPB. We don't try to use the LBA value here -- PFF discovers
	// partition / FAT geometry from the boot sector itself.
	FRESULT r = pf_mount( &g_fs );
	if( r != FR_OK )
	{
		printf( "[pff] mount err %d\n", (int)r );
		return;
	}
	printf( "[pff] volume mounted (fs_type=%u)\n", (unsigned)g_fs.fs_type );

	// List the root directory. PFF can iterate entries one at a time.
	DIR d;
	FILINFO fi;
	if( pf_opendir( &d, "/" ) == FR_OK )
	{
		printf( "[pff] root directory:\n" );
		for( ;; )
		{
			r = pf_readdir( &d, &fi );
			if( r != FR_OK || fi.fname[0] == 0 ) break;
			printf( "  %c %8lu  %s\n",
				( fi.fattrib & AM_DIR ) ? 'd' : '-',
				(unsigned long)fi.fsize, fi.fname );
		}
	}

	// Try to open "firmware.bin" -- a common IAP target. If it's not
	// there, we just print "not found" and stop.
	if( pf_open( "firmware.bin" ) == FR_OK )
	{
		uint8_t buf[ 64 ];
		UINT    br = 0;
		uint32_t total = 0;
		printf( "[pff] firmware.bin present, reading 4 KiB...\n" );
		for( int i = 0; i < 64; i++ )
		{
			pf_read( buf, sizeof buf, &br );
			if( br == 0 ) break;
			total += br;
		}
		printf( "[pff] read %lu bytes\n", (unsigned long)total );
	}
	else
	{
		printf( "[pff] no firmware.bin in root\n" );
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

	printf( "\n[usbhost_msc] CH32V203 USBFS host, HCLK=%lu Hz\n",
		(unsigned long)USBH_GetHclk() );
	printf( "[usbhost_msc] waiting for device...\n" );

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
					printf( "[usbhost_msc] device removed\n" );
					speed = USB_SPEED_UNKNOWN;
					enumerated = 0;
					break;

				case USBH_PORT_ATTACHED:
					printf( "[usbhost_msc] device attached, resetting bus...\n" );
					Delay_Ms( 100 );
					USBH_BusReset( 0 );
					{
						uint8_t rc = USBH_PortEnable( &speed );
						if( rc == USBH_PORT_ENABLED )
						{
							printf( "[usbhost_msc] port enabled, speed=%s\n",
								speed == USB_SPEED_LOW  ? "low (1.5 Mbps)" :
								speed == USB_SPEED_FULL ? "full (12 Mbps)" :
								                                            "unknown" );
						}
						else printf( "[usbhost_msc] port enable failed rc=%u\n", rc );
					}
					break;

				case USBH_PORT_ENABLED:
					break;
			}
			last_state = state;
		}

		if( state == USBH_PORT_ENABLED && !enumerated )
		{
			enumerated = 1;
			uint8_t s = USBH_ERR_USB_UNKNOWN;
			for( uint8_t etry = 0; etry < 5 && USBH_IsDevicePresent(); etry++ )
			{
				if( etry > 0 )
				{
					printf( "[usbhost_msc] enum err=%02x, reset+retry (%u/5)\n",
						s, (unsigned)(etry + 1) );
					Delay_Ms( 100 );
					USBH_BusReset( 0 );
					if( USBH_PortEnable( &speed ) != USBH_PORT_ENABLED ) break;
				}
				s = USBH_Enumerate( speed );
				if( s == USBH_ERR_SUCCESS ) break;
			}
		}

		// Idle: heartbeat once per 2 s.
		if( ( now - last_print_ms ) > 12000000 )
		{
			last_print_ms = now;
			if( state == USBH_PORT_DETACHED ) printf( "." );
		}
	}
}
