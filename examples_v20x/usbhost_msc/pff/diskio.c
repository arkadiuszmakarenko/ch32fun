// diskio.c -- Low level disk I/O module for Petit FatFs on ch32fun.
//
// Petit FatFs is (C) ChaN, 2014. This shim is what we hand it.
//
// The file provides three functions the PFF core calls:
//   disk_initialize() -- claim a USB mass-storage device, return 0
//                        on success.
//   disk_readp(buff, sector, offset, count) -- read a partial block
//                        (typically a few bytes from a 512-byte
//                        sector) from the attached device.
//   disk_writep(buff, sector) -- write a partial block. PFF can
//                        stream multiple calls with `buff` non-NULL
//                        to fill a 512-byte buffer; pass `buff=NULL`
//                        with `sc`=0 to flush, or `buff=NULL` with
//                        `sc`=sector to start a new write at sector
//                        `sector` (with an empty buffer).
//
// The actual USB Bulk-Only Transport is in usbhost_msc.c -- we just
// call USBH_MscRead() / USBH_MscWrite() here.

#include "diskio.h"
#include "usbhost_msc.h"
#include "usbhost_xfer.h"
#include "usbhost_defs.h"
#include <string.h>
#include <stdio.h>

// Block size is always 512 for a USB flash drive; the BOT layer
// validates this on init and we use it directly here.
#define DISK_BLOCK_SIZE  512

// PFF expects DSTATUS bit flags.
#define STA_NOINIT       0x01
#define STA_NODISK       0x02

// One full sector of buffer for assembling PFF's partial reads.
static uint8_t g_sector_buf[ DISK_BLOCK_SIZE ];

// disk_initialize -- DSTATUS per the PFF contract.
DSTATUS disk_initialize( void )
{
	if( !USBH_MscIsPresent() ) return STA_NOINIT | STA_NODISK;
	return 0;
}

// disk_readp -- read `count` bytes at `offset` from sector `sector`.
DRESULT disk_readp( BYTE *buff, DWORD sector, UINT offset, UINT count )
{
	if( !USBH_MscIsPresent() ) return RES_NOTRDY;
	if( offset + count > DISK_BLOCK_SIZE ) return RES_PARERR;

	uint8_t s = USBH_MscRead( sector, g_sector_buf, 1 );
	if( s != USBH_ERR_SUCCESS ) return RES_ERROR;
	if( buff ) memcpy( buff, &g_sector_buf[ offset ], count );
	return RES_OK;
}

// disk_writep -- buffered sector write. PFF either:
//   - starts a new write  : buff=NULL, sc=sector
//   - writes `sc` bytes   : buff!=NULL, sc=bytecount
//   - flushes the buffer  : buff=NULL, sc=0
static DWORD   g_w_sector = 0;
static UINT    g_w_offset = 0;
static uint8_t g_w_active = 0;

DRESULT disk_writep( const BYTE *buff, DWORD sc )
{
	if( !USBH_MscIsPresent() ) return RES_NOTRDY;

	if( buff == NULL )
	{
		if( sc == 0 )
		{
			// Flush: write the current buffered sector (if any
			// data has accumulated) and clear the active state.
			if( g_w_active && g_w_offset > 0 )
			{
				uint8_t s = USBH_MscWrite( g_w_sector, g_sector_buf, 1 );
				g_w_active = 0;
				g_w_offset = 0;
				return s == USBH_ERR_SUCCESS ? RES_OK : RES_ERROR;
			}
			g_w_active = 0;
			g_w_offset = 0;
			return RES_OK;
		}
		// buff=NULL, sc=sector -> start a new write at `sector`.
		g_w_sector = sc;
		g_w_offset = 0;
		g_w_active = 1;
		memset( g_sector_buf, 0, DISK_BLOCK_SIZE );
		return RES_OK;
	}

	// buff!=NULL: copy `sc` bytes into the buffer, flushing if full.
	if( !g_w_active ) return RES_ERROR;
	UINT remaining = sc;
	while( remaining > 0 )
	{
		UINT room = DISK_BLOCK_SIZE - g_w_offset;
		UINT chunk = remaining < room ? remaining : room;
		memcpy( &g_sector_buf[ g_w_offset ], buff, chunk );
		g_w_offset += chunk;
		buff       += chunk;
		remaining  -= chunk;
		if( g_w_offset == DISK_BLOCK_SIZE )
		{
			uint8_t s = USBH_MscWrite( g_w_sector, g_sector_buf, 1 );
			if( s != USBH_ERR_SUCCESS )
			{
				g_w_active = 0;
				return RES_ERROR;
			}
			g_w_sector++;
			g_w_offset = 0;
			memset( g_sector_buf, 0, DISK_BLOCK_SIZE );
		}
	}
	return RES_OK;
}
