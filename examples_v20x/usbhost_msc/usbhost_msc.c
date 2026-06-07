// usbhost_msc.c
//
// USB Mass Storage Class driver -- Bulk-Only Transport (BBB) + SCSI.
//
// On top of the ch32fun USB host primitives (USBH_BulkOrIntr{In,Out},
// USBH_ClrStall, USBH_CtrlXfer), this file implements the BOT
// protocol:
//
//   Host -> Device:  CBW (31 bytes) on bulk-OUT
//   Device -> Host:  optional data on bulk-IN  (if data_len>0, dir=IN)
//                   or
//                   Host -> Device: data on bulk-OUT (if data_len>0, dir=OUT)
//   Device -> Host:  CSW (13 bytes) on bulk-IN
//
// We then issue a small set of standard SCSI commands:
//   - TEST UNIT READY    (0x00)
//   - REQUEST SENSE      (0x03)
//   - INQUIRY            (0x12)        - not strictly required by PFF
//   - READ CAPACITY (10) (0x25)
//   - READ (10)          (0x28)
//   - WRITE (10)         (0x2A)
//
// On any error we clear both bulk endpoints' halt feature and retry.
// After `msc_reset_max_attempts` failures we give up.

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_hw.h"
#include "usbhost_xfer.h"
#include "usbhost_defs.h"
#include "usbhost_msc.h"

// Class / protocol constants.
#define USBH_MSC_CLASS          0x08
#define USBH_MSC_SUBCLASS_SCSI  0x06
#define USBH_MSC_PROTOCOL_BOT   0x50
#define USBH_EP_TYPE_BULK       0x02

// BOT class requests (host -> device, class, interface).
#define USBH_MSC_REQ_GET_MAX_LUN    0xFE
#define USBH_MSC_REQ_RESET          0xFF

// BOT signatures (USBC spec).
#define USBH_MSC_CBW_SIG    0x43425355u   // "USBC" little-endian
#define USBH_MSC_CSW_SIG    0x53425355u   // "USBS" little-endian

// SCSI opcodes we use.
#define SCSI_TEST_UNIT_READY    0x00
#define SCSI_REQUEST_SENSE      0x03
#define SCSI_INQUIRY            0x12
#define SCSI_READ_CAPACITY_10   0x25
#define SCSI_READ_10            0x28
#define SCSI_WRITE_10           0x2A

// Retry budgets.
#define USBH_MSC_BOT_MAX_TRIES   4
#define USBH_MSC_BOT_SETTLE_US   200

// Per-endpoint state. The class driver is single-threaded so we can
// use static storage safely.
static uint8_t  msc_bulk_in_ep   = 0;     // low nibble, no direction
static uint8_t  msc_bulk_out_ep  = 0;
static uint8_t  msc_bulk_in_tog  = 0;
static uint8_t  msc_bulk_out_tog = 0;
static uint32_t msc_bulk_in_maxp = 0;
static uint32_t msc_bulk_out_maxp = 0;
static uint32_t msc_block_size   = 512;
static uint8_t  msc_ep0_size     = 64;    // cached from enumeration
static uint32_t msc_tag_counter  = 0;
static uint8_t  msc_present      = 0;     // 1 after successful USBH_MscAttach

// Per-instance accessors used by diskio.c. diskio.c doesn't know
// the EP numbers -- it just calls USBH_MscRead / USBH_MscWrite.
uint8_t USBH_MscIsPresent( void )
{
	return msc_present;
}

// On-the-wire CBW / CSW structures (packed, little-endian on the bus).
typedef struct __attribute__((packed))
{
	uint32_t dCBWSignature;
	// 0x55534243 ("USBC")
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t  bmCBWFlags;
	// 0x80 = IN, 0x00 = OUT
	uint8_t  bCBWLUN;
	// typically 0
	uint8_t  bCBWCBLength;
	// 1..16
	uint8_t  CBWCB[16];
	// SCSI CDB
} USBH_MscCbw;

typedef struct __attribute__((packed))
{
	uint32_t dCSWSignature;
	// 0x55534253 ("USBS")
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t  bCSWStatus;
	// 0=ok, 1=fail, 2=phase error
} USBH_MscCsw;

// Helper: 32-bit little-endian writer.
static inline void put32_le( uint8_t *p, uint32_t v )
{
	p[0] = (uint8_t)( v & 0xFF );
	p[1] = (uint8_t)( (v >> 8) & 0xFF );
	p[2] = (uint8_t)( (v >> 16) & 0xFF );
	p[3] = (uint8_t)( (v >> 24) & 0xFF );
}

// Send a CBW (31 bytes) on bulk-OUT.
static uint8_t msc_send_cbw( const uint8_t *cdb, uint8_t cdb_len,
	uint32_t data_len, uint8_t direction )
{
	USBH_MscCbw cbw;

	put32_le( (uint8_t*)&cbw.dCBWSignature, USBH_MSC_CBW_SIG );
	put32_le( (uint8_t*)&cbw.dCBWTag, ++msc_tag_counter );
	put32_le( (uint8_t*)&cbw.dCBWDataTransferLength, data_len );
	cbw.bmCBWFlags = direction;
	cbw.bCBWLUN = 0;
	cbw.bCBWCBLength = cdb_len;
	memcpy( cbw.CBWCB, cdb, cdb_len );
	memset( &cbw.CBWCB[cdb_len], 0, 16 - cdb_len );

	uint8_t s = USBH_BulkOrIntrOut( msc_bulk_out_ep, &msc_bulk_out_tog,
		(uint8_t*)&cbw, sizeof(cbw), 2000 );
	return s;
}

// Receive a CSW (13 bytes) on bulk-IN. We always read the full CSW
// structure; some devices may send it as a 1-byte residue that's
// padded -- we don't need to care, the BOT spec says CSW is always 13.
static uint8_t msc_receive_csw( uint8_t *pstatus )
{
	USBH_MscCsw csw;
	uint8_t rx = 0;
	uint8_t s;

	s = USBH_BulkOrIntrIn( msc_bulk_in_ep, msc_bulk_in_tog,
		(uint8_t*)&csw, &rx, 2000 );
	if( s != USBH_ERR_SUCCESS ) return s;

	if( rx != sizeof(csw) )
	{
		printf( "msc: CSW short rx=%u\n", (unsigned)rx );
		return USBH_ERR_USB_MSC_CSW;
	}

	uint32_t sig;
	memcpy( &sig, &csw.dCSWSignature, 4 );
	if( sig != USBH_MSC_CSW_SIG )
	{
		printf( "msc: CSW bad sig %08lx\n", (unsigned long)sig );
		return USBH_ERR_USB_MSC_CSW;
	}

	*pstatus = csw.bCSWStatus;
	return USBH_ERR_SUCCESS;
}

// STALL recovery: clear halt on both bulk endpoints, reset toggles to 0.
static void msc_clear_halts( void )
{
	if( msc_bulk_in_ep )  USBH_ClrStall( 0x80 | msc_bulk_in_ep,  msc_ep0_size );
	if( msc_bulk_out_ep ) USBH_ClrStall( msc_bulk_out_ep,        msc_ep0_size );
	msc_bulk_in_tog = 0;
	msc_bulk_out_tog = 0;
	Delay_Us( USBH_MSC_BOT_SETTLE_US );
}

// Mass-Storage-Reset class request. Resets the device to its
// post-Power-On state. Issued on initialisation and on hard errors.
static uint8_t msc_reset( uint8_t iface_num )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t s;

	req.bmRequestType = 0x21u;  // host-to-device, class, interface
	req.bRequest = USBH_MSC_REQ_RESET;
	((uint8_t*)&req.wValue)[0] = 0;
	((uint8_t*)&req.wValue)[1] = 0;
	((uint8_t*)&req.wIndex)[0] = iface_num;
	((uint8_t*)&req.wIndex)[1] = 0;
	((uint8_t*)&req.wLength)[0] = 0;
	((uint8_t*)&req.wLength)[1] = 0;

	s = USBH_CtrlXfer( &req, NULL, 0, msc_ep0_size, &got );
	Delay_Ms( 50 );
	return s;
}

// Walk the configuration descriptor for the first Mass Storage
// interface (class=8, subclass=6, protocol=0x50), and within that
// interface find the bulk-IN and bulk-OUT endpoints.
static uint8_t msc_find_interface( const uint8_t *cfg, uint16_t total,
	uint8_t *piface, uint8_t *pin_ep, uint8_t *pout_ep )
{
	uint16_t off = 0;
	while( off + 2 <= total )
	{
		uint8_t len = cfg[off];
		uint8_t type = cfg[off + 1];
		if( len == 0 ) break;
		if( off + len > total ) break;

		if( type == USBH_DESC_INTERFACE && len >= 9 )
		{
			uint8_t cls = cfg[off + 5];
			uint8_t sub = cfg[off + 6];
			uint8_t proto = cfg[off + 7];
			uint8_t n_ep = cfg[off + 4];

			if( cls == USBH_MSC_CLASS &&
			    sub == USBH_MSC_SUBCLASS_SCSI &&
			    proto == USBH_MSC_PROTOCOL_BOT )
			{
				*piface = cfg[off + 2];

				// Walk forward to find bulk endpoints. The endpoint
				// descriptors may follow any class-specific descriptors
				// (none for plain BBB), so we scan until the next
				// interface or the end of the config.
				uint16_t end = off + len;
				while( end + 2 <= total && cfg[end + 1] != USBH_DESC_INTERFACE )
				{
					uint8_t elen = cfg[end];
					if( elen == 0 ) break;
					end += elen;
				}

				*pin_ep = 0; *pout_ep = 0;
				for( uint16_t e = off + len; e < end; e += cfg[e] )
				{
					if( cfg[e] < 7 ) continue;
					if( cfg[e + 1] != USBH_DESC_ENDPOINT ) continue;
					uint8_t eaddr = cfg[e + 2];
					uint8_t eattr = cfg[e + 3];
					uint16_t emaxp = (uint16_t)cfg[e + 4] | ((uint16_t)cfg[e + 5] << 8);
					if( ( eattr & 0x03 ) != USBH_EP_TYPE_BULK ) continue;
					if( eaddr & 0x80 )
					{
						*pin_ep = eaddr & 0x0F;
						msc_bulk_in_maxp = emaxp;
					}
					else
					{
						*pout_ep = eaddr & 0x0F;
						msc_bulk_out_maxp = emaxp;
					}
				}

				if( *pin_ep && *pout_ep ) return USBH_ERR_SUCCESS;
				printf( "msc: BOT iface #%u found but bulk EP missing\n",
					*piface );
				return USBH_ERR_USB_UNKNOWN;
			}
			(void)n_ep;
		}

		off += len;
	}
	return USBH_ERR_USB_UNKNOWN;
}

// TEST UNIT READY with retry / sense. Returns 0 on success.
static uint8_t msc_test_unit_ready( void )
{
	uint8_t cdb[6] = { SCSI_TEST_UNIT_READY, 0, 0, 0, 0, 0 };
	uint8_t status = 0;
	uint8_t s;
	uint8_t sense[18];

	for( uint8_t attempt = 0; attempt < 5; attempt++ )
	{
		s = msc_send_cbw( cdb, 6, 0, 0x00 );
		if( s != USBH_ERR_SUCCESS )
		{
			msc_clear_halts();
			continue;
		}
		s = msc_receive_csw( &status );
		if( s == USBH_ERR_SUCCESS && status == 0 )
		{
			return USBH_ERR_SUCCESS;
		}
		// Drive indicated failure: ask what went wrong.
		// REQUEST SENSE: 6-byte CDB, 18 bytes response, IN.
		uint8_t rsense = 0;
		cdb[0] = SCSI_REQUEST_SENSE;
		cdb[1] = 0; cdb[2] = 0; cdb[3] = 0; cdb[4] = 18; cdb[5] = 0;
		if( msc_send_cbw( cdb, 6, 18, 0x80 ) == USBH_ERR_SUCCESS )
		{
			uint8_t rxl = 0;
			(void)USBH_BulkOrIntrIn( msc_bulk_in_ep, msc_bulk_in_tog,
				sense, &rxl, 2000 );
			msc_receive_csw( &rsense );
		}
		uint8_t sk = sense[2] & 0x0F;
		if( sk == 0x02 ) { Delay_Ms( 100 ); continue; }  // NOT READY
		if( sk == 0x06 ) { Delay_Ms( 50 );  continue; }  // UNIT ATTENTION
		msc_clear_halts();
	}
	return USBH_ERR_USB_MSC_CSW;
}

// READ CAPACITY (10): returns last LBA and block size.
static uint8_t msc_read_capacity( uint32_t *plast_lba, uint32_t *pblock_size )
{
	uint8_t  cdb[10] = { SCSI_READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t  data[8] = {0};
	uint8_t  status = 0;
	uint8_t  rx = 0;
	uint8_t  s;

	s = msc_send_cbw( cdb, 10, 8, 0x80 );
	if( s != USBH_ERR_SUCCESS ) return USBH_ERR_USB_MSC_CBW;
	Delay_Us( 100 );
	s = USBH_BulkOrIntrIn( msc_bulk_in_ep, msc_bulk_in_tog, data, &rx, 2000 );
	if( s != USBH_ERR_SUCCESS || rx != 8 ) return USBH_ERR_USB_MSC_CBW;
	if( msc_receive_csw( &status ) != USBH_ERR_SUCCESS ) return USBH_ERR_USB_MSC_CSW;
	if( status != 0 ) return USBH_ERR_USB_MSC_CSW;

	*plast_lba =
		((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
		((uint32_t)data[2] <<  8) | ((uint32_t)data[3]      );
	*pblock_size =
		((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
		((uint32_t)data[6] <<  8) | ((uint32_t)data[7]      );
	return USBH_ERR_SUCCESS;
}

// Public: bring up the MSC class driver.
uint8_t USBH_MscInit( const uint8_t *cfg_desc, uint16_t cfg_len,
	uint8_t ep0_size_in,
	uint8_t *pin_ep, uint16_t *pin_maxp,
	uint8_t *pout_ep, uint16_t *pout_maxp,
	uint32_t *plast_lba, uint32_t *pblock_size )
{
	uint8_t iface = 0;
	uint8_t s;

	msc_ep0_size = ep0_size_in;
	s = msc_find_interface( cfg_desc, cfg_len, &iface, pin_ep, pout_ep );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[msc] no BOT interface\n" );
		return s;
	}
	msc_bulk_in_ep = *pin_ep;
	msc_bulk_out_ep = *pout_ep;
	msc_bulk_in_tog = 0;
	msc_bulk_out_tog = 0;
	*pin_maxp = (uint16_t)msc_bulk_in_maxp;
	*pout_maxp = (uint16_t)msc_bulk_out_maxp;

	// Class-level reset. Some flash drives need this to come out of
	// a sticky "stalled" state left over from a previous host.
	s = msc_reset( iface );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[msc] BOT reset err %02x\n", s );
		// Not fatal -- continue.
	}

	// Allow the device a moment to settle after the reset.
	Delay_Ms( 50 );

	// Wait for the unit to be ready.
	s = msc_test_unit_ready();
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[msc] TEST_UNIT_READY failed\n" );
		return s;
	}

	uint32_t lba = 0, bsz = 0;
	s = msc_read_capacity( &lba, &bsz );
	if( s != USBH_ERR_SUCCESS )
	{
		printf( "[msc] READ_CAPACITY failed %02x\n", s );
		return USBH_ERR_USB_MSC_CAP;
	}
	if( bsz == 0 || bsz > 4096 ) bsz = 512;  // some drives report 0
	msc_block_size = bsz;

	printf( "[msc] BOT iface #%u ep_in=0x%02x ep_out=0x%02x last_lba=%lu block=%lu\n",
		iface, msc_bulk_in_ep, msc_bulk_out_ep,
		(unsigned long)lba, (unsigned long)bsz );

	*plast_lba = lba;
	*pblock_size = bsz;
	return USBH_ERR_SUCCESS;
}

// USBH_MscAttach -- convenience wrapper that the project main() /
// diskio.c calls. Drives the BOT init and reports the final
// success/failure.
uint8_t USBH_MscAttach( const uint8_t *cfg_desc, uint16_t cfg_len,
	uint8_t ep0_size )
{
	uint8_t  in_ep = 0, out_ep = 0;
	uint16_t in_maxp = 0, out_maxp = 0;
	uint32_t last_lba = 0, block_size = 512;

	uint8_t s = USBH_MscInit( cfg_desc, cfg_len, ep0_size,
		&in_ep, &in_maxp,
		&out_ep, &out_maxp,
		&last_lba, &block_size );
	msc_present = ( s == USBH_ERR_SUCCESS ) ? 1u : 0u;
	return s;
}

// Public: read `count` blocks starting at `lba`.
uint8_t USBH_MscRead( uint32_t lba, uint8_t *buf, uint32_t count )
{
	uint8_t  cdb[10];
	uint8_t  status = 0;
	uint8_t  s;
	uint32_t total_bytes = count * msc_block_size;

	cdb[0] = SCSI_READ_10;
	cdb[1] = 0;
	cdb[2] = (uint8_t)( (lba >> 24) & 0xFF );
	cdb[3] = (uint8_t)( (lba >> 16) & 0xFF );
	cdb[4] = (uint8_t)( (lba >>  8) & 0xFF );
	cdb[5] = (uint8_t)(  lba        & 0xFF );
	cdb[6] = 0;
	cdb[7] = (uint8_t)( (count >> 8) & 0xFF );
	cdb[8] = (uint8_t)(  count       & 0xFF );
	cdb[9] = 0;

	for( uint8_t attempt = 0; attempt < USBH_MSC_BOT_MAX_TRIES; attempt++ )
	{
		msc_bulk_in_tog = 0;  // fresh toggle per attempt
		s = msc_send_cbw( cdb, 10, total_bytes, 0x80 );
		if( s != USBH_ERR_SUCCESS )
		{
			msc_clear_halts();
			continue;
		}

		// Drain 64-byte packets until we've read all the bytes.
		uint8_t *p = buf;
		uint32_t remaining = total_bytes;
		uint8_t err = 0;
		while( remaining > 0 )
		{
			Delay_Us( USBH_MSC_BOT_SETTLE_US );
			uint8_t rx = 0;
			s = USBH_BulkOrIntrIn( msc_bulk_in_ep, msc_bulk_in_tog,
				p, &rx, 2000 );
			if( s != USBH_ERR_SUCCESS ) { err = 1; break; }
			if( rx == 0 ) break;  // short packet = end of data
			p += rx;
			remaining -= rx;
		}
		if( err )
		{
			msc_clear_halts();
			continue;
		}

		s = msc_receive_csw( &status );
		if( s != USBH_ERR_SUCCESS ) { msc_clear_halts(); continue; }
		if( status != 0 ) { msc_clear_halts(); continue; }
		return USBH_ERR_SUCCESS;
	}
	return USBH_ERR_USB_MSC_CSW;
}

// Public: write `count` blocks.
uint8_t USBH_MscWrite( uint32_t lba, const uint8_t *buf, uint32_t count )
{
	uint8_t  cdb[10];
	uint8_t  status = 0;
	uint8_t  s;
	uint32_t total_bytes = count * msc_block_size;

	cdb[0] = SCSI_WRITE_10;
	cdb[1] = 0;
	cdb[2] = (uint8_t)( (lba >> 24) & 0xFF );
	cdb[3] = (uint8_t)( (lba >> 16) & 0xFF );
	cdb[4] = (uint8_t)( (lba >>  8) & 0xFF );
	cdb[5] = (uint8_t)(  lba        & 0xFF );
	cdb[6] = 0;
	cdb[7] = (uint8_t)( (count >> 8) & 0xFF );
	cdb[8] = (uint8_t)(  count       & 0xFF );
	cdb[9] = 0;

	for( uint8_t attempt = 0; attempt < USBH_MSC_BOT_MAX_TRIES; attempt++ )
	{
		msc_bulk_out_tog = 0;
		s = msc_send_cbw( cdb, 10, total_bytes, 0x00 );
		if( s != USBH_ERR_SUCCESS )
		{
			msc_clear_halts();
			continue;
		}

		const uint8_t *p = buf;
		uint32_t remaining = total_bytes;
		uint8_t err = 0;
		while( remaining > 0 )
		{
			Delay_Us( USBH_MSC_BOT_SETTLE_US );
			uint8_t chunk = remaining > 64 ? 64 : (uint8_t)remaining;
			s = USBH_BulkOrIntrOut( msc_bulk_out_ep, &msc_bulk_out_tog,
				p, chunk, 2000 );
			if( s != USBH_ERR_SUCCESS ) { err = 1; break; }
			p += chunk;
			remaining -= chunk;
		}
		if( err )
		{
			msc_clear_halts();
			continue;
		}

		s = msc_receive_csw( &status );
		if( s != USBH_ERR_SUCCESS ) { msc_clear_halts(); continue; }
		if( status != 0 ) { msc_clear_halts(); continue; }
		return USBH_ERR_SUCCESS;
	}
	return USBH_ERR_USB_MSC_CSW;
}
