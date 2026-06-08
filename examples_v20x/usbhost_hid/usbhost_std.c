// usbhost_std.c
//
// Phase C: the four standard USB requests we need to enumerate a device.
// Each function builds a SETUP packet in USBH_TxBuf, calls USBH_CtrlXfer,
// and copies/extracts data as appropriate.

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_xfer.h"
#include "usbhost_std.h"
#include "usbhost_defs.h"
#include "usbhost_hw.h"

// ---------------------------------------------------------------------------
// USBH_SetupWrite
//
// Helper: write a little-endian 16-bit value to USBH_TxBuf[off].
// (We don't depend on memcpy to do this because the SETUP packet is
// always pre-filled by us; we know the chip is little-endian, so a
// direct assignment of the host-order value is wrong — we need to
// byte-swap.)
// ---------------------------------------------------------------------------
static inline void USBH_Put16( uint8_t *p, uint16_t v )
{
	p[0] = (uint8_t)( v       & 0xFFu );
	p[1] = (uint8_t)( (v >> 8) & 0xFFu );
}

// ---------------------------------------------------------------------------
// USBH_GetDeviceDesc
//
// Issues GET_DESCRIPTOR(DEVICE). Per USB 2.0, the very first GET_DESCRIPTOR
// must be exactly 8 bytes long so the device can learn what the host's
// EP0 packet size is via bMaxPacketSize0. After we read those 8 bytes we
// immediately re-issue with wLength=18 to get the full descriptor.
//
//   *pep0_size  - out: bMaxPacketSize0 from the device
//   pbuf        - out: pointer to the *full* 18-byte descriptor
//
// We do not re-read the 8-byte version into pbuf — the caller only cares
// about the full descriptor and bMaxPacketSize0.
// ---------------------------------------------------------------------------
uint8_t USBH_GetDeviceDesc( uint8_t *pep0_size, uint8_t *pbuf )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	// First call: 8 bytes, just to learn bMaxPacketSize0.
	req.bmRequestType = USBH_REQ_GET_DEVICE_DESC_BYTE;
	req.bRequest       = USBH_REQ_GET_DESCRIPTOR;
	USBH_Put16( (uint8_t*)&req.wValue, ( USBH_DESC_DEVICE << 8 ) );
	req.wIndex         = 0;
	USBH_Put16( (uint8_t*)&req.wLength, 8 );

	{
		uint8_t tmp[8];
		s = USBH_CtrlXfer( &req, tmp, 8, 8, &got );
		if( s != USBH_ERR_SUCCESS ) {
			printf( "GetDevDesc(8) xfer=%02x\n", (unsigned)s );
			return USBH_ERR_DEV_DESCR_FAIL;
		}
		if( got < 8 ) { printf( "GetDevDesc(8) short got=%u\n", (unsigned)got ); return USBH_ERR_DEV_DESCR_FAIL; }
		*pep0_size = tmp[7];  // bMaxPacketSize0
	}

	// Second call: full 18-byte descriptor, using the discovered ep0 size.
	USBH_Put16( (uint8_t*)&req.wLength, 18 );
	s = USBH_CtrlXfer( &req, pbuf, 18, *pep0_size, &got );
	if( s != USBH_ERR_SUCCESS ) {
		printf( "GetDevDesc(18) xfer=%02x\n", (unsigned)s );
		return USBH_ERR_DEV_DESCR_FAIL;
	}
	if( got < 18 ) { printf( "GetDevDesc(18) short got=%u\n", (unsigned)got ); return USBH_ERR_DEV_DESCR_FAIL; }
	return USBH_ERR_SUCCESS;
}

// ---------------------------------------------------------------------------
// USBH_GetConfigDesc
//
// Reads the first 4 bytes of the configuration descriptor (header), then
// re-issues with the total length from wTotalLength.
//
//   ep0_size - bMaxPacketSize0 (used for the data stage)
//   pbuf     - out: full config descriptor (up to buf_len bytes)
//   buf_len  - capacity of pbuf
//   pcfg_len - out: actual number of bytes read
// ---------------------------------------------------------------------------
uint8_t USBH_GetConfigDesc( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	req.bmRequestType = USBH_REQ_GET_DEVICE_DESC_BYTE;
	req.bRequest       = USBH_REQ_GET_DESCRIPTOR;
	USBH_Put16( (uint8_t*)&req.wValue, ( USBH_DESC_CONFIGURATION << 8 ) );
	req.wIndex         = 0;
	USBH_Put16( (uint8_t*)&req.wLength, 4 );

	s = USBH_CtrlXfer( &req, pbuf, 4, ep0_size, &got );
	if( s != USBH_ERR_SUCCESS ) return USBH_ERR_CFG_DESCR_FAIL;
	if( got < 4 ) return USBH_ERR_CFG_DESCR_FAIL;

	uint16_t total = (uint16_t)pbuf[2] | ( (uint16_t)pbuf[3] << 8 );
	if( total > buf_len ) total = buf_len;
	if( total < 4 )       total = 4;

	// If wTotalLength is exactly 4 we already have everything.
	if( total == 4 ) { *pcfg_len = 4; return USBH_ERR_SUCCESS; }

	// Re-issue the request for the full descriptor. We must zero the
	// first 4 bytes back at offset 0 (some devices don't re-send them
	// if the data stage returns a shorter packet on the second read).
	USBH_Put16( (uint8_t*)&req.wLength, total );
	s = USBH_CtrlXfer( &req, pbuf, total, ep0_size, &got );
	if( s != USBH_ERR_SUCCESS ) return USBH_ERR_CFG_DESCR_FAIL;

	*pcfg_len = ( got < total ) ? got : total;
	return USBH_ERR_SUCCESS;
}

// ---------------------------------------------------------------------------
// USBH_SetAddress
//
// Standard SET_ADDRESS. The device's status stage is an IN ZLP; on success
// the device will only respond to the new address for the *next* token.
// The USB 2.0 spec requires the host to wait at least 2 ms (a SET_ADDRESS
// recovery interval) before issuing the next request to the new address;
// the caller handles that.
// ---------------------------------------------------------------------------
uint8_t USBH_SetAddress( uint8_t ep0_size, uint8_t addr )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	req.bmRequestType = USBH_REQ_SET_ADDRESS_BYTE;
	req.bRequest       = USBH_REQ_SET_ADDRESS;
	USBH_Put16( (uint8_t*)&req.wValue, addr );
	req.wIndex         = 0;
	USBH_Put16( (uint8_t*)&req.wLength, 0 );

	s = USBH_CtrlXfer( &req, NULL, 0, ep0_size, &got );
	if( s != USBH_ERR_SUCCESS ) return USBH_ERR_ADDR_SET_FAIL;
	return USBH_ERR_SUCCESS;
}

// ---------------------------------------------------------------------------
// USBH_SetConfig
//
// SET_CONFIGURATION. No data stage.
// ---------------------------------------------------------------------------
uint8_t USBH_SetConfig( uint8_t ep0_size, uint8_t cfg )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	req.bmRequestType = USBH_REQ_SET_CONFIGURATION_BYTE;
	req.bRequest       = USBH_REQ_SET_CONFIGURATION;
	USBH_Put16( (uint8_t*)&req.wValue, cfg );
	req.wIndex         = 0;
	USBH_Put16( (uint8_t*)&req.wLength, 0 );

	s = USBH_CtrlXfer( &req, NULL, 0, ep0_size, &got );
	return s;
}

// ---------------------------------------------------------------------------
// USBH_GetHidReportDesc
//
// GET_DESCRIPTOR(HID_REPORT) for the given interface. Per USB HID 1.11
// §4.2 this is a class-specific GET_DESCRIPTOR, device-to-host,
// recipient = interface, descriptor type = 0x22.
//
// We do the two-step read (just like GetConfigDesc): if the caller
// passes a *plen that's larger than what the device's first chunk
// reveals, we re-issue. In practice the descriptor is short enough
// (typically < 256 bytes for mice/joysticks, occasionally up to ~1 KB
// for gaming keyboards) that the first call almost always returns the
// whole thing.
//
//   ep0_size  - bMaxPacketSize0
//   iface     - bInterfaceNumber of the HID interface
//   pbuf      - caller-supplied buffer
//   plen      - in: buffer capacity; out: actual bytes read
// ---------------------------------------------------------------------------
uint8_t USBH_GetHidReportDesc( uint8_t ep0_size, uint8_t iface,
                               uint8_t *pbuf, uint16_t *plen )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	if( pbuf == NULL || plen == NULL || *plen == 0 ) {
		return USBH_ERR_USB_UNKNOWN;
	}

	req.bmRequestType = 0x81u;  // device-to-host, standard, interface
	req.bRequest       = USBH_REQ_GET_DESCRIPTOR;
	USBH_Put16( (uint8_t*)&req.wValue, ( USBH_DESC_HID_REPORT << 8 ) );
	USBH_Put16( (uint8_t*)&req.wIndex, iface );
	USBH_Put16( (uint8_t*)&req.wLength, *plen );

	s = USBH_CtrlXfer( &req, pbuf, *plen, ep0_size, &got );
	if( s != USBH_ERR_SUCCESS ) return s;

	*plen = got;
	return USBH_ERR_SUCCESS;
}
