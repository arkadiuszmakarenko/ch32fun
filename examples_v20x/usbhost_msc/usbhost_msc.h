// usbhost_msc.h
//
// USB Mass Storage Class driver (Bulk-Only Transport) on top of the
// ch32fun USB host primitives.
//
// This is a *thin* BOT layer. It issues SCSI CDBs and shuttles CBW /
// data / CSW packets through the bulk IN/OUT endpoints, but it does
// NOT speak the FAT file system. (See pff/diskio.c for the PFF hook.)
//
// All public functions are blocking and single-threaded. The class
// driver must be called from main() context.

#ifndef USBHOST_MSC_H
#define USBHOST_MSC_H

#include <stdint.h>
#include "usbhost_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Wire up the MSC driver: scan the configuration descriptor for the
// first Mass Storage interface, issue GET_MAX_LUN, do a BOT
// Mass-Storage-Reset class request, and confirm TEST_UNIT_READY.
//
// On success, fills *pin_ep / *pout_ep with the bulk endpoint numbers
// (low 4 bits, no direction bit) and *pep0_size with the bMaxPacketSize0
// from the device descriptor (needed for control requests during
// error recovery).
//
// ep0_size_in is the same ep0 size we used for enumeration; it's
// cached here so diskio.c can issue CLEAR_FEATURE on its own.
uint8_t USBH_MscInit( const uint8_t *cfg_desc, uint16_t cfg_len,
	uint8_t ep0_size_in,
	uint8_t *pin_ep,
	uint16_t *pin_maxp,
	uint8_t *pout_ep,
	uint16_t *pout_maxp,
	uint32_t *plast_lba,
	uint32_t *pblock_size );

// Read `count` 512-byte blocks starting at `lba` into `buf` (which
// must be 32-bit aligned). Returns USBH_ERR_SUCCESS on success, or
// one of the standard error codes.
uint8_t USBH_MscRead(  uint32_t lba, uint8_t *buf, uint32_t count );

// Write `count` 512-byte blocks from `buf` to `lba`. Returns
// USBH_ERR_SUCCESS on success.
uint8_t USBH_MscWrite( uint32_t lba, const uint8_t *buf, uint32_t count );

// Called by diskio.c / main() to bring up the BOT layer after the
// device has been enumerated and SET_CONFIGURATION has run. The MSC
// state is cached inside the module. Returns USBH_ERR_SUCCESS on
// success.
uint8_t USBH_MscAttach( const uint8_t *cfg_desc, uint16_t cfg_len,
	uint8_t ep0_size );

// True if a mass-storage device is currently attached and the BOT
// layer has been successfully initialised.
uint8_t USBH_MscIsPresent( void );

// Block size is always 512 for a USB flash drive; some legacy ZIP
// drives were 512 anyway, so we don't expose this as a knob.
#define USBH_MSC_BLOCK_SIZE   512

#ifdef __cplusplus
}
#endif

#endif // USBHOST_MSC_H
