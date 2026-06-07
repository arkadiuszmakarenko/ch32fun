// usbhost_std.h
// Phase C: standard USB request helpers.

#ifndef USBHOST_STD_H
#define USBHOST_STD_H

#include <stdint.h>
#include "usbhost_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Issues GET_DESCRIPTOR(DEVICE) twice: first 8 bytes to learn
// bMaxPacketSize0, then the full 18-byte descriptor. *pep0_size receives
// bMaxPacketSize0; pbuf must be at least 18 bytes.
uint8_t USBH_GetDeviceDesc( uint8_t *pep0_size, uint8_t *pbuf );

// Issues GET_DESCRIPTOR(CONFIGURATION) twice: first 4 bytes for the
// header, then the full descriptor. *pcfg_len receives the actual byte
// count read.
uint8_t USBH_GetConfigDesc( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len );

// SET_ADDRESS. No data stage. Caller must wait at least 2 ms (per USB
// 2.0 spec) before issuing the next request on the new address.
uint8_t USBH_SetAddress( uint8_t ep0_size, uint8_t addr );

// SET_CONFIGURATION. No data stage.
uint8_t USBH_SetConfig( uint8_t ep0_size, uint8_t cfg );

#ifdef __cplusplus
}
#endif

#endif // USBHOST_STD_H
