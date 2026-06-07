// usbhost_xfer.h
// Phase B: USB transfer primitives on the CH32V203 USBFS host SIE.

#ifndef USBHOST_XFER_H
#define USBHOST_XFER_H

#include <stdint.h>
#include "usbhost_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// One-token transaction: issue SETUP / IN / OUT on endpoint 0, with the
// requested DATA-toggle, polling the SIE for up to `timeout_us` microseconds
// per retry. Retries as long as the device returns NAK.
//
// Returns one of:
//   USBH_ERR_SUCCESS
//   USBH_ERR_USB_TRANSFER (NAK timeout, STALL, or toggle error)
//   USBH_ERR_USB_UNKNOWN  (SIE never set the transfer-done flag)
uint8_t USBH_Transact( uint8_t ep_pid, uint8_t ep_tog, uint16_t timeout_us );

// Three-stage control transfer. Caller pre-fills a SETUP packet at
// USBH_TxBuf[0..7] (the same buffer the SIE will use for the OUT phase).
//
//   len == 0   → no DATA phase (e.g. SET_ADDRESS)
//   dir_in     → DATA stage is IN,   STATUS stage is OUT zero-length
//   !dir_in    → DATA stage is OUT,  STATUS stage is IN  zero-length
//
// On success, *rx_len holds the number of bytes the device sent (IN) or
// was accepted (OUT).
uint8_t USBH_CtrlXfer( const USBH_SetupReq *setup, uint8_t *buf,
                       uint16_t req_len, uint8_t ep0_size, uint16_t *rx_len );

// ---------------------------------------------------------------------------
// Bulk / Interrupt transfer primitives (Phase D).
//
// These wrap USBH_Transact for non-control endpoints.  Caller passes the
// 4-bit endpoint number (1..15) and a pointer to a per-endpoint DATA
// toggle byte (the SIE needs a separate toggle state per direction and
// per endpoint; the class driver owns the storage).
//
// `len` is the maximum packet size for the endpoint (≤ the SIE's
// HOST_FIFO size = 64 bytes for full-speed bulk / interrupt). For
// full-speed bulk IN, a single call reads at most one packet — the
// caller loops and reassembles. (Some flash drives happily send
// 64-byte back-to-back packets; others need a short-packet ZLP to
// terminate.)
//
// For OUT, the SIE auto-toggles internally; the *ptog value is the
// expected toggle the device should see, and is *not* updated (the SIE
// flips it for the next call). For IN, the caller passes the expected
// toggle, and on success the SIE has *not* auto-toggled (host side
// tracks the expected toggle for IN); the caller updates *ptog to
// (TOG_OK ? tog^1 : tog) before the next call.
// ---------------------------------------------------------------------------

// Issue one bulk / interrupt OUT token. ep_num is 1..15. The byte at
// *ptog is the data toggle (0 = DATA0, 1 = DATA1) and is *not* updated
// — the SIE handles auto-toggle for OUT tokens.
//
// Returns USBH_ERR_SUCCESS on ACK, USBH_PID_STALL | USBH_ERR_USB_TRANSFER
// on a stall, USBH_ERR_USB_TRANSFER on retry exhaustion / timeout.
uint8_t USBH_BulkOrIntrOut( uint8_t ep_num, uint8_t *ptog,
                            const uint8_t *buf, uint8_t len,
                            uint16_t timeout_us );

// Issue one bulk / interrupt IN token. tog is the data toggle the
// caller expects. *prx_len is filled with the number of bytes actually
// received (≤ 64 for full-speed). The caller must update its toggle
// state to (tog ^ 1) on a successful TOG_OK reply.
uint8_t USBH_BulkOrIntrIn( uint8_t ep_num, uint8_t tog,
                           uint8_t *buf, uint8_t *prx_len,
                           uint16_t timeout_us );

// Issue CLEAR_FEATURE(ENDPOINT_HALT) on EP0 for the given non-zero
// endpoint. Used by class drivers to recover from a STALL handshake
// (most flash drives / hubs STALL an endpoint on a protocol error,
// and the host must explicitly clear the halt before issuing the next
// token). `ep` is the *full* endpoint address (0x81 for IN1, 0x02 for
// OUT2). Pass the matching wValue (0 = ENDPOINT_HALT).
uint8_t USBH_ClrStall( uint8_t ep_addr, uint8_t ep0_size );

// Two 64-byte scratch buffers, exported so the caller can place a SETUP
// packet at USBH_TxBuf[0..7] before calling USBH_CtrlXfer().
extern uint8_t USBH_TxBuf[];
extern uint8_t USBH_RxBuf[];

#ifdef __cplusplus
}
#endif

#endif // USBHOST_XFER_H
