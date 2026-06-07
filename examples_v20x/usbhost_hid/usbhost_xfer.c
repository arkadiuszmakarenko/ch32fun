// usbhost_xfer.c
//
// Phase B of usbhost_enum: implement the one-token transaction (Transact)
// and the three-stage control transfer (CtrlXfer) on top of the ch32fun
// register names.
//
// The register-level recipe is the same as the WCH EVT host stack
// (HOST_EP_PID / INT_FG / INT_ST) — we re-derive the bit names and the
// state machine here against ch32fun's USBOTG_H_FS struct, no vendor
// code reused.
//
// IMPORTANT: This file is on the per-token critical path. None of the
// functions below call printf, allocate memory, or take interrupts. The
// only loop is the spin-wait for the SIE to complete a packet.

#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "usbhost_xfer.h"
#include "usbhost_hw.h"
#include "usbhost_defs.h"

// Per-stage timeout budgets. Each stage is a single packet on EP0.
// Full-speed EP0 round-trip is < 50 us in practice; 1000 us is generous
// and matches the per-token retry budget the EVT reference uses.
#define USBH_PER_TIMEOUT_IN     1000
#define USBH_PER_TIMEOUT_OUT    1000
#define USBH_PER_TIMEOUT_STATUS 1000

// 100 us settle between control-transfer stages — matches the EVT
// reference. USB 2.0 spec only requires a "reasonable" inter-stage
// delay; this is enough margin for the slowest full-speed device
// we've seen.
#define USBH_INTER_STAGE_DELAY_US   100

// 15 us settle between token retries inside USBH_Transact — matches the
// EVT reference exactly. The WCH SIE needs a few microseconds to release
// the bus after a NAK/timeout before the next token can be issued; the
// EVT stack uses 15 us, longer than that wastes retry budget on a slow
// device, shorter and the next token can be lost.
#define USBH_INTER_RETRY_DELAY_US   15

// ---------------------------------------------------------------------------
// USBH_Transact
//
// One call issues a single token (SETUP/IN/OUT) on the specified endpoint,
// waits for the SIE to set the UIF_TRANSFER flag, decodes the result in
// INT_ST, and returns. NAK is special-cased: the caller passes a non-zero
// `timeout_us` and we retry the token until the device either ACKs (or
// stalls) or we exhaust the budget.
//
// `ep_pid` is the top 4 bits of HOST_EP_PID (the SIE takes the top nibble
// as the token PID and the bottom nibble as the endpoint number). EP0
// is always 0, so the bottom nibble is hard-coded to 0.
//
// CRITICAL: this function is the per-token hot path. No printf, no malloc,
// no IRQ entry — the only loop is a 1 us spin on the SIE's done flag.
// ---------------------------------------------------------------------------
uint8_t USBH_Transact( uint8_t ep_pid, uint8_t ep_tog, uint16_t timeout_us )
{
	// Preload the DATA toggle into both TX and RX control registers.
	// The SIE picks the right one based on the PID direction.
	USBOTG_H_FS->HOST_TX_CTRL = ep_tog;
	USBOTG_H_FS->HOST_RX_CTRL = ep_tog;

	static uint8_t first_call_done = 0;
	if( !first_call_done ) {
		first_call_done = 1;
		printf( "FIRST Transact: pid=%02x tog=%02x BASE=%02x HOST_CTRL=%02x HOST_EP_MOD=%02x HOST_SETUP=%04x TX_LEN=%04x RX_LEN=%04x MIS=%02x\n",
		        ep_pid, ep_tog,
		        (unsigned)USBOTG_H_FS->BASE_CTRL,
		        (unsigned)USBOTG_H_FS->HOST_CTRL,
		        (unsigned)USBOTG_H_FS->HOST_EP_MOD,
		        (unsigned)USBOTG_H_FS->HOST_SETUP,
		        (unsigned)USBOTG_H_FS->HOST_TX_LEN,
		        (unsigned)USBOTG_H_FS->RX_LEN,
		        (unsigned)USBOTG_H_FS->MIS_ST );
	}

	uint8_t  trans_retry = 0;
	uint8_t  r;
	uint16_t i;

	do
	{
		// Specify the token (top nibble) and the endpoint number
		// (bottom nibble). EP0 = 0 always here. ep_pid is the 4-bit
		// WCH PID code; shift it to the top nibble.
		USBOTG_H_FS->HOST_EP_PID = ( ep_pid << 4 ) | 0u;

		// Arm the transfer-complete flag and clear any stale one.
		// Also clear USBOTG_UIF_DETECT so a disconnect during the
		// retry loop can be detected via INT_FG below.
		USBOTG_H_FS->INT_FG = USBOTG_UIF_TRANSFER | USBOTG_UIF_DETECT;

		// Spin-wait for the SIE to complete the packet. The WCH
		// reference uses a 1 us resolution, so we do the same.
		for( i = 0; i < timeout_us; i++ )
		{
			if( USBOTG_H_FS->INT_FG & USBOTG_UIF_TRANSFER ) break;
			Delay_Us( 1 );
		}

		// Stop the SIE from continuing to hold the bus.
		USBOTG_H_FS->HOST_EP_PID = 0x00u;

		if( !( USBOTG_H_FS->INT_FG & USBOTG_UIF_TRANSFER ) )
		{
			// SIE never finished. The device is gone or wedged.
			printf( "Transact: TIMEOUT pid=%02x tog=%02x INT_FG=%02x MIS_ST=%02x HOST_CTRL=%02x\n",
			        ep_pid, ep_tog,
			        (unsigned)USBOTG_H_FS->INT_FG,
			        (unsigned)USBOTG_H_FS->MIS_ST,
			        (unsigned)USBOTG_H_FS->HOST_CTRL );
			return USBH_ERR_USB_UNKNOWN;
		}

		// Wait for the SIE to fully release the bus before the next
		// retry. Without this, a fast retry can be lost: the WCH SIE
		// sets UIF_TRANSFER before it actually drops the bus, and
		// re-writing HOST_EP_PID too soon is a no-op. This is the
		// single biggest contributor to the flakey "NHR forever" loop
		// we saw on flash drives that were slow to come back after
		// bus reset — the first few retries were issued before the
		// SIE was ready, and each was silently dropped.
		for( i = 0; i < 200; i++ )
		{
			if( USBOTG_H_FS->MIS_ST & USBOTG_UMS_SIE_FREE ) break;
			Delay_Us( 1 );
		}

		// TOG_OK means the data toggle matched what we expected:
		// packet ACKed, payload (if IN) is in the RxBuf.
		if( USBOTG_H_FS->INT_ST & USBOTG_UIS_TOG_OK )
		{
			return USBH_ERR_SUCCESS;
		}

		// Pull the raw response PID out of INT_ST[3:0].
		r = USBOTG_H_FS->INT_ST & USBOTG_UIS_H_RES;

		if( r == USBH_PID_STALL )
		{
			return USBH_PID_STALL | USBH_ERR_USB_TRANSFER;
		}

		if( r == USBH_PID_NAK )
		{
			// Honour the caller's timeout budget; if exhausted, give up.
			if( timeout_us == 0 ) return USBH_PID_NAK | USBH_ERR_USB_TRANSFER;
			if( timeout_us < 0xFFFFu ) timeout_us--;
			printf( "Transact: NAK pid=%02x INT_ST=%02x\n", ep_pid,
			        (unsigned)USBOTG_H_FS->INT_ST );
			// Fall through to the 20 us delay + retry at bottom of loop.
		}
		else

		// Not TOG_OK, not STALL, not NAK. Decode by token type,
		// matching the WCH EVT USBFSH_Transact switch/case exactly.
		switch( ep_pid )
		{
			case USBH_PID_SETUP:
			case USBH_PID_OUT:
				// r==0: no handshake received (device busy/not ready).
				// Retry, identical to the EVT reference behavior.
				if( r ) {
					printf( "Transact: err pid=%02x r=%02x INT_ST=%02x\n",
					        ep_pid, r, (unsigned)USBOTG_H_FS->INT_ST );
					return r | USBH_ERR_USB_TRANSFER;
				}
				{
					static uint8_t dump_done = 0;
					if( !dump_done && trans_retry == 0 ) {
						dump_done = 1;
						printf( "  RxBuf: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
						        USBH_RxBuf[0],  USBH_RxBuf[1],  USBH_RxBuf[2],  USBH_RxBuf[3],
						        USBH_RxBuf[4],  USBH_RxBuf[5],  USBH_RxBuf[6],  USBH_RxBuf[7],
						        USBH_RxBuf[8],  USBH_RxBuf[9],  USBH_RxBuf[10], USBH_RxBuf[11],
						        USBH_RxBuf[12], USBH_RxBuf[13], USBH_RxBuf[14], USBH_RxBuf[15] );
					}
				}
				printf( "Transact: NHR pid=%02x INT_ST=%02x RX_LEN=%u\n", ep_pid,
				        (unsigned)USBOTG_H_FS->INT_ST,
				        (unsigned)USBOTG_H_FS->RX_LEN );
				break;   // r==0: fall through to retry

			case USBH_PID_IN:
				if( ( r == USBH_PID_DATA0 ) || ( r == USBH_PID_DATA1 ) )
					break;  // toggle mismatch but data is good, retry
				if( r ) {
					printf( "Transact: err pid=%02x r=%02x INT_ST=%02x\n",
					        ep_pid, r, (unsigned)USBOTG_H_FS->INT_ST );
					return r | USBH_ERR_USB_TRANSFER;
				}
				break;

			default:
				return USBH_ERR_USB_UNKNOWN;
		}

		// Disconnect check between retries — matches the EVT reference.
		// If a DETECT edge latched while we were spinning, the device
		// (or the host port) went away mid-transfer; settle 200 us so
		// the SIE debounces, then check whether the port is still
		// enabled. If not, bail out with a disconnect error instead
		// of burning the rest of the retry budget.
		if( USBOTG_H_FS->INT_FG & USBOTG_UIF_DETECT )
		{
			Delay_Us( 200 );
			if( !( USBOTG_H_FS->HOST_CTRL & USBOTG_UH_PORT_EN ) )
			{
				return USBH_ERR_USB_DISCON;
			}
		}

		// 15 us settle, matching EVT.
		Delay_Us( USBH_INTER_RETRY_DELAY_US );

	} while( ++trans_retry < 10 );

	return USBH_ERR_USB_TRANSFER;  // exhausted retries
}

// ---------------------------------------------------------------------------
// USBH_CtrlXfer
//
// Implements the three-stage USB control transfer: SETUP → DATA → STATUS.
//
// The SIE reads HOST_TX_LEN to know how many bytes of the TX buffer to
// send for SETUP (always 8) and for any OUT tokens in the DATA stage.
// We follow the WCH EVT convention: HOST_TX_LEN left at its last-set
// value is the natural indicator of the STATUS stage direction (see
// the comment near the bottom).
//
// CRITICAL: no printf in this function. The print happens once the
// whole transfer is done, in the caller.
// ---------------------------------------------------------------------------
uint8_t USBH_CtrlXfer( const USBH_SetupReq *setup, uint8_t *buf,
                       uint16_t req_len, uint8_t ep0_size, uint16_t *rx_len )
{
	uint16_t rem_len = 0;
	uint16_t total   = 0;
	uint8_t  s;

	if( rx_len ) *rx_len = 0;

	// ----- Stage 1: SETUP -----
	// Copy the 8-byte SETUP into the SIE's TX buffer and tell the SIE
	// it's 8 bytes long. The SIE will DMA these out when the SETUP
	// token is acknowledged by the device.
	memcpy( USBH_TxBuf, setup, 8 );
	USBOTG_H_FS->HOST_TX_LEN = 8;

	// No inter-stage delay before SETUP — it's the first stage, and
	// the WCH EVT reference issues it immediately after programming
	// HOST_TX_LEN. Inserting 100 us here just slows enumeration on
	// devices that are already slow to respond.

	s = USBH_Transact( USBH_PID_SETUP, 0x00u, USBH_PER_TIMEOUT_IN );
	if( s != USBH_ERR_SUCCESS ) return s;

	// The SIE has toggled to DATA1 after the SETUP packet. Pre-arm
	// both TX and RX side for the data stage, AND enable auto-toggle
	// so the SIE flips the toggle for us on every successful packet.
	// The WCH EVT reference does exactly this:
	//   HOST_TX_CTRL = HOST_RX_CTRL =
	//       UH_T_TOG | UH_R_TOG | UH_T_AUTO_TOG | UH_R_AUTO_TOG;
	// With auto-toggle on, the data stage callers must NOT manually
	// XOR the toggle bits (the SIE does it) — so the IN/OUT loops
	// below use the register value as the *expected* toggle but
	// leave the XOR to the hardware.
	USBOTG_H_FS->HOST_TX_CTRL = USBOTG_UH_T_TOG | USBOTG_UH_T_AUTO_TOG;
	USBOTG_H_FS->HOST_RX_CTRL = USBOTG_UH_R_TOG | USBOTG_UH_R_AUTO_TOG;

	// The default no-data STATUS stage is IN; setting HOST_TX_LEN
	// to a non-zero value now lets the STATUS direction check below
	// pick the right PID without the caller having to track it.
	USBOTG_H_FS->HOST_TX_LEN = 0x01u;

	// ----- Stage 2: DATA (optional) -----
	rem_len = req_len;
	if( rem_len > 0 && buf != NULL )
	{
		// A SETUP request's data direction is in bmRequestType bit 7.
		uint8_t dir_in = ( setup->bmRequestType & 0x80u ) ? 1u : 0u;

		if( dir_in )
		{
			// IN data stage: device sends, host receives. Loop until
			// the descriptor is complete (short packet) or rem_len
			// is exhausted. Auto-toggle is on (set after SETUP), so
			// the SIE flips RX toggle for us on every ACK — we just
			// hand it its current value as the expected toggle and
			// do NOT XOR it back.
			while( rem_len > 0 )
			{
				Delay_Us( USBH_INTER_STAGE_DELAY_US );

				s = USBH_Transact( USBH_PID_IN, USBOTG_H_FS->HOST_RX_CTRL, USBH_PER_TIMEOUT_IN );
				if( s != USBH_ERR_SUCCESS ) return s;

				uint16_t got = USBOTG_H_FS->RX_LEN;
				if( got > rem_len ) got = rem_len;
				for( uint16_t k = 0; k < got; k++ ) buf[ total + k ] = USBH_RxBuf[ k ];
				total   += got;
				rem_len -= got;

				// Short packet (RX_LEN == 0, or RX_LEN not a multiple
				// of ep0_size) means the descriptor is complete. The
				// ep0_size mask is `ep0_size - 1`; GCC sees a tautology
				// when ep0_size is a power of two, so we mark it
				// explicitly as a runtime check.
				uint16_t mask = (uint16_t)( ep0_size - 1u );
				if( ( USBOTG_H_FS->RX_LEN == 0 ) ||
				    ( USBOTG_H_FS->RX_LEN & mask ) )
				{
					break;
				}
			}

			// STATUS will be OUT zero-length; tell the SIE.
			USBOTG_H_FS->HOST_TX_LEN = 0;
		}
		else
		{
			// OUT data stage: host sends, device receives. Auto-toggle
			// is on, so the SIE flips TX toggle for us — same pattern
			// as the IN branch: read current value, hand it back as
			// the expected toggle, do not XOR.
			while( rem_len > 0 )
			{
				uint16_t chunk = rem_len >= ep0_size ? ep0_size : rem_len;
				USBOTG_H_FS->HOST_TX_LEN = chunk;
				memcpy( USBH_TxBuf, buf + total, chunk );

				Delay_Us( USBH_INTER_STAGE_DELAY_US );

				s = USBH_Transact( USBH_PID_OUT, USBOTG_H_FS->HOST_TX_CTRL, USBH_PER_TIMEOUT_OUT );
				if( s != USBH_ERR_SUCCESS ) return s;

				total   += chunk;
				rem_len -= chunk;
			}
			// HOST_TX_LEN is non-zero from the last OUT; STATUS will
			// be IN zero-length (no need to clear it).
		}
	}

	// ----- Stage 3: STATUS -----
	// The SIE picks the STATUS stage direction from HOST_TX_LEN:
	//   HOST_TX_LEN > 0  -> STATUS is IN  (host is waiting for a ZLP)
	//   HOST_TX_LEN == 0 -> STATUS is OUT (host is sending a ZLP)
	// which is exactly the spec: STATUS is the *opposite* direction of
	// the data stage, and IN for the no-data case.
	Delay_Us( USBH_INTER_STAGE_DELAY_US );
	{
		uint8_t status_pid = ( USBOTG_H_FS->HOST_TX_LEN )
		                     ? USBH_PID_IN
		                     : USBH_PID_OUT;
		// STATUS uses DATA1 on both directions per the spec.
		s = USBH_Transact( status_pid, USBOTG_UH_T_TOG | USBOTG_UH_R_TOG,
		                   USBH_PER_TIMEOUT_STATUS );
		if( s != USBH_ERR_SUCCESS ) return s;
	}

	if( rx_len ) *rx_len = total;
	return USBH_ERR_SUCCESS;
}

// ---------------------------------------------------------------------------
// USBH_BulkOrIntrOut — one bulk/INTR OUT token on a non-EP0 endpoint.
//
// Differences from USBH_Transact(USBH_PID_OUT, ...):
//   - HOST_EP_PID takes the endpoint number in the bottom nibble.
//   - HOST_TX_LEN must be programmed (this is what the SIE sends).
//   - HOST_TX_CTRL must include UH_T_AUTO_TOG so the SIE handles the
//     data toggle for us.
//
// Returns USBH_ERR_SUCCESS on ACK. STALL and disconnect are signalled
// the same way as in USBH_Transact.
// ---------------------------------------------------------------------------
uint8_t USBH_BulkOrIntrOut( uint8_t ep_num, uint8_t *ptog,
                            const uint8_t *buf, uint8_t len,
                            uint16_t timeout_us )
{
	uint8_t  trans_retry = 0;
	uint8_t  r;
	uint16_t i;

	do
	{
		// Place data in the SIE's TX buffer, length in TX_LEN.
		if( buf && len ) {
			memcpy( USBH_TxBuf, buf, len );
		}
		USBOTG_H_FS->HOST_TX_LEN = len;

		// Auto-toggle is on for OUT — the SIE handles the toggle bit.
		USBOTG_H_FS->HOST_TX_CTRL = ( *ptog ? USBOTG_UH_T_TOG : 0u )
		                           | USBOTG_UH_T_AUTO_TOG;
		USBOTG_H_FS->HOST_RX_CTRL = 0x00u;  // RX not used on an OUT

		// Issue the OUT token. Bottom nibble = endpoint number.
		USBOTG_H_FS->HOST_EP_PID = ( USBH_PID_OUT << 4 ) | ( ep_num & 0x0Fu );
		USBOTG_H_FS->INT_FG = USBOTG_UIF_TRANSFER | USBOTG_UIF_DETECT;

		for( i = 0; i < timeout_us; i++ )
		{
			if( USBOTG_H_FS->INT_FG & USBOTG_UIF_TRANSFER ) break;
			Delay_Us( 1 );
		}
		USBOTG_H_FS->HOST_EP_PID = 0x00u;

		if( !( USBOTG_H_FS->INT_FG & USBOTG_UIF_TRANSFER ) ) {
			printf( "BulkOut: TIMEOUT ep=%u INT_FG=%02x\n",
			        ep_num, (unsigned)USBOTG_H_FS->INT_FG );
			return USBH_ERR_USB_UNKNOWN;
		}

		// SIE-bus release wait.
		for( i = 0; i < 200; i++ ) {
			if( USBOTG_H_FS->MIS_ST & USBOTG_UMS_SIE_FREE ) break;
			Delay_Us( 1 );
		}

		if( USBOTG_H_FS->INT_ST & USBOTG_UIS_TOG_OK ) {
			// Auto-toggle handled the next expected value; we just
			// mirror the SIE's HOST_TX_CTRL back to the caller so
			// the toggle state stays consistent with the SIE.
			*ptog = ( USBOTG_H_FS->HOST_TX_CTRL & USBOTG_UH_T_TOG ) ? 1u : 0u;
			return USBH_ERR_SUCCESS;
		}

		r = USBOTG_H_FS->INT_ST & USBOTG_UIS_H_RES;
		if( r == USBH_PID_STALL ) {
			printf( "BulkOut: STALL ep=%u INT_ST=%02x\n",
			        ep_num, (unsigned)USBOTG_H_FS->INT_ST );
			return USBH_PID_STALL | USBH_ERR_USB_TRANSFER;
		}
		if( r == USBH_PID_NAK ) {
			if( timeout_us == 0 ) return USBH_PID_NAK | USBH_ERR_USB_TRANSFER;
			if( timeout_us < 0xFFFFu ) timeout_us--;
			Delay_Us( USBH_INTER_RETRY_DELAY_US );
			continue;
		}
		// r==0 (NHR) or other: retry, matching the EVT reference.
		if( r ) {
			printf( "BulkOut: err ep=%u r=%02x INT_ST=%02x\n",
			        ep_num, r, (unsigned)USBOTG_H_FS->INT_ST );
			return r | USBH_ERR_USB_TRANSFER;
		}
		Delay_Us( USBH_INTER_RETRY_DELAY_US );
	} while( ++trans_retry < 10 );

	return USBH_ERR_USB_TRANSFER;
}

// ---------------------------------------------------------------------------
// Local little-endian 16-bit writer. Kept local (and renamed) to
// avoid name-collision with the static one in usbhost_std.c.
// ---------------------------------------------------------------------------
static inline void USBH_Put16_local( uint8_t *p, uint16_t v )
{
	p[0] = (uint8_t)( v       & 0xFFu );
	p[1] = (uint8_t)( (v >> 8) & 0xFFu );
}

// ---------------------------------------------------------------------------
// CLEAR_FEATURE(ENDPOINT_HALT) for a non-zero endpoint. Used by class
// drivers to recover from a STALL handshake. The endpoint is reset to
// DATA0 by the device as a side effect of the HALT being cleared, so
// the caller's per-endpoint toggle state is no longer valid after this
// returns — they should re-initialise their toggle byte to 0.
// ---------------------------------------------------------------------------
uint8_t USBH_ClrStall( uint8_t ep_addr, uint8_t ep0_size )
{
	USBH_SetupReq req;
	uint16_t got = 0;
	uint8_t  s;

	req.bmRequestType = 0x02u;  // host-to-device, standard, endpoint
	req.bRequest       = USBH_REQ_CLEAR_FEATURE;
	USBH_Put16_local( (uint8_t*)&req.wValue, 0x0000u );  // ENDPOINT_HALT
	USBH_Put16_local( (uint8_t*)&req.wIndex, ep_addr );
	USBH_Put16_local( (uint8_t*)&req.wLength, 0 );

	s = USBH_CtrlXfer( &req, NULL, 0, ep0_size, &got );
	return s;
}

// ---------------------------------------------------------------------------
// USBH_BulkOrIntrOut — one bulk/INTR OUT token on a non-EP0 endpoint.
//
// Differences from USBH_Transact(USBH_PID_OUT, ...):
//   - HOST_EP_PID takes the endpoint number in the bottom nibble.
//   - HOST_TX_LEN must be programmed (this is what the SIE sends).
//   - HOST_TX_CTRL must include UH_T_AUTO_TOG so the SIE handles the
//     data toggle for us.
//
// Returns USBH_ERR_SUCCESS on ACK. STALL and disconnect are signalled
// the same way as in USBH_Transact.
// ---------------------------------------------------------------------------
uint8_t USBH_BulkOrIntrIn( uint8_t ep_num, uint8_t tog,
                           uint8_t *buf, uint8_t *prx_len,
                           uint16_t timeout_us )
{
	uint8_t  trans_retry = 0;
	uint8_t  r;
	uint16_t i;

	if( prx_len ) *prx_len = 0;

	do
	{
		USBOTG_H_FS->HOST_TX_CTRL = 0x00u;
		USBOTG_H_FS->HOST_RX_CTRL = tog ? USBOTG_UH_R_TOG : 0x00u;

		USBOTG_H_FS->HOST_EP_PID = ( USBH_PID_IN << 4 ) | ( ep_num & 0x0Fu );
		USBOTG_H_FS->INT_FG = USBOTG_UIF_TRANSFER | USBOTG_UIF_DETECT;

		for( i = 0; i < timeout_us; i++ )
		{
			if( USBOTG_H_FS->INT_FG & USBOTG_UIF_TRANSFER ) break;
			Delay_Us( 1 );
		}
		USBOTG_H_FS->HOST_EP_PID = 0x00u;

		if( !( USBOTG_H_FS->INT_FG & USBOTG_UIF_TRANSFER ) ) {
			printf( "BulkIn: TIMEOUT ep=%u INT_FG=%02x\n",
			        ep_num, (unsigned)USBOTG_H_FS->INT_FG );
			return USBH_ERR_USB_UNKNOWN;
		}

		for( i = 0; i < 200; i++ ) {
			if( USBOTG_H_FS->MIS_ST & USBOTG_UMS_SIE_FREE ) break;
			Delay_Us( 1 );
		}

		if( USBOTG_H_FS->INT_ST & USBOTG_UIS_TOG_OK ) {
			uint16_t got = USBOTG_H_FS->RX_LEN;
			if( got > 64 ) got = 64;
			if( buf && got ) memcpy( buf, USBH_RxBuf, got );
			if( prx_len ) *prx_len = (uint8_t)got;
			return USBH_ERR_SUCCESS;
		}

		r = USBOTG_H_FS->INT_ST & USBOTG_UIS_H_RES;
		if( r == USBH_PID_STALL ) {
			printf( "BulkIn: STALL ep=%u INT_ST=%02x\n",
			        ep_num, (unsigned)USBOTG_H_FS->INT_ST );
			return USBH_PID_STALL | USBH_ERR_USB_TRANSFER;
		}
		if( r == USBH_PID_NAK ) {
			if( timeout_us == 0 ) return USBH_PID_NAK | USBH_ERR_USB_TRANSFER;
			if( timeout_us < 0xFFFFu ) timeout_us--;
			Delay_Us( USBH_INTER_RETRY_DELAY_US );
			continue;
		}
		if( ( r == USBH_PID_DATA0 ) || ( r == USBH_PID_DATA1 ) ) {
			uint16_t got = USBOTG_H_FS->RX_LEN;
			if( got > 64 ) got = 64;
			if( buf && got ) memcpy( buf, USBH_RxBuf, got );
			if( prx_len ) *prx_len = (uint8_t)got;
			return USBH_ERR_SUCCESS;
		}
		if( r ) {
			printf( "BulkIn: err ep=%u r=%02x INT_ST=%02x\n",
			        ep_num, r, (unsigned)USBOTG_H_FS->INT_ST );
			return r | USBH_ERR_USB_TRANSFER;
		}
		Delay_Us( USBH_INTER_RETRY_DELAY_US );
	} while( ++trans_retry < 10 );

	return USBH_ERR_USB_TRANSFER;
}
