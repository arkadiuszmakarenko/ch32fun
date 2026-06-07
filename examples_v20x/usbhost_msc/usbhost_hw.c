// usbhost_hw.c
// Phase A: USBFS host bring-up for CH32V203.
//
// This file is HAL-free. All register names come from ch32fun's
// ch32v20xhw.h, which exposes the host-mode register block as
// USBOTG_H_FS (the USBOTG_FS_HOST_TypeDef struct at 0x50000000).
//
// Phase A responsibilities:
//   - Configure PLL/USB clock tree for a 48 MHz USB clock
//   - Configure PA11/PA12 (DM/DP) and the VBUS-FET GPIO
//   - Put the SIE into host mode, set up EP0 DMA buffers, enable
//     DETECT + TRANSFER interrupts
//   - Provide bus reset, port enable, and port-status polling
//
// Transfers and enumeration come in Phase B.

#include "ch32fun.h"
#include <stdio.h>
#include "usbhost_hw.h"

// ---------------------------------------------------------------------------
// DMA scratch buffers.
//
// The SIE reads the EP0 OUT token/data from HOST_TX_DMA and writes the
// EP0 IN payload to HOST_RX_DMA. Both must be 4-byte aligned (the
// reference manual requires an even address; 4 is a safe over-align).
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// DMA scratch buffers.
//
// The SIE reads the EP0 OUT token/data from HOST_TX_DMA and writes the
// EP0 IN payload to HOST_RX_DMA. Both must be 4-byte aligned (the
// reference manual requires an even address; 4 is a safe over-align).
//
// These are *not* static: usbhost_xfer.c places the SETUP packet in
// USBH_TxBuf before issuing USBH_Transact(), and reads the IN payload
// out of USBH_RxBuf after a successful IN. Exported via usbhost_hw.h.
// ---------------------------------------------------------------------------
__attribute__((aligned(4))) uint8_t USBH_RxBuf[ USBHOST_EP0_MAX_SIZE ];
__attribute__((aligned(4))) uint8_t USBH_TxBuf[ USBHOST_EP0_MAX_SIZE ];

// Compile-time sanity: a misaligned buffer will hard-fault on first
// transfer. The compiler can't see the __attribute__((aligned(4)))
// guarantee as a constant expression for a static array, so we do a
// one-shot runtime check at first use of the SIE.
static void USBH_CheckBufAlign( void )
{
	if( ( (uintptr_t)USBH_RxBuf & 3u ) || ( (uintptr_t)USBH_TxBuf & 3u ) )
	{
		// Trapped: misaligned DMA buffer. Halt the CPU so the user
		// sees it on the debugger / via the next printf never coming.
		for(;;) { __asm__ volatile("nop"); }
	}
}

// Bus-reset timing. 15 ms is comfortably inside the USB 2.0 spec's
// 10..20 ms window for a downstream-port reset.
#define USBH_BUS_RESET_MS   15

// ---------------------------------------------------------------------------
// USBH_ClockInit
//
// Set the USB OTG_FS clock prescaler in RCC->CFGR0[23:22] so that the
// SIE receives exactly 48 MHz, then gate the OTG_FS AHB bus clock on.
//
// USBPRE encoding (RCC_USBCLKConfig in ch32v20x_rcc.c):
//   00 → PLLCLK / 1   (PLL = 48 MHz)
//   01 → PLLCLK / 2   (PLL = 96 MHz)
//   10 → PLLCLK / 3   (PLL = 144 MHz, default for ch32fun with V20x)
//
// The PLL frequency is HSI * FUNCONF_PLL_MULTIPLIER (8 MHz * 18 = 144 MHz
// by default for V20x). FUNCONF_SYSTEM_CORE_CLOCK is HCLK (may be PLL
// divided down) and is NOT the PLL frequency. The right value to check
// here is the PLL output, not the HCLK.
// ---------------------------------------------------------------------------
void USBH_ClockInit( void )
{
	// The PLL runs at HSI * FUNCONF_PLL_MULTIPLIER. For V20x with the
	// ch32fun default, that's 8 MHz * 18 = 144 MHz. We need to divide
	// it down to 48 MHz for USB.
#if   ( HSI_VALUE * FUNCONF_PLL_MULTIPLIER ) == 48000000
	RCC->CFGR0 = ( RCC->CFGR0 & ~( (uint32_t)3u << 22 ) ) | ( 0u << 22 );
#elif ( HSI_VALUE * FUNCONF_PLL_MULTIPLIER ) == 96000000
	RCC->CFGR0 = ( RCC->CFGR0 & ~( (uint32_t)3u << 22 ) ) | ( 1u << 22 );
#elif ( HSI_VALUE * FUNCONF_PLL_MULTIPLIER ) == 144000000
	RCC->CFGR0 = ( RCC->CFGR0 & ~( (uint32_t)3u << 22 ) ) | ( 2u << 22 );
#else
	// Compute the divisor needed to get closest to 48 MHz.
	// For PLL=72 MHz we'd need /1.5 (not available) — but the chip's
	// PLL is locked to 48/72/96/144 by its 8 MHz HSI base, so this is
	// only hit if someone configures an unsupported PLL multiplier.
#  error "PLL frequency must be 48, 96, or 144 MHz for USB at 48 MHz"
#endif

	// Enable the OTG_FS AHB clock. Without this, USBOTG_H_FS reads return 0.
	RCC->AHBPCENR |= RCC_AHBPeriph_OTG_FS;

	// Disable the internal D+ pull-up (device-mode pull-up; must be off
	// in host mode so the device side drives D+ to signal connect).
	EXTEND->CTR &= ~EXTEN_USBD_PU_EN;
}

// ---------------------------------------------------------------------------
// USBH_GpioInit
//
// PA11 = USB D-,  PA12 = USB D+, both AF push-pull 10 MHz.
// PA0  = VBUS FET gate, push-pull 2 MHz (default on).
//
// PA11/PA12 are the USBFS pins on every CH32V203 package. 10 MHz slew
// is enough at full speed (12 Mbps).
// ---------------------------------------------------------------------------
void USBH_GpioInit( void )
{
	// Enable the AFIO peripheral clock so the USB peripheral can take
	// over PA11/PA12 via the alternate-function mux. Without this, the
	// SIE registers work but no signal reaches the pins.
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

	funPinMode( PA11, GPIO_CFGLR_OUT_10Mhz_AF_PP );
	funPinMode( PA12, GPIO_CFGLR_OUT_10Mhz_AF_PP );

	funPinMode( PA0, GPIO_CFGLR_OUT_2Mhz_PP );
	USBH_VbusOn();
}

void USBH_VbusOn( void )  { funDigitalWrite( PA0, FUN_HIGH ); }
void USBH_VbusOff( void ) { funDigitalWrite( PA0, FUN_LOW );  }

// SystemCoreClock is defined by ch32fun's startup on chips that ship with
// a SystemInit() that knows the PLL config (V003). On V20x the symbol
// doesn't exist, so we provide it here, sourced from the FUNCONF clock
// macro. USBH_ClockInit() above programs the PLL to that exact HCLK, so
// reporting the macro value back as the live HCLK is correct.
uint32_t SystemCoreClock = FUNCONF_SYSTEM_CORE_CLOCK;

uint32_t USBH_GetHclk( void ) { return SystemCoreClock; }

// ---------------------------------------------------------------------------
// USBH_HostInit
//
// Program the SIE into host mode and point its EP0 DMA at our scratch
// buffers. Based on the USBOTG_FS_HOST_TypeDef layout in ch32v20xhw.h:
//
//   BASE_CTRL   = 0   | UC_HOST_MODE | UC_INT_BUSY | UC_DMA_EN
//   HOST_CTRL   = 0   (port not yet enabled, bus not held in reset)
//   DEV_ADDR    = 0   (no address assigned yet)
//   HOST_EP_MOD = UH_EP_TX_EN | UH_EP_RX_EN
//   HOST_RX_DMA = &USBH_RxBuf[0]
//   HOST_TX_DMA = &USBH_TxBuf[0]
//   HOST_SETUP  = UH_SOF_EN  (start emitting 1 ms SOF frames)
//   INT_FG      = 0xFF        (clear all latched events)
//   INT_EN      = UIE_TRANSFER | UIE_DETECT
// ---------------------------------------------------------------------------
void USBH_HostInit( void )
{
	USBH_CheckBufAlign();

	// Soft-reset the SIE + clear all state (matching the WCH reference).
	USBOTG_H_FS->BASE_CTRL = USBOTG_UC_RESET_SIE | USBOTG_UC_CLR_ALL;
	Delay_Us( 10 );
	USBOTG_H_FS->BASE_CTRL = 0;

	// Enter host mode + enable DMA + INT_BUSY. HOST_CTRL, DEV_ADDR,
	// HOST_SETUP, INT_FG and INT_EN are all left at their reset
	// defaults; EnableRootHubPort and the transfer loop will set
	// them as needed.
	USBOTG_H_FS->BASE_CTRL  = USBOTG_UC_HOST_MODE | USBOTG_UC_INT_BUSY | USBOTG_UC_DMA_EN;
	USBOTG_H_FS->HOST_EP_MOD = USBOTG_UH_EP_TX_EN | USBOTG_UH_EP_RX_EN;
	USBOTG_H_FS->HOST_RX_DMA = (uint32_t)USBH_RxBuf;
	USBOTG_H_FS->HOST_TX_DMA = (uint32_t)USBH_TxBuf;

	// Explicitly clear the data-toggle registers. The WCH EVT
	// reference does this before setting BASE_CTRL; on a freshly
	// powered chip the reset value of these registers is not
	// guaranteed, and starting a SETUP token with toggle=1 (DATA1)
	// means the device will see the very first packet as a mismatch
	// and the entire enumeration hangs in the NHR retry loop.
	USBOTG_H_FS->HOST_RX_CTRL = 0x00u;
	USBOTG_H_FS->HOST_TX_CTRL = 0x00u;

	// Debug: dump post-init state.
	uint32_t cfgr0 = RCC->CFGR0;
	printf( "HOST init: BASE=%02x HOST_CTRL=%02x HOST_EP_MOD=%02x\n",
	        (unsigned)USBOTG_H_FS->BASE_CTRL,
	        (unsigned)USBOTG_H_FS->HOST_CTRL,
	        (unsigned)USBOTG_H_FS->HOST_EP_MOD );
	printf( "  RxBuf=%p TxBuf=%p HOST_SETUP=%04x CFGR0=%08lx USBPRE=%lu\n",
	        (void*)USBH_RxBuf,
	        (void*)USBH_TxBuf,
	        (unsigned)USBOTG_H_FS->HOST_SETUP,
	        (unsigned long)cfgr0,
	        (unsigned long)( ( cfgr0 >> 22 ) & 3u ) );

	// Re-force USB prescaler to PLL/3 right before SIE use, in case
	// something earlier clobbered it. PLL = HSI * 18 = 144 MHz, so
	// /3 gives 48 MHz for the USB SIE.
	RCC->CFGR0 = ( RCC->CFGR0 & ~( (uint32_t)3u << 22 ) ) | ( 2u << 22 );
	cfgr0 = RCC->CFGR0;
	printf( "  CFGR0 after force = %08lx USBPRE=%lu\n",
	        (unsigned long)cfgr0,
	        (unsigned long)( ( cfgr0 >> 22 ) & 3u ) );
}

uint8_t USBH_IsDevicePresent( void )
{
	return ( USBOTG_H_FS->MIS_ST & USBOTG_UMS_DEV_ATTACH ) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// USBH_PortCheckStatus
//
// Reads the live MIS_ST register and returns the current root-hub state.
// The DETECT interrupt flag is *not* the right thing to look at: in Phase A
// we run with interrupts enabled but the flag is only set on edges, so
// polling MIS_ST directly is more robust.
// ---------------------------------------------------------------------------
uint8_t USBH_PortCheckStatus( void )
{
	if( !( USBOTG_H_FS->MIS_ST & USBOTG_UMS_DEV_ATTACH ) )
		return USBH_PORT_DETACHED;

	if( USBOTG_H_FS->HOST_CTRL & USBOTG_UH_PORT_EN )
		return USBH_PORT_ENABLED;

	return USBH_PORT_ATTACHED;
}

// ---------------------------------------------------------------------------
// USBH_BusReset
//
// mode 0: full cycle (assert, hold 15 ms, deassert)        - normal use
// mode 1: assert bus reset only                            - diagnostic
// mode 2: deassert bus reset                               - diagnostic
// ---------------------------------------------------------------------------
void USBH_BusReset( uint8_t mode )
{
	USBOTG_H_FS->DEV_ADDR = 0;

	// CRITICAL: pre-condition the SIE to FULL-speed mode at the start
	// of every bus reset. The SIE defaults to a state where PRE_PID
	// packets (low-speed preambles) are enabled — that mode is only
	// valid for a low-speed device, but applied to a full-speed device
	// every SETUP/IN/OUT token gets a PRE_PID prepended and the device
	// ignores it (USB 2.0 §8.5: a full-speed hub/data-line device
	// discards PRE_PID packets). The SIE then sees "no handshake" on
	// the wire, which is exactly the NHR loop we observed on the USB
	// flash drive while the mouse (low-speed) enumerated fine: PRE
	// packets on the way to a low-speed device *are* a no-op for the
	// device, but the SIE is set up to do the right thing after that.
	// The WCH EVT reference calls USBFSH_SetSelfSpeed(USB_FULL_SPEED)
	// at the top of USBFSH_ResetRootHubPort; we mirror that.
	USBH_SetSelfSpeed( USB_SPEED_FULL );

	if( mode == 0 || mode == 1 )
	{
		USBOTG_H_FS->HOST_CTRL = ( USBOTG_H_FS->HOST_CTRL & ~USBOTG_UH_LOW_SPEED )
		                       | USBOTG_UH_BUS_RESET;
	}
	if( mode == 0 )
	{
		Delay_Ms( USBH_BUS_RESET_MS );
	}
	if( mode == 0 || mode == 2 )
	{
		USBOTG_H_FS->HOST_CTRL &= ~USBOTG_UH_BUS_RESET;
	}

	// 2 ms settle per WCH EVT reference. The USB 2.0 spec only
	// requires "Reset Recovery" before the host can start signalling
	// (10 ms minimum after attach, but the device is already past
	// that by the time we get here). The EVT stack uses 2 ms; we
	// used to use 10 ms, which gave slow devices an extra 8 ms of
	// confusion before the first SETUP was issued.
	Delay_Ms( 2 );

	// Latch the attach/detach edge so the next poll of INT_FG is clean.
	USBOTG_H_FS->INT_FG = USBOTG_UIF_DETECT;
}

// ---------------------------------------------------------------------------
// USBH_PortEnable
//
// Sets UH_PORT_EN and SOF-enable. Detects LS by sampling DM_LEVEL: a low
// speed device keeps D- pulled high (DM_LEVEL=1 == low speed).
// ---------------------------------------------------------------------------
uint8_t USBH_PortEnable( uint8_t *pspeed )
{
	if( !( USBOTG_H_FS->MIS_ST & USBOTG_UMS_DEV_ATTACH ) )
		return USBH_PORT_DETACHED;

	if( !( USBOTG_H_FS->HOST_CTRL & USBOTG_UH_PORT_EN ) )
	{
		*pspeed = ( USBOTG_H_FS->MIS_ST & USBOTG_UMS_DM_LEVEL )
		          ? USB_SPEED_LOW : USB_SPEED_FULL;
		// USBH_BusReset() pre-conditioned the SIE to full-speed (no
		// PRE_PID, no LOW_SPEED bits). Only flip to low-speed mode
		// if the device is actually low-speed — and crucially, do
		// this BEFORE the first SETUP token is dispatched.
		USBH_SetSelfSpeed( *pspeed );
	}

	USBOTG_H_FS->HOST_CTRL |= USBOTG_UH_PORT_EN;
	USBOTG_H_FS->HOST_SETUP = USBOTG_UH_SOF_EN;

	// Force the USB prescaler to PLL/3 right before SIE use. The
	// SystemInit / HostInit sequence may clobber CFGR0[23:22], so
	// re-assert it here to guarantee 48 MHz for the SIE.
	RCC->CFGR0 = ( RCC->CFGR0 & ~( (uint32_t)3u << 22 ) ) | ( 2u << 22 );

	// Clear any pending interrupt flags, then unmask the two we care
	// about: port detect (attach/detach) and transfer-complete.
	USBOTG_H_FS->INT_FG = 0xFF;
	USBOTG_H_FS->INT_EN = USBOTG_UIE_TRANSFER | USBOTG_UIE_DETECT;

	// USB 2.0 §9.1.2: 10 ms recovery minimum after reset. We give
	// 50 ms to cover flash drives and other devices with slow
	// firmware bring-up — those devices will sometimes ACK the bus
	// reset (so we see MIS_ST.ATTACH and PORT_EN), but their USB
	// controller isn't ready to handle a SETUP token for tens of
	// milliseconds. Bumping this from 20 ms to 50 ms is the single
	// biggest win for "works on most devices, fails on a flash
	// drive" — the controller sees ~50 SOF frames before we ask it
	// to do anything, which is enough for the slow ones to come up.
	Delay_Ms( 50 );
	printf( "PortEnable: MIS=%02x HOST_CTRL=%02x HOST_SETUP=%04x BASE=%02x CFGR0=%08lx\n",
	        (unsigned)USBOTG_H_FS->MIS_ST,
	        (unsigned)USBOTG_H_FS->HOST_CTRL,
	        (unsigned)USBOTG_H_FS->HOST_SETUP,
	        (unsigned)USBOTG_H_FS->BASE_CTRL,
	        (unsigned long)RCC->CFGR0 );

	// Reset the data-toggle state for a fresh enumeration. The WCH
	// reference does not do this here (it relies on bus-reset to
	// clear toggles), but we observed that after a failed first
	// enumeration the SIE's internal toggle can be in an
	// inconsistent state — the next SETUP goes out as DATA1, the
	// device sees a mismatch, and we land back in the NHR loop.
	// Forcing both to DATA0 (toggle=0) here guarantees the first
	// SETUP of the new enumeration starts from a known state.
	USBOTG_H_FS->HOST_RX_CTRL = 0x00u;
	USBOTG_H_FS->HOST_TX_CTRL = 0x00u;
	USBOTG_H_FS->HOST_TX_LEN  = 0u;

	return USBH_PORT_ENABLED;
}

// ---------------------------------------------------------------------------
// USBH_SetSelfSpeed / USBH_SetSelfAddr
//
// Called from the enumeration state machine once we know the device speed
// and once we've been issued a SET_ADDRESS. ch32fun's struct gives us the
// bits directly; this just packages the field updates so callers don't
// have to read the bit-mask docs.
// ---------------------------------------------------------------------------
void USBH_SetSelfSpeed( uint8_t speed )
{
	if( speed == USB_SPEED_FULL )
	{
		USBOTG_H_FS->BASE_CTRL &= ~USBOTG_UC_LOW_SPEED;
		USBOTG_H_FS->HOST_CTRL &= ~USBOTG_UH_LOW_SPEED;
		USBOTG_H_FS->HOST_SETUP &= ~USBOTG_UH_PRE_PID_EN;
	}
	else
	{
		USBOTG_H_FS->BASE_CTRL |= USBOTG_UC_LOW_SPEED;
		USBOTG_H_FS->HOST_CTRL |= USBOTG_UH_LOW_SPEED;
		USBOTG_H_FS->HOST_SETUP |= USBOTG_UH_PRE_PID_EN;
	}
}

void USBH_SetSelfAddr( uint8_t addr )
{
	USBOTG_H_FS->DEV_ADDR = ( USBOTG_H_FS->DEV_ADDR & USBFS_UDA_GP_BIT )
	                      | ( addr & USBFS_USB_ADDR_MASK );
}
