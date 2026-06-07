// usbhost_hw.h
// Phase A: bring up the CH32V203 USBFS host SIE (clock, GPIO, SIE reset,
// bus reset, port enable, port-status polling). No transfers yet.

#ifndef USBHOST_HW_H
#define USBHOST_HW_H

#include <stdint.h>

// USB low-/full-speed constants (USB 2.0 spec)
#define USB_SPEED_UNKNOWN   0xFF
#define USB_SPEED_LOW       0x00
#define USB_SPEED_FULL      0x01

// 64 bytes is the largest full-speed EP0 packet. Buffers are 4-byte aligned
// (the SIE DMA requires an even address; we use 4 to be safe).
#define USBHOST_EP0_MAX_SIZE    64

// EP0 scratch buffers shared between the SIE (DMA) and the CPU (SETUP
// packet placement and IN-payload read-back). Both must stay 4-byte
// aligned; see usbhost_hw.c for the attribute((aligned(4))).
extern uint8_t USBH_RxBuf[ USBHOST_EP0_MAX_SIZE ];
extern uint8_t USBH_TxBuf[ USBHOST_EP0_MAX_SIZE ];

// Status reported by USBH_PortCheckStatus() and friends.
enum {
	USBH_PORT_DETACHED = 0,
	USBH_PORT_ATTACHED,
	USBH_PORT_ENABLED,
	USBH_PORT_ERROR,
};

#ifdef __cplusplus
extern "C" {
#endif

// One-time init: PLL to 48 MHz, USB clock sourced from PLL, USB AHB clock
// enabled, internal D+ pull-up disabled (host mode), VBUS FET driven on.
void USBH_ClockInit( void );

// Returns the running HCLK in Hz. Set by ch32fun's SystemInit() from the
// CFGR0 / PLLMUL configuration we picked in USBH_ClockInit().
uint32_t USBH_GetHclk( void );

// Configure PA11 (D-) / PA12 (D+) as the USBFS host port pins, plus the
// VBUS power switch GPIO (PA0 by default). Call once before USBH_HostInit.
void USBH_GpioInit( void );

// Initialize the USBFS SIE into host mode with EP0 DMA buffers, SOFs on,
// and DETECT + TRANSFER interrupts unmasked. SIE resets all host state.
void USBH_HostInit( void );

// Returns one of USBH_PORT_* reflecting the current root-hub state.
uint8_t USBH_PortCheckStatus( void );

// Drive bus reset for ~15 ms (mode=0: full reset+release, 1: assert, 2:
// deassert). Spec-mandated reset is 10..20 ms.
void USBH_BusReset( uint8_t mode );

// Enables the port after a device has been attached. Writes the detected
// speed (USB_SPEED_LOW / USB_SPEED_FULL) into *pspeed.
uint8_t USBH_PortEnable( uint8_t *pspeed );

// Drive VBUS on/off (some boards use an active-low P-MOSFET gate).
void USBH_VbusOn( void );
void USBH_VbusOff( void );

// True if a device is currently attached (pulled from the live MIS_ST
// register, independent of the debounced state machine in main()).
uint8_t USBH_IsDevicePresent( void );

// Tell the SIE which speed the attached device runs at. Must be called
// after USBH_PortEnable() and before the first transaction on EP0.
void USBH_SetSelfSpeed( uint8_t speed );

// Tell the SIE which address the device has been assigned via
// SET_ADDRESS. Must be called after the device has accepted the new
// address (>= 2 ms settle is the spec requirement, the caller handles it).
void USBH_SetSelfAddr( uint8_t addr );

#ifdef __cplusplus
}
#endif

#endif // USBHOST_HW_H
