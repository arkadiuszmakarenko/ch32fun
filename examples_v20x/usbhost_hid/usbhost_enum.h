// usbhost_enum.h
//
// Public surface of the enumeration state machine in usbhost_enum.c.
//
// The class-driver extensions (HID, MSC, ...) sit downstream of
// USBH_Enumerate(): once a device has been successfully enumerated,
// the main loop invokes the user-supplied USBH_OnEnumSuccess() callback
// so the per-class driver can take over (claim interfaces, set
// protocol, mount the volume, etc).

#ifndef USBHOST_ENUM_H
#define USBHOST_ENUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Run the full enumeration sequence on the device currently attached
// to the root port:
//
//   1. GET_DESCRIPTOR(DEVICE, 8)   →  bMaxPacketSize0
//   2. GET_DESCRIPTOR(DEVICE)     →  full 18-byte descriptor
//   3. SET_ADDRESS(2)             →  move off address 0
//   4. GET_DESCRIPTOR(CONFIG, 4)  →  wTotalLength
//   5. GET_DESCRIPTOR(CONFIG)     →  full config descriptor
//   6. SET_CONFIGURATION(1)       →  device is now configured
//
// On success, USBH_OnEnumSuccess() is invoked with the speed and a
// pointer to the freshly-read device + config descriptors. The config
// descriptor buffer is USBH_CFG_DESC_BUFFER_SIZE bytes long; the
// actual length is in cfg_len.
//
// `speed` is USB_SPEED_LOW (0) or USB_SPEED_FULL (1), matching the
// values USBH_PortEnable() wrote back.
//
// Returns USBH_ERR_SUCCESS (0) on success, or one of the USBH_ERR_*
// error codes on failure. (Failures here don't fire the callback.)
uint8_t USBH_Enumerate( uint8_t speed );

// Size of the internal config-descriptor scratch buffer. 256 bytes is
// enough for the vast majority of flash drives / HID / CDC devices;
// a HID report descriptor alone can be longer, but it lives outside
// this buffer (class drivers do their own).
#define USBH_CFG_DESC_BUFFER_SIZE   256

// Callback fired from the main loop, exactly once per successful
// enumeration. The default implementation in usbhost_enum.c is a
// weak symbol — project-specific main()s override it to bring up
// their class driver.
//
// Parameters:
//   speed     — USB_SPEED_LOW or USB_SPEED_FULL
//   dev_desc  — pointer to the 18-byte device descriptor
//   cfg_desc  — pointer to the (USBH_CFG_DESC_BUFFER_SIZE-byte max)
//               configuration descriptor scratch buffer
//   cfg_len   — number of valid bytes in *cfg_desc
//
// The buffer stays valid only until the next attach/detach or bus
// reset (the next enumeration overwrites it). Class drivers must
// copy out anything they need long-term.
void USBH_OnEnumSuccess( uint8_t speed,
                         const uint8_t *dev_desc,
                         const uint8_t *cfg_desc,
                         uint16_t       cfg_len );

#ifdef __cplusplus
}
#endif

#endif // USBHOST_ENUM_H
