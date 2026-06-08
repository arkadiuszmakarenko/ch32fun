// usbhost_reportparser.c
//
// HID Report Descriptor parser for the ch32fun USB host stack.
// Adapted from the reference firmware's usb_hid_reportparser.c
// (MIT-like, see firmware/src/User/USB_Host/usb_hid_reportparser.c).
//
// The descriptor grammar is the standard HID "short items" form
// (USB HID 1.11 spec, section 6.2.2.2).  We only care about the
// fields we actually need: 2 axes, 12 buttons, 1 hat, 1 wheel,
// and a Report ID prefix.  Anything else is skipped.
//
// The parser is forgiving (it bails on unknown main/global items
// rather than trying to keep parsing) but it handles the common
// patterns seen in mice, keyboards, and consumer gamepads.

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usbhost_reportparser.h"


// Internal flags tracking which required pieces we've found.
#define RP_MOUSE_AXIS_X   0x01
#define RP_MOUSE_AXIS_Y   0x02
#define RP_MOUSE_BTN0     0x04
#define RP_MOUSE_BTN1     0x08
#define RP_JOY_COMPLETE   (RP_MOUSE_AXIS_X | RP_MOUSE_AXIS_Y | RP_MOUSE_BTN0)
#define RP_MOUSE_COMPLETE (RP_MOUSE_AXIS_X | RP_MOUSE_AXIS_Y | RP_MOUSE_BTN0 | RP_MOUSE_BTN1)


// Last bail-out reason, captured for callers that want to print it
// after `parse_report_descriptor()` returns 0. The numeric value is
// the (type << 8) | tag of the offending short item, or 0xFFFF if
// the parser gave up because no End-Collection was seen.
static uint16_t g_rp_last_err = 0;
static uint16_t g_rp_last_off = 0;

uint16_t usbhost_reportparser_last_err( uint16_t *poff )
{
	if( poff ) *poff = g_rp_last_off;
	return g_rp_last_err;
}

#define RP_BAIL(reason) do { g_rp_last_err = (reason); g_rp_last_off = (uint16_t)( rep - rep_base ); return 0; } while(0)


// ---------------------------------------------------------------------------
// collect_bits
//
// Extract `size` bits starting at `offset` from a little-endian bit
// stream `p`. Mirrors firmware/src/User/utils.c::collect_bits:
//   - If `is_signed` is true, sign-extend the result.
//   - The HID descriptor is allowed to use logical_minimum > logical_maximum
//     to indicate a signed value; the caller does that conversion.
// ---------------------------------------------------------------------------
uint16_t collect_bits( const uint8_t *p, uint16_t offset, uint8_t size, int is_signed )
{
	uint8_t  mask = (uint8_t)( 0xffu << ( offset & 7 ) );
	uint8_t  byte = (uint8_t)( offset / 8 );
	uint8_t  bits = size;
	uint8_t  shift = (uint8_t)( offset & 7 );
	uint16_t rval;

	rval = (uint16_t)( ( p[byte++] & mask ) >> shift );
	mask = 0xffu;
	shift = (uint8_t)( 8 - shift );

	// The first byte held more bits than we need.
	if( shift > size )
	{
		rval &= (uint16_t)( ( 1u << size ) - 1u );
	}
	else
	{
		bits = (uint8_t)( bits - shift );
		while( bits )
		{
			mask = ( bits < 8 ) ? (uint8_t)( 0xffu >> ( 8 - bits ) ) : 0xffu;
			rval = (uint16_t)( rval | (uint16_t)( (uint16_t)( p[byte++] & mask ) << shift ) );
			shift = (uint8_t)( shift + 8 );
			bits  = ( bits > 8 ) ? (uint8_t)( bits - 8 ) : 0;
		}
	}

	if( is_signed )
	{
		uint16_t sign_bit = (uint16_t)( 1u << ( size - 1 ) );
		if( rval & sign_bit )
		{
			while( sign_bit )
			{
				rval |= sign_bit;
				sign_bit = (uint16_t)( sign_bit << 1 );
			}
		}
	}

	return rval;
}


// ---------------------------------------------------------------------------
// HID short-item layout
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
	uint8_t bSize : 2;
	uint8_t bType : 2;
	uint8_t bTag  : 4;
} item_t;


// Returns 1 if the report (described by `conf`) is "complete" —
// has at least the X/Y axis and one button for a mouse/joystick,
// or any keys for a keyboard.
static int report_is_usable( uint16_t bit_count, uint8_t report_complete, hid_report_t *conf )
{
	conf->report_size = (uint8_t)( ( bit_count + 7 ) / 8 );

	if( ( conf->type == USBH_REPORT_TYPE_JOYSTICK &&
	      ( report_complete & RP_JOY_COMPLETE ) == RP_JOY_COMPLETE ) ||
	    ( conf->type == USBH_REPORT_TYPE_MOUSE &&
	      ( report_complete & RP_MOUSE_COMPLETE ) == RP_MOUSE_COMPLETE ) ||
	    ( conf->type == USBH_REPORT_TYPE_KEYBOARD ) )
	{
		return 1;
	}
	return 0;
}


// ---------------------------------------------------------------------------
// parse_report_descriptor
// ---------------------------------------------------------------------------
int parse_report_descriptor( const uint8_t *rep, uint16_t rep_size, hid_report_t *conf )
{
	const uint8_t *rep_base = rep;
	g_rp_last_err = 0;
	g_rp_last_off = 0;

	memset( conf, 0, sizeof( *conf ) );
	conf->type = USBH_REPORT_TYPE_NONE;

	int8_t   app_collection = 0;
	int8_t   phys_log_collection = 0;
	uint8_t  skip_collection = 0;
	int8_t   collection_depth = 0;

	uint8_t  report_size = 0;
	uint8_t  report_count = 0;
	uint16_t bit_count = 0;
	uint16_t usage_count = 0;
	uint16_t logical_minimum = 0;
	uint16_t logical_maximum = 0;

	uint8_t  report_complete = 0;

	int8_t   axis[2] = { -1, -1 };
	int8_t   hat = -1;
	int8_t   wheel = -1;

	while( rep_size )
	{
		// Decode short item header.
		item_t it;
		memcpy( &it, rep, sizeof( it ) );
		uint8_t tag  = it.bTag;
		uint8_t type = it.bType;
		uint8_t size = it.bSize;
		rep      += 1;
		rep_size -= 1;

		// Decode 0/1/2/4-byte data payload.
		uint32_t value = 0;
		if( size >= 1 ) { value  = *rep++; rep_size--; }
		if( size >= 2 ) { value |= (uint32_t)( (uint32_t)*rep++ << 8 ); rep_size--; }
		if( size >= 3 ) {
			value &= 0xFFFFu;
			value |= (uint32_t)( *rep++ ) << 16;
			value |= (uint32_t)( *rep++ ) << 24;
			rep_size -= 2;
		}
		(void)value;

		if( skip_collection )
		{
			// Track nesting depth, otherwise ignore.
			if( type == 0 )
			{
				if( tag == 10 ) { skip_collection++; collection_depth++; }
				else if( tag == 12 ) {
					skip_collection--; collection_depth--;
				}
			}
			continue;
		}

		switch( type )
		{
			case 0: // Main item
			{
				switch( tag )
				{
					case 8:  // Input
					{
						// Buttons: collect up to USBH_REPORT_BUTTONS bits.
						// (Most devices emit one bit per button; we
						// record the byte/bit of each, plus a total
						// count of button-bits so the decoder can
						// reach button[4..11] even when the parser
						// only stored offsets for the first four.)
						uint8_t b;
						for( b = 0; b < USBH_REPORT_BUTTONS; b++ )
						{
							if( b < report_count )
							{
								uint16_t this_bit = (uint16_t)( bit_count + b );
								conf->button[b].byte_offset = (uint8_t)( this_bit / 8 );
								conf->button[b].bitmask     = (uint8_t)( 1u << ( this_bit & 7 ) );
							}
						}
						conf->button_count = (uint8_t)( report_count * report_size );
						if( conf->button_count > 0 ) report_complete |= RP_MOUSE_BTN0;
						if( conf->button_count > 1 ) report_complete |= RP_MOUSE_BTN1;

						// Axes.
						uint8_t c;
						for( c = 0; c < 2; c++ )
						{
							if( axis[c] >= 0 )
							{
								uint16_t off = (uint16_t)( bit_count + report_size * axis[c] );
								conf->axis[c].offset = off;
								conf->axis[c].size   = report_size;
								conf->axis[c].logical.min = logical_minimum;
								conf->axis[c].logical.max = logical_maximum;
								if( c == 0 ) report_complete |= RP_MOUSE_AXIS_X;
								if( c == 1 ) report_complete |= RP_MOUSE_AXIS_Y;
							}
						}

						// Hat (joystick only).
						if( hat >= 0 && conf->type == USBH_REPORT_TYPE_JOYSTICK )
						{
							uint16_t off = (uint16_t)( bit_count + report_size * hat );
							conf->hat.offset = off;
							conf->hat.size   = report_size;
						}

						// Wheel (mouse only).
						if( wheel >= 0 && conf->type == USBH_REPORT_TYPE_MOUSE )
						{
							uint16_t off = (uint16_t)( bit_count + report_size * wheel );
							conf->wheel.offset = off;
							conf->wheel.size   = report_size;
							conf->wheel.logical.min = logical_minimum;
							conf->wheel.logical.max = logical_maximum;
						}

						// Reset per-field state.
						bit_count = (uint16_t)( bit_count + report_count * report_size );
						usage_count = 0;
						axis[0] = axis[1] = -1;
						hat = -1;
						wheel = -1;
						break;
					}

					case 9:   // Output — ignored
					case 11:  // Feature — ignored
						break;

					case 10:  // Collection
					{
						collection_depth++;
						usage_count = 0;
						if( value == 1 ) {                 // Application
							app_collection++;
						} else if( value == 0 || value == 2 ) { // Physical / Logical
							phys_log_collection++;
						} else {
							skip_collection++;
						}
						break;
					}

					case 12:  // End collection
					{
						collection_depth--;
						if( phys_log_collection )
						{
							phys_log_collection--;
						}
						else if( app_collection )
						{
							app_collection--;
							if( report_is_usable( bit_count, report_complete, conf ) )
								return 1;
							// Reset and try the next report within this
							// descriptor (e.g. a combined mouse+keyboard
							// device with two top-level collections).
							bit_count = 0;
							report_complete = 0;
							conf->type = USBH_REPORT_TYPE_NONE;
						}
						else
						{
							// Unbalanced End-Collection.
							RP_BAIL( 0xFFFEu );
							return 0;
						}
						break;
					}

					default:
						RP_BAIL( ( 0u << 8 ) | tag );
					// Unknown main-item tag.
					return 0;
				}
				break;
			}

			case 1: // Global item
			{
				switch( tag )
				{
					case 0:  // Usage Page — nothing to do
					case 3:  // Physical Minimum
					case 4:  // Physical Maximum
					case 5:  // Unit Exponent
					case 6:  // Unit
						break;
					case 1:  logical_minimum = (uint16_t)value; break;
					case 2:  logical_maximum = (uint16_t)value; break;
					case 7:  report_size = (uint8_t)value;     break;
					case 8:  conf->report_id = (uint8_t)value; break;
					case 9:  report_count = (uint8_t)value;    break;
					default:
						RP_BAIL( ( 1u << 8 ) | tag );
					// Unknown global-item tag.
					return 0;
				}
				break;
			}

			case 2: // Local item
			{
				switch( tag )
				{
					case 0:  // Usage
					{
						// Top-level usage sets the device class.
						if( collection_depth == 0 )
						{
							if( value == 6 )      conf->type = USBH_REPORT_TYPE_KEYBOARD;
							else if( value == 2 ) conf->type = USBH_REPORT_TYPE_MOUSE;
							else if( value == 4 || value == 5 )
								conf->type = USBH_REPORT_TYPE_JOYSTICK;
						}
						else if( app_collection )
						{
							// In an application collection, X/Y = axis,
							// pointer is allowed, hat/wheel too.
							if( value == 48 /*X*/ || value == 49 /*Y*/ )
							{
								if( conf->type == USBH_REPORT_TYPE_JOYSTICK ||
								    conf->type == USBH_REPORT_TYPE_MOUSE )
								{
									if( value == 48 )      axis[0] = (int8_t)usage_count;
									else                   axis[1] = (int8_t)usage_count;
								}
							}
							else if( value == 57 /*HAT*/ &&
							         conf->type == USBH_REPORT_TYPE_JOYSTICK )
							{
								hat = (int8_t)usage_count;
							}
							else if( value == 56 /*WHEEL*/ &&
							         conf->type == USBH_REPORT_TYPE_MOUSE )
							{
								wheel = (int8_t)usage_count;
							}
						}
						usage_count++;
						break;
					}
					case 1:  // Usage Minimum
					{
						// range-populate: usage_count += (max-min)
						uint8_t min = (uint8_t)value;
						// We don't have the max here, but the parser
						// convention is Usage Minimum = N means
						// usage_count += N. Caller follows with Usage
						// Maximum = M. We decrement here (will be
						// re-incremented when Maximum arrives).
						usage_count = (uint16_t)( usage_count - ( min - 1u ) );
						break;
					}
					case 2:  // Usage Maximum
						usage_count += (uint16_t)value;
						break;
					default:
						break;
				}
				break;
			}

			default:
				// Reserved.
				break;
		}
	}

	// Descriptor ended without an End-Collection. Still accept if usable.
	if( !report_is_usable( bit_count, report_complete, conf ) )
	{
		// Incomplete report — record why (the type field still
		// holds the most recent Usage-driven classification, so
		// the caller can read that for context).
		g_rp_last_err = 0xFFFFu;
		g_rp_last_off = (uint16_t)( rep - rep_base );
		return 0;
	}
	return 1;
}
