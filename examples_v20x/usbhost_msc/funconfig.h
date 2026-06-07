#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

// USB host needs a known, accurate 48 MHz clock so the SOF frame timer
// is within the +-0.25% tolerance USB 2.0 requires.
#define FUNCONF_USE_PLL                1
#define FUNCONF_USE_HSI                1
#define FUNCONF_SYSTEM_CORE_CLOCK      144000000
#define FUNCONF_HSE_BYPASS             0

// Debug printf goes over the SWIO programming wire (no UART required).
#define FUNCONF_USE_DEBUGPRINTF        1
#define FUNCONF_USE_UARTPRINTF         0

#endif
