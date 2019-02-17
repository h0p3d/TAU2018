// This file is generated by Simplicity Studio.  Please do not edit manually.
//
//

// Enclosing macro to prevent multiple inclusion
#ifndef __FLEX_CONFIG__
#define __FLEX_CONFIG__




// Top level macros
#define EMBER_AF_DEVICE_NAME "RFsense_radio"


// Generated plugin macros

// Use this macro to check if HAL Library plugin is included
#define EMBER_AF_PLUGIN_HAL_EFR32

// Use this macro to check if Application Configuration plugin is included
#define EMBER_AF_PLUGIN_RAIL_APP_CONFIG

// Use this macro to check if RAIL Library plugin is included
#define EMBER_AF_PLUGIN_RAIL_LIBRARY
// User options for plugin RAIL Library
#define EMBER_AF_PLUGIN_RAIL_LIBRARY_RAILPHYDEF 1


// Generated API headers

// API rail-library from RAIL Library plugin
#define EMBER_AF_API_RAIL_LIBRARY "../../../../../SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v2.3/platform/radio/rail_lib/common/rail.h"


// Custom macros
#ifdef APP_SERIAL
#undef APP_SERIAL
#endif
#define APP_SERIAL 1

#ifdef APP_BAUD_RATE
#undef APP_BAUD_RATE
#endif
#define APP_BAUD_RATE BAUD_115200

#ifdef EMBER_ASSERT_SERIAL_PORT
#undef EMBER_ASSERT_SERIAL_PORT
#endif
#define EMBER_ASSERT_SERIAL_PORT 1

#ifdef EMBER_AF_BAUD_RATE
#undef EMBER_AF_BAUD_RATE
#endif
#define EMBER_AF_BAUD_RATE 115200

#ifdef EMBER_SERIAL1_MODE
#undef EMBER_SERIAL1_MODE
#endif
#define EMBER_SERIAL1_MODE EMBER_SERIAL_FIFO

#ifdef EMBER_SERIAL1_RX_QUEUE_SIZE
#undef EMBER_SERIAL1_RX_QUEUE_SIZE
#endif
#define EMBER_SERIAL1_RX_QUEUE_SIZE 128

#ifdef EMBER_SERIAL1_TX_QUEUE_SIZE
#undef EMBER_SERIAL1_TX_QUEUE_SIZE
#endif
#define EMBER_SERIAL1_TX_QUEUE_SIZE 128

#ifdef EMBER_SERIAL1_BLOCKING
#undef EMBER_SERIAL1_BLOCKING
#endif
#define EMBER_SERIAL1_BLOCKING

#ifdef EMBER_AF_RADIO
#undef EMBER_AF_RADIO
#endif
#define EMBER_AF_RADIO EFR32

#ifdef EMBER_AF_RADIO_FULL
#undef EMBER_AF_RADIO_FULL
#endif
#define EMBER_AF_RADIO_FULL EFR32FG14P233F256GM48

#ifdef EMBER_AF_RADIO_FAMILY
#undef EMBER_AF_RADIO_FAMILY
#endif
#define EMBER_AF_RADIO_FAMILY F

#ifdef EMBER_AF_RADIO_SERIES
#undef EMBER_AF_RADIO_SERIES
#endif
#define EMBER_AF_RADIO_SERIES 1

#ifdef EMBER_AF_RADIO_DEVICE_CONFIGURATION
#undef EMBER_AF_RADIO_DEVICE_CONFIGURATION
#endif
#define EMBER_AF_RADIO_DEVICE_CONFIGURATION 4

#ifdef EMBER_AF_RADIO_PERFORMANCE
#undef EMBER_AF_RADIO_PERFORMANCE
#endif
#define EMBER_AF_RADIO_PERFORMANCE P

#ifdef EMBER_AF_RADIO_RADIO
#undef EMBER_AF_RADIO_RADIO
#endif
#define EMBER_AF_RADIO_RADIO 233

#ifdef EMBER_AF_RADIO_FLASH
#undef EMBER_AF_RADIO_FLASH
#endif
#define EMBER_AF_RADIO_FLASH 256K

#ifdef EMBER_AF_RADIO_TEMP
#undef EMBER_AF_RADIO_TEMP
#endif
#define EMBER_AF_RADIO_TEMP G

#ifdef EMBER_AF_RADIO_PACKAGE
#undef EMBER_AF_RADIO_PACKAGE
#endif
#define EMBER_AF_RADIO_PACKAGE M

#ifdef EMBER_AF_RADIO_PINS
#undef EMBER_AF_RADIO_PINS
#endif
#define EMBER_AF_RADIO_PINS 48

#ifdef EMBER_AF_MCU
#undef EMBER_AF_MCU
#endif
#define EMBER_AF_MCU EFR32

#ifdef EMBER_AF_MCU_FULL
#undef EMBER_AF_MCU_FULL
#endif
#define EMBER_AF_MCU_FULL EFR32FG14P233F256GM48

#ifdef EMBER_AF_MCU_FAMILY
#undef EMBER_AF_MCU_FAMILY
#endif
#define EMBER_AF_MCU_FAMILY F

#ifdef EMBER_AF_MCU_SERIES
#undef EMBER_AF_MCU_SERIES
#endif
#define EMBER_AF_MCU_SERIES 1

#ifdef EMBER_AF_MCU_DEVICE_CONFIGURATION
#undef EMBER_AF_MCU_DEVICE_CONFIGURATION
#endif
#define EMBER_AF_MCU_DEVICE_CONFIGURATION 4

#ifdef EMBER_AF_MCU_PERFORMANCE
#undef EMBER_AF_MCU_PERFORMANCE
#endif
#define EMBER_AF_MCU_PERFORMANCE P

#ifdef EMBER_AF_MCU_RADIO
#undef EMBER_AF_MCU_RADIO
#endif
#define EMBER_AF_MCU_RADIO 233

#ifdef EMBER_AF_MCU_FLASH
#undef EMBER_AF_MCU_FLASH
#endif
#define EMBER_AF_MCU_FLASH 256K

#ifdef EMBER_AF_MCU_TEMP
#undef EMBER_AF_MCU_TEMP
#endif
#define EMBER_AF_MCU_TEMP G

#ifdef EMBER_AF_MCU_PACKAGE
#undef EMBER_AF_MCU_PACKAGE
#endif
#define EMBER_AF_MCU_PACKAGE M

#ifdef EMBER_AF_MCU_PINS
#undef EMBER_AF_MCU_PINS
#endif
#define EMBER_AF_MCU_PINS 48

#ifdef EMBER_AF_BOARD_TYPE
#undef EMBER_AF_BOARD_TYPE
#endif
#define EMBER_AF_BOARD_TYPE BRD4259A



#endif // __FLEX_CONFIG__
