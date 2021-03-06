/**
 * RAMPS-FD v2
 *
 * EEPROM supported
 * Use 1k thermistor tables
 */

#define BOARD_NAME         "RAMPS-FD v2"

#include "pins_RAMPS_FD.h"

//
// Heaters / Fans
//
#undef INVERTED_HEATER_PINS
#undef INVERTED_BED_PINS
#undef INVERTED_FAN_PINS

//
// Misc. Functions
//
#define I2C_EEPROM
