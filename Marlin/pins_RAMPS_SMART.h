/**
 * Arduino Due with RAMPS-SMART pin assignments
 *
 * Applies to the following boards:
 *
 *  RAMPS_SMART_EFB (Hotend, Fan, Bed)
 *  RAMPS_SMART_EEB (Hotend0, Hotend1, Bed)
 *  RAMPS_SMART_EFF (Hotend, Fan0, Fan1)
 *  RAMPS_SMART_EEF (Hotend0, Hotend1, Fan)
 *  RAMPS_SMART_SF  (Spindle, Controller Fan)
 *
 *  Differences between
 *  RAMPS_14 | RAMPS-SMART
 *      NONE | D16 (Additional AUX-3 pin(AUX3_2PIN), shares the same pin with AUX4_18PIN)
 *      NONE | D17 (Additional AUX-3 pin(AUX3_1PIN), shares the same pin with AUX4_17PIN)
 *        D0 | NONE
 *        D1 | NONE
 *    A3/D57 | NONE
 *    A4/D58 | NONE
 *    A5/D59 | A3/D57
 *    A9/D63 | A4/D58
 *   A10/D64 | A5/D59
 *   A11/D65 | D66
 *   A12/D66 | D67
 *       A13 | A9
 *       A14 | A10
 *       A15 | A11
 */

#if !defined(ARDUINO_ARCH_SAM)
  #error "Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME       "RAMPS-SMART"
#endif

#define IS_RAMPS_SMART
#include "pins_RAMPS.h"

//
// Temperature Sensors
//
#undef TEMP_0_PIN
#define TEMP_0_PIN          9   // Analog Input

#undef TEMP_1_PIN
#define TEMP_1_PIN         10   // Analog Input

#undef TEMP_BED_PIN
#define TEMP_BED_PIN       11   // Analog Input

// SPI for Max6675 or Max31855 Thermocouple
#undef MAX6675_SS
#define MAX6675_SS         67

//
// Misc. Functions
//
#define I2C_EEPROM

//
// LCD / Controller
//
#if ENABLED(ULTRA_LCD)
  #if ENABLED(NEWPANEL)
    #if ENABLED(VIKI2)
      #if ENABLED(AZSMZ_12864_LCD)
        #undef BEEPER_PIN
        #define BEEPER_PIN       66

        // Pins for DOGM SPI LCD Support
        #undef DOGLCD_A0
        #define DOGLCD_A0        59

        #undef DOGLCD_CS
        #define DOGLCD_CS        44

        #undef BTN_EN1
        #define BTN_EN1          58

        #undef BTN_EN2
        #define BTN_EN2          40

        #undef BTN_ENC
        #define BTN_ENC          67

        #undef SD_DETECT_PIN
        #define SD_DETECT_PIN    49 // Pin 49 for display sd interface, 72 for easy adapter board

        #undef KILL_PIN
        #define KILL_PIN         42
      #endif
    #endif
  #endif // NEWPANEL
#endif // ULTRA_LCD
