/**
 * Arduino Mega or Due with RAMPS Duo pin assignments
 *
 * Applies to the following boards:
 *
 *  RAMPS_DUO_EFB (Hotend, Fan, Bed)
 *  RAMPS_DUO_EEB (Hotend0, Hotend1, Bed)
 *  RAMPS_DUO_EFF (Hotend, Fan0, Fan1)
 *  RAMPS_DUO_EEF (Hotend0, Hotend1, Fan)
 *  RAMPS_DUO_SF  (Spindle, Controller Fan)
 *
 *  Differences between
 *  RAMPS_14 | RAMPS_DUO
 *    A9/D63 | A12/D66
 *   A10/D64 | A13/D67
 *   A11/D65 | A14/D68
 *   A12/D66 | A15/D69
 *       A13 | A9
 *       A14 | A10
 *       A15 | A11
 */

#if !defined(__SAM3X8E__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Due' or 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME       "RAMPS Duo"
#endif

#define IS_RAMPS_DUO
#include "pins_RAMPS_14.h"

#undef TEMP_0_PIN
#define TEMP_0_PIN          9 // ANALOG NUMBERING

#undef TEMP_1_PIN
#define TEMP_1_PIN         11 // ANALOG NUMBERING

#undef TEMP_BED_PIN
#define TEMP_BED_PIN       10 // ANALOG NUMBERING

#if ENABLED(ULTRA_LCD)
  #if ENABLED(NEWPANEL)
    #if ENABLED(PANEL_ONE)
      #undef LCD_PINS_D4
      #define LCD_PINS_D4  68

      #undef LCD_PINS_D5
      #define LCD_PINS_D5  69

      #undef LCD_PINS_D7
      #define LCD_PINS_D7  67
    #endif

    #if ENABLED(MINIPANEL)
      #undef DOGLCD_CS
      #define DOGLCD_CS    69

      #undef LCD_PIN_BL
      #define LCD_PIN_BL   68 // backlight LED on A14/D68

      #undef KILL_PIN
      #define KILL_PIN     67

      #undef BTN_EN2
      #define BTN_EN2      66
    #else
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #undef BTN_EN1
        #define BTN_EN1    67 // encoder

        #undef BTN_ENC
        #define BTN_ENC    66 // enter button
      #elif ENABLED(PANEL_ONE)
        #undef BTN_EN2
        #define BTN_EN2    66 // AUX2 PIN 4
      #endif
    #endif
  #endif // !NEWPANEL
#endif // ULTRA_LCD

#undef MAX6675_SS
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS       69 // Do not use pin 53 if there is even the remote possibility of using Display/SD card
#else
  #define MAX6675_SS       69 // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
#endif
