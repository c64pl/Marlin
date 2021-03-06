/**
 * DUE3DOM MINI
 * due3dom.pl
 */

#if !defined(ARDUINO_ARCH_SAM)
  #error "Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu."
#endif

#define BOARD_NAME "DUE3DOM MINI"

//
// Servos
//
#define SERVO0_PIN          5
#define SERVO1_PIN          6
#define SERVO2_PIN          8 // 4-pin header FAN0
#define SERVO3_PIN         -1

//
// Limit Switches
//
#define X_MIN_PIN          38
#define X_MAX_PIN          -1
#define Y_MIN_PIN          34
#define Y_MAX_PIN          -1
#define Z_MIN_PIN          30
#define Z_MAX_PIN          -1

//
// Steppers
//
#define X_STEP_PIN         17
#define X_DIR_PIN          16
#define X_ENABLE_PIN       22

#define Y_STEP_PIN          2
#define Y_DIR_PIN           3
#define Y_ENABLE_PIN       26

#define Z_STEP_PIN         64
#define Z_DIR_PIN          63
#define Z_ENABLE_PIN       15

#define E0_STEP_PIN        61
#define E0_DIR_PIN         60
#define E0_ENABLE_PIN      62

//
// Temperature Sensors
//
#define TEMP_0_PIN          0   // Analog Input (HOTEND0 thermistor)
#define TEMP_1_PIN          2   // Analog Input (unused)
#define TEMP_2_PIN          5   // Analog Input (OnBoard thermistor beta 3950)
#define TEMP_BED_PIN        1   // Analog Input (BED thermistor)

// SPI for Max6675 or Max31855 Thermocouple
#define MAX6675_SS         53

//
// Heaters / Fans
//
#define HEATER_0_PIN       13 // HOTEND0 MOSFET
#define HEATER_BED_PIN      7 // BED MOSFET

#define FAN_PIN            11 // FAN1 header on board - PRINT FAN
#define FAN1_PIN           12 // FAN2 header on board - CONTROLLER FAN
#define FAN2_PIN            9 // FAN3 header on board - EXTRUDER0 FAN
//#define FAN3_PIN          8 // FAN0 4-pin header on board

//
// Misc. Functions
//
#define SDSS                4
#define PS_ON_PIN          40

#define I2C_EEPROM

//
// LCD / Controller
//
#if ENABLED(ULTRA_LCD)

  #if DISABLED(SPARK_FULL_GRAPHICS)
    #define LCD_PINS_RS       42
    #define LCD_PINS_ENABLE   43
    #define LCD_PINS_D4       44
  #endif
  #define LCD_PINS_D5         45
  #define LCD_PINS_D6         46
  #define LCD_PINS_D7         47

  #if ENABLED(NEWPANEL)

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define BEEPER_PIN      41

      #define BTN_EN1         50
      #define BTN_EN2         52
      #define BTN_ENC         48

      #define SD_DETECT_PIN   14

    #elif ENABLED(SPARK_FULL_GRAPHICS)
      #define LCD_PINS_RS     25
      #define LCD_PINS_ENABLE 27
      #define LCD_PINS_D4     29

      #define BTN_EN1         35
      #define BTN_EN2         33
      #define BTN_ENC         37

    #elif ENABLED(SSD1306_OLED_I2C_CONTROLLER)
      #define BEEPER_PIN      41

      #define BTN_EN1         50
      #define BTN_EN2         52
      #define BTN_ENC         48

      #define LCD_SDSS         4
      #undef SD_DETECT_PIN

    #endif
  #endif // NEWPANEL

#endif // ULTRA_LCD
