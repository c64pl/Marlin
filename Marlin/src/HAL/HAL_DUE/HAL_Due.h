/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (C) 2015 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (C) 2016 Bob Cousins bobcousins42@googlemail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
 *
 * For ARDUINO_ARCH_SAM
 */

#ifndef HAL_DUE_H
#define HAL_DUE_H

/**
 * Includes
 */

#include <stdint.h>
#include "Arduino.h"
#if 0
#include "../../../types.h"
#endif
#include "../HAL_fastio.h"
#include "HAL_timers_Due.h"

/**
 * Defines
 */

#define CRITICAL_SECTION_START uint32_t primask = __get_PRIMASK(); __disable_irq();
#define CRITICAL_SECTION_END if (primask == 0) __enable_irq();

// On AVR this is in sfr_defs.h
#ifndef _BV
  #define _BV(b) (1 << (b))
#endif

// Variant files of Alligator Board is old
#if MB(ALLIGATOR)
  #define strncpy_P(s1, s2, n) strncpy((s1), (s2), (n))
  #define analogInputToDigitalPin(p) ((p < 12u) ? (p) + 54u : -1)
#endif

// Fix bug in pgm_read_ptr
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))

/**
 * Public Variables
 */

constexpr uint8_t MAX_ANALOG_PIN_NUMBER = 11;

// Delays
constexpr uint8_t CYCLES_EATEN_BY_CODE = 12;
constexpr uint8_t CYCLES_EATEN_BY_E = 12;

// Voltage
constexpr float LOGIC_VOLTAGE = 3.3;

// reset reason
constexpr uint8_t RST_POWER_ON = 1;
constexpr uint8_t RST_EXTERNAL = 2;
constexpr uint8_t RST_BROWN_OUT = 4;
constexpr uint8_t RST_WATCHDOG = 8;
constexpr uint8_t RST_JTAG = 16;
constexpr uint8_t RST_SOFTWARE = 32;
constexpr uint8_t RST_BACKUP = 64;

/**
 * Public functions
 */

// Disable interrupts
void cli(void);

// Enable interrupts
void sei(void);

// clear reset reason
void HAL_clear_reset_source(void);

// reset reason
uint8_t HAL_get_reset_source(void);

// return free memory between end of heap (or end bss) and whatever is current
int freeMemory(void);

// A/D converter
uint16_t getAdcReading(adc_channel_num_t chan);
void startAdcConversion(adc_channel_num_t chan);
adc_channel_num_t pinToAdcChannel(int pin);

uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion = false);
uint16_t getAdcSuperSample(adc_channel_num_t chan);
void setAdcFreerun(void);
void stopAdcFreerun(adc_channel_num_t chan);

#if ENABLED(DELTA_FAST_SQRT)
  #undef ATAN2
  #undef FABS
  #undef POW
  #undef SQRT
  #undef CEIL
  #undef FLOOR
  #undef LROUND
  #undef FMOD

  static FORCE_INLINE float ATAN2(float y, float x) {
    return atan2f(y, x);
  }

  static FORCE_INLINE float FABS(float x) {
    return fabsf(x);
  }

  static FORCE_INLINE float POW(float x, float y) {
    return powf(x, y);
  }

  static FORCE_INLINE float SQRT(float x) {
    return sqrtf(x);
  }

  static FORCE_INLINE float CEIL(float x) {
    return ceilf(x);
  }

  static FORCE_INLINE float FLOOR(float x) {
    return floorf(x);
  }

  static FORCE_INLINE long LROUND(float x) {
    return lroundf(x);
  }

  static FORCE_INLINE float FMOD(float x, float y) {
    return fmodf(x, y);
  }
#endif

#endif // HAL_DUE_H
