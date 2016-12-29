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

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>
#include "Arduino.h"
#if 0
#include "../../../types.h"
#endif
#include "../HAL_fastio.h"
#include "HAL_timers_Due.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define MAX_ANALOG_PIN_NUMBER 11

#define FORCE_INLINE __attribute__((always_inline)) inline

#define CRITICAL_SECTION_START uint32_t primask = __get_PRIMASK(); __disable_irq();
#define CRITICAL_SECTION_END if (primask == 0) __enable_irq();

// On AVR this is in sfr_defs.h
#ifndef _BV
  #define _BV(b) (1 << (b))
#endif

// Variant files of Alligator Board is old
#if MB(ALLIGATOR)
  #define strncpy_P(s1, s2, n) strncpy((s1), (s2), (n))
  #define analogInputToDigitalPin(p)  ((p < 12u) ? (p) + 54u : -1)
#endif

// Fix bug in pgm_read_ptr
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))

#if ENABLED(DELTA_FAST_SQRT)
  #undef ATAN2
  #undef FABS
  #undef POW
  #undef SQRT
  #undef CEIL
  #undef FLOOR
  #undef LROUND
  #undef FMOD
  #define ATAN2(y, x) atan2f(y, x)
  #define FABS(x) fabsf(x)
  #define POW(x, y) powf(x, y)
  #define SQRT(x) sqrtf(x)
  #define CEIL(x) ceilf(x)
  #define FLOOR(x) floorf(x)
  #define LROUND(x) lroundf(x)
  #define FMOD(x, y) fmodf(x, y)
#endif

// Delays
#define CYCLES_EATEN_BY_CODE 12
#define CYCLES_EATEN_BY_E 12

// Voltage
#define LOGIC_VOLTAGE 3.3

// reset reason
#define RST_POWER_ON   1
#define RST_EXTERNAL   2
#define RST_BROWN_OUT  4
#define RST_WATCHDOG   8
#define RST_JTAG      16
#define RST_SOFTWARE  32
#define RST_BACKUP    64

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// Disable interrupts
void cli(void);

// Enable interrupts
void sei(void);

// clear reset reason
void HAL_clear_reset_source(void);

// reset reason
uint8_t HAL_get_reset_source(void);

#if 0
// Delays
static inline void HAL_delay(millis_t ms) {
  unsigned int del;
  while (ms > 0) {
    del = ms > 100 ? 100 : ms;
    delay(del);
    ms -= del;
  }
}
#endif

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

#endif // HAL_DUE_H
