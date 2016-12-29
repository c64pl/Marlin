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

#ifndef HAL_TIMERS_DUE_H
#define HAL_TIMERS_DUE_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

// timers
#define HAL_TIMER_TYPE uint32_t
#define NUM_HARDWARE_TIMERS 9

#define STEPPER_TIMER 2
#define STEPPER_TIMER_PRIORITY 2
#define STEPPER_TIMER_FREQUENCY REFERENCE_STEPPER_TIMER_FREQUENCY
#define STEPPER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor
#define STEPPER_TIMER_PRESCALE 2
#define HAL_STEPPER_TIMER_RATE (F_CPU / STEPPER_TIMER_PRESCALE) // = 42MHz
#define STEPPER_TIMER_FREQUENCY_FACTOR (STEPPER_TIMER_FREQUENCY / REFERENCE_STEPPER_TIMER_FREQUENCY)
#define STEPPER_TIMER_FACTOR (HAL_STEPPER_TIMER_RATE / HAL_REFERENCE_STEPPER_TIMER_RATE / STEPPER_TIMER_FREQUENCY_FACTOR)
#define STEPPER_TIMER_TICKS_PER_MILLISECOND (HAL_STEPPER_TIMER_RATE / 1000)

#define TEMP_TIMER 3
#define TEMP_TIMER_PRIORITY 15
#define TEMP_TIMER_FREQUENCY REFERENCE_TEMP_TIMER_FREQUENCY
#define TEMP_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK2 // TIMER_CLOCK2 -> 8 divisor
#define TEMP_TIMER_PRESCALE 8
#define HAL_TEMP_TIMER_RATE (F_CPU / TEMP_TIMER_PRESCALE) // = 10.5MHz
#define TEMP_TIMER_FREQUENCY_FACTOR (TEMP_TIMER_FREQUENCY / REFERENCE_TEMP_TIMER_FREQUENCY)
#define TEMP_TIMER_FACTOR (HAL_TEMP_TIMER_RATE / HAL_REFERENCE_TEMP_TIMER_RATE / TEMP_TIMER_FREQUENCY_FACTOR)
#define TEMP_TIMER_TICKS_PER_MILLISECOND (HAL_TEMP_TIMER_RATE / 1000)

#define HAL_ISR_WATCHDOG_TIMER void WDT_Handler()
#define WATCHDOG_TIMER_PRIORITY 1

#define _HAL_ISR(p) void TC ## p ## _Handler()
#define HAL_ISR(p) _HAL_ISR(p)
#define HAL_TIMER_START(n) HAL_timer_start(n, n ## _PRIORITY, n ## _FREQUENCY, n ## _CLOCK, n ## _PRESCALE)

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEPPER_TIMER)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEPPER_TIMER)
#define HAL_TIMER_SET_STEPPER_COUNT(n) HAL_timer_set_count(STEPPER_TIMER, n)

#define ENABLE_TEMP_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER)
#define DISABLE_TEMP_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER)
#define HAL_TIMER_SET_TEMP_COUNT(n) HAL_timer_set_count(TEMP_TIMER, n)

#if 0
#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define EXTRUDER_TIMER 1
  #define EXTRUDER_TIMER_PRIORITY 6
  #define EXTRUDER_TIMER_FREQUENCY REFERENCE_EXTRUDER_TIMER_FREQUENCY
  #define EXTRUDER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor
  #define EXTRUDER_TIMER_PRESCALE 2
  #define HAL_EXTRUDER_TIMER_RATE (F_CPU / EXTRUDER_TIMER_PRESCALE) // = 42MHz
  #define EXTRUDER_TIMER_FACTOR (HAL_EXTRUDER_TIMER_RATE / HAL_REFERENCE_EXTRUDER_TIMER_RATE)
  #define EXTRUDER_TIMER_TICKS_PER_MILLISECOND (HAL_EXTRUDER_TIMER_RATE / 1000)
#endif

#define BEEPER_TIMER 4
#define BEEPER_TIMER_PRIORITY NVIC_EncodePriority(4, 6, 3)
#define BEEPER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK4 // TIMER_CLOCK4 -> 128 divisor
#define BEEPER_TIMER_PRESCALE 128
#define HAL_BEEPER_TIMER_RATE (F_CPU / BEEPER_TIMER_PRESCALE)
#define BEEPER_TIMER_TICKS_PER_MILLISECOND (HAL_BEEPER_TIMER_RATE / 1000)

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define ENABLE_EXTRUDER_INTERRUPT() HAL_timer_enable_interrupt(EXTRUDER_TIMER)
  #define DISABLE_EXTRUDER_INTERRUPT() HAL_timer_disable_interrupt(EXTRUDER_TIMER)
  #define HAL_TIMER_SET_EXTRUDER_COUNT(n) HAL_timer_set_count(EXTRUDER_TIMER, n)
#endif
#endif

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// timers
typedef struct {
  Tc          *pTimerRegs;
  uint16_t    channel;
  IRQn_Type   IRQ_Id;
} tTimerConfig;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

constexpr HAL_TIMER_TYPE ADV_NEVER = UINT32_MAX;

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// this should not be written in .h, but I can not solve compilation error
static const tTimerConfig TimerConfig[NUM_HARDWARE_TIMERS] =
{
  { TC0, 0, TC0_IRQn},
  { TC0, 1, TC1_IRQn},
  { TC0, 2, TC2_IRQn},
  { TC1, 0, TC3_IRQn},
  { TC1, 1, TC4_IRQn},
  { TC1, 2, TC5_IRQn},
  { TC2, 0, TC6_IRQn},
  { TC2, 1, TC7_IRQn},
  { TC2, 2, TC8_IRQn},
};

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// Timers
void HAL_timer_start(uint8_t timer_num, uint8_t priority, uint32_t frequency, uint32_t clock, uint8_t prescale);
void HAL_timer_enable_interrupt(uint8_t timer_num);
#if ENABLED(USE_WATCHDOG)
  void watchdogSetup(void);
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    void HAL_watchdog_timer_enable_interrupt(uint32_t timeout);
  #endif
#endif
void HAL_timer_disable_interrupt(uint8_t timer_num);

static FORCE_INLINE void HAL_timer_isr_prologue(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  TC_GetStatus(pConfig->pTimerRegs, pConfig->channel); // clear status register
}

static FORCE_INLINE uint32_t HAL_timer_get_count(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

static FORCE_INLINE uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return TC_ReadCV(pConfig->pTimerRegs, pConfig->channel);
}

static FORCE_INLINE void HAL_timer_set_count(uint8_t timer_num, uint32_t count) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  TC_SetRC(pConfig->pTimerRegs, pConfig->channel, count);
}

#if 0
#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  void HAL_extruder_timer_start(void);
#endif
void HAL_step_timer_start(void);
void HAL_temp_timer_start(void);

static FORCE_INLINE void HAL_timer_isr_status(Tc* tc, uint32_t channel) {
  TC_GetStatus(tc, channel); // clear status register
}

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  static FORCE_INLINE void HAL_extruder_count(uint32_t count) {
    // Get the ISR from table
    Tc *tc = TimerConfig[EXTRUDER_TIMER].pTimerRegs;
    uint32_t channel = TimerConfig[EXTRUDER_TIMER].channel;
    uint32_t counter_value = TC_ReadCV(tc, channel) + 2 * EXTRUDER_TIMER_FACTOR; // we need time for other stuff!

    //if (count < 105) count = 105;
    TC_SetRC(tc, channel, (counter_value <= count) ? count : counter_value);
  }
#endif

static FORCE_INLINE void HAL_timer_stepper_count(uint32_t count) {
  // Get the ISR from table
  Tc *tc = TimerConfig[STEPPER_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[STEPPER_TIMER].channel;
  uint32_t counter_value = TC_ReadCV(tc, channel) + 2 * STEPPER_TIMER_FACTOR; // we need time for other stuff!

  //if (count < 105) count = 105;
  TC_SetRC(tc, channel, (counter_value <= count) ? count : counter_value);
}

void tone(uint8_t pin, int frequency);
void noTone(uint8_t pin);
//void tone(uint8_t pin, int frequency, long duration);
#endif

#endif // HAL_TIMERS_DUE_H
