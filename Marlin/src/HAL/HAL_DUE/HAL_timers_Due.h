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

/**
 * Includes
 */

#include <stdint.h>

/**
 * Defines
 */

// timers
#define STEPPER_TIMER 2
#define STEPPER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor

#define TEMP_TIMER 3
#define TEMP_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK2 // TIMER_CLOCK2 -> 8 divisor

#define HAL_ISR_WATCHDOG_TIMER void WDT_Handler()

#define _HAL_ISR(p) void TC ## p ## _Handler()
#define HAL_ISR(p) _HAL_ISR(p)
#define HAL_TIMER_START(n) HAL_timer_start(n, n ## _PRIORITY, n ## _FREQUENCY, n ## _CLOCK, n ## _PRESCALE)

/**
 * Types
 */

// timers
typedef uint32_t HAL_TIMER_TYPE;

typedef struct {
  Tc* pTimerRegs;
  uint16_t channel;
  IRQn_Type IRQ_Id;
} tTimerConfig;

/**
 * Public Variables
 */

// timers
constexpr uint8_t NUM_HARDWARE_TIMERS = 9;

constexpr uint32_t STEPPER_TIMER_PRIORITY = 2;
constexpr double STEPPER_TIMER_FREQUENCY = REFERENCE_STEPPER_TIMER_FREQUENCY;
constexpr uint32_t STEPPER_TIMER_PRESCALE = 2;
constexpr double HAL_STEPPER_TIMER_RATE = F_CPU / STEPPER_TIMER_PRESCALE; // = 42MHz
constexpr double STEPPER_TIMER_FREQUENCY_FACTOR = STEPPER_TIMER_FREQUENCY / REFERENCE_STEPPER_TIMER_FREQUENCY;
constexpr double STEPPER_TIMER_FACTOR = HAL_STEPPER_TIMER_RATE / HAL_REFERENCE_STEPPER_TIMER_RATE / STEPPER_TIMER_FREQUENCY_FACTOR;
constexpr double STEPPER_TIMER_TICKS_PER_MILLISECOND = HAL_STEPPER_TIMER_RATE / 1000;

constexpr uint32_t TEMP_TIMER_PRIORITY = 15;
constexpr double TEMP_TIMER_FREQUENCY = REFERENCE_TEMP_TIMER_FREQUENCY;
constexpr uint32_t TEMP_TIMER_PRESCALE = 8;
constexpr double HAL_TEMP_TIMER_RATE = F_CPU / TEMP_TIMER_PRESCALE; // = 10.5MHz
constexpr double TEMP_TIMER_FREQUENCY_FACTOR = TEMP_TIMER_FREQUENCY / REFERENCE_TEMP_TIMER_FREQUENCY;
constexpr double TEMP_TIMER_FACTOR = HAL_TEMP_TIMER_RATE / HAL_REFERENCE_TEMP_TIMER_RATE / TEMP_TIMER_FREQUENCY_FACTOR;
constexpr double TEMP_TIMER_TICKS_PER_MILLISECOND = HAL_TEMP_TIMER_RATE / 1000;

constexpr uint32_t WATCHDOG_TIMER_PRIORITY = 1;

constexpr HAL_TIMER_TYPE ADV_NEVER = UINT32_MAX;

/**
 * Private Variables
 */

// this should not be written in .h, but I can not solve compilation error
static constexpr tTimerConfig TimerConfig[NUM_HARDWARE_TIMERS] = {
  { TC0, 0, TC0_IRQn },
  { TC0, 1, TC1_IRQn },
  { TC0, 2, TC2_IRQn },
  { TC1, 0, TC3_IRQn },
  { TC1, 1, TC4_IRQn },
  { TC1, 2, TC5_IRQn },
  { TC2, 0, TC6_IRQn },
  { TC2, 1, TC7_IRQn },
  { TC2, 2, TC8_IRQn }
};

/**
 * Public functions
 */

// Timers
void HAL_timer_start(const uint8_t timer_num, const uint8_t priority, const uint32_t frequency, const uint32_t clock, const uint8_t prescale);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
#if ENABLED(USE_WATCHDOG)
  void watchdogSetup(void);
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    void HAL_watchdog_timer_enable_interrupt(const uint32_t timeout);
  #endif
#endif
void HAL_timer_disable_interrupt(const uint8_t timer_num);

static FORCE_INLINE void HAL_timer_isr_prologue(const uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  TC_GetStatus(pConfig->pTimerRegs, pConfig->channel); // clear status register
}

static FORCE_INLINE uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

static FORCE_INLINE uint32_t HAL_timer_get_current_count(const uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return TC_ReadCV(pConfig->pTimerRegs, pConfig->channel);
}

static FORCE_INLINE void HAL_timer_set_count(const uint8_t timer_num, uint32_t count) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  TC_SetRC(pConfig->pTimerRegs, pConfig->channel, count);
}

static FORCE_INLINE void ENABLE_STEPPER_DRIVER_INTERRUPT(void) {
  HAL_timer_enable_interrupt(STEPPER_TIMER);
}

static FORCE_INLINE void DISABLE_STEPPER_DRIVER_INTERRUPT(void) {
  HAL_timer_disable_interrupt(STEPPER_TIMER);
}

static FORCE_INLINE void HAL_TIMER_SET_STEPPER_COUNT(uint32_t count) {
  HAL_timer_set_count(STEPPER_TIMER, count);
}

static FORCE_INLINE void ENABLE_TEMP_INTERRUPT(void) {
  HAL_timer_enable_interrupt(TEMP_TIMER);
}

static FORCE_INLINE void DISABLE_TEMP_INTERRUPT(void) {
  HAL_timer_disable_interrupt(TEMP_TIMER);
}

static FORCE_INLINE void HAL_TIMER_SET_TEMP_COUNT(uint32_t count) {
  HAL_timer_set_count(TEMP_TIMER, count);
}

#endif // HAL_TIMERS_DUE_H
