/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if defined(ARDUINO_ARCH_SAM)

/**
 * Includes
 */

#include "../../../Marlin.h"
#include "HAL_timers_Due.h"

/**
 * Public functions
 */

// Timers
/*
  Timer_clock1: Prescaler 2 -> 42MHz
  Timer_clock2: Prescaler 8 -> 10.5MHz
  Timer_clock3: Prescaler 32 -> 2.625MHz
  Timer_clock4: Prescaler 128 -> 656.25kHz
*/

// new timer by Ps991
// thanks for that work
// http://forum.arduino.cc/index.php?topic=297397.0

void HAL_timer_start(const uint8_t timer_num, const uint8_t priority, const uint32_t frequency, const uint32_t clock, const uint8_t prescale) {
  // Get the ISR from table
  Tc *tc = TimerConfig[timer_num].pTimerRegs;
  uint32_t channel = TimerConfig[timer_num].channel;
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority(irq, priority);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);

  TC_SetRC(tc, channel, VARIANT_MCK / prescale / frequency);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IDR = TC_IER_CPCS; // disable interrupt

  NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt
}

#if ENABLED(USE_WATCHDOG)
  // this function has to be present, otherwise watchdog won't work
  void watchdogSetup(void) {
    // do what you want here
  }

  #if ENABLED(WATCHDOG_RESET_MANUAL)
    void HAL_watchdog_timer_enable_interrupt(uint32_t timeout) {
      /* this assumes the slow clock is running at 32.768 kHz
         watchdog frequency is therefore 32768 / 128 = 256 Hz */
      timeout = timeout * 256 / 1000;
      if (timeout == 0)
        timeout = 1;
      else if (timeout > 0xFFF)
        timeout = 0xFFF;

      /* Enable WDT interrupt line from the core */
      NVIC_DisableIRQ(WDT_IRQn);
      NVIC_ClearPendingIRQ(WDT_IRQn);
      NVIC_SetPriority(WDT_IRQn, WATCHDOG_TIMER_PRIORITY);
      NVIC_EnableIRQ(WDT_IRQn);

      WDT_Enable(WDT, WDT_MR_WDFIEN | WDT_MR_WDV(timeout) | WDT_MR_WDD(timeout));
    }
  #endif
#endif

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IDR = TC_IER_CPCS; // disable interrupt
}

#endif // ARDUINO_ARCH_SAM
