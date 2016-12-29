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

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../Marlin.h"
#include "HAL_timers_Due.h"

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

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

void HAL_timer_start(uint8_t timer_num, uint8_t priority, uint32_t frequency, uint32_t clock, uint8_t prescale) {
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

void HAL_timer_enable_interrupt(uint8_t timer_num) {
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

void HAL_timer_disable_interrupt(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IDR = TC_IER_CPCS; // disable interrupt
}

#if 0
// from DueTimer by Ivan Seidel
// https://github.com/ivanseidel/DueTimer
uint8_t bestClock(double frequency, uint32_t& retRC){
  /*
    Pick the best Clock, thanks to Ogle Basil Hall!
    Timer         Definition
    TIMER_CLOCK1  MCK /  2
    TIMER_CLOCK2  MCK /  8
    TIMER_CLOCK3  MCK / 32
    TIMER_CLOCK4  MCK /128
  */
  const struct {
    uint8_t flag;
    uint8_t divisor;
  } clockConfig[] = {
    { TC_CMR_TCCLKS_TIMER_CLOCK1,   2 },
    { TC_CMR_TCCLKS_TIMER_CLOCK2,   8 },
    { TC_CMR_TCCLKS_TIMER_CLOCK3,  32 },
    { TC_CMR_TCCLKS_TIMER_CLOCK4, 128 }
  };
  float ticks;
  float error;
  int clkId = 3;
  int bestClock = 3;
  float bestError = 9.999e99;
  do
  {
    ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[clkId].divisor;
    // error = abs(ticks - round(ticks));
    error = clockConfig[clkId].divisor * abs(ticks - round(ticks));  // Error comparison needs scaling
    if (error < bestError)
    {
      bestClock = clkId;
      bestError = error;
    }
  } while (clkId-- > 0);
  ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[bestClock].divisor;
  retRC = (uint32_t) round(ticks);
  return clockConfig[bestClock].flag;
}

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  void HAL_extruder_timer_start() {
    // Get the ISR from table
    Tc *tc = TimerConfig[EXTRUDER_TIMER].pTimerRegs;
    uint32_t channel = TimerConfig[EXTRUDER_TIMER].channel;
    IRQn_Type irq = TimerConfig[EXTRUDER_TIMER].IRQ_Id;

    pmc_set_writeprotect(false); // remove write protection on registers
    pmc_enable_periph_clk((uint32_t)irq);
    NVIC_SetPriority(irq, EXTRUDER_TIMER_PRIORITY);

    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | EXTRUDER_TIMER_CLOCK);

    TC_SetRC(tc, channel, VARIANT_MCK / EXTRUDER_TIMER_PRESCALE / EXTRUDER_TIMER_FREQUENCY); // start with EXTRUDER_TIMER_FREQUENCY(Hz) as frequency; //interrupt occurs every x interations of the timer counter
    TC_Start(tc, channel);

    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt

    NVIC_EnableIRQ(irq); // enable Nested Vector Interrupt Controller
  }
#endif

void HAL_step_timer_start() {
  // Timer for stepper
  // Timer 3 HAL.h STEPPER_TIMER
  // uint8_t timer_num = STEPPER_TIMER;

  // Get the ISR from table
  Tc *tc = TimerConfig[STEPPER_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[STEPPER_TIMER].channel;
  IRQn_Type irq = TimerConfig[STEPPER_TIMER].IRQ_Id;

  pmc_set_writeprotect(false); //remove write protection on registers
  pmc_enable_periph_clk((uint32_t)irq); //we need a clock?
  NVIC_SetPriority(irq, STEPPER_TIMER_PRIORITY);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | STEPPER_TIMER_CLOCK);

  TC_SetRC(tc, channel, VARIANT_MCK / STEPPER_TIMER_PRESCALE / STEPPER_TIMER_FREQUENCY); // start with STEPPER_TIMER_FREQUENCY(Hz) as frequency; //interrupt occurs every x interations of the timer counter
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt

  NVIC_EnableIRQ(irq); //enable Nested Vector Interrupt Controller
}

void HAL_temp_timer_start() {
  // Get the ISR from table
  Tc *tc = TimerConfig[TEMP_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[TEMP_TIMER].channel;
  IRQn_Type irq = TimerConfig[TEMP_TIMER].IRQ_Id;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority(irq, TEMP_TIMER_PRIORITY);

  TC_Configure(tc, channel, TC_CMR_CPCTRG | TEMP_TIMER_CLOCK);

  TC_SetRC(tc, channel, VARIANT_MCK / TEMP_TIMER_PRESCALE / TEMP_TIMER_FREQUENCY); // start with TEMP_TIMER_FREQUENCY(Hz) as frequency; //interrupt occurs every x interations of the timer counter
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt

  NVIC_EnableIRQ(irq);
}

// Due have no tone, this is from Repetier 0.92.3
static uint32_t tone_pin = 0;

void tone(uint8_t pin, int frequency) {
  // set up timer counter 1 channel 0 to generate interrupts for
  // toggling output pin.

  /*TC1, 1, TC4_IRQn*/
  Tc *tc = TimerConfig[BEEPER_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig[BEEPER_TIMER].channel;
  IRQn_Type irq = TimerConfig[BEEPER_TIMER].IRQ_Id;

  uint32_t rc = VARIANT_MCK / BEEPER_TIMER_PRESCALE / frequency;

  SET_OUTPUT(pin);
  tone_pin = pin;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  // set interrupt to lowest possible priority
  NVIC_SetPriority(irq, BEEPER_TIMER_PRIORITY);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | BEEPER_TIMER_CLOCK);

  TC_SetRA(tc, channel, rc / 2); // 50% duty cycle
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;

  NVIC_EnableIRQ(irq);
}

void noTone(uint8_t pin) {
  const tTimerConfig *pConfig = &TimerConfig[BEEPER_TIMER];

  TC_Stop(pConfig->pTimerRegs, pConfig->channel);
  WRITE_VAR(pin, LOW);
}


// IRQ handler for tone generator
HAL_ISR(BEEPER_TIMER) {
    static bool toggle = 0;

    HAL_timer_isr_prologue(BEEPER_TIMER);
    WRITE_VAR(tone_pin, toggle);
    toggle = !toggle;
}
#endif

#endif // ARDUINO_ARCH_SAM
