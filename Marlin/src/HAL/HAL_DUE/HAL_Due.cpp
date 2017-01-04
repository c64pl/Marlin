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

/**
 * Public functions
 */

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
}

void HAL_clear_reset_source(void) {}

uint8_t HAL_get_reset_source(void) {
  switch ( (RSTC->RSTC_SR >> 8) & 7)
  {
    case 0: return RST_POWER_ON; break;
    case 1: return RST_BACKUP; break;
    case 2: return RST_WATCHDOG; break;
    case 3: return RST_SOFTWARE; break;
    case 4: return RST_EXTERNAL; break;
    default:
      return 0;
  }
}

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory;
  int heap_end = (int)_sbrk(0);

  if(heap_end == 0)
    free_memory = ((int)&free_memory) - ((int)&_ebss);
  else
    free_memory = ((int)&free_memory) - heap_end;

  return free_memory;
}

// A/D converter
uint16_t getAdcReading(adc_channel_num_t chan) {
  if ((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)) {
    uint16_t rslt = ADC->ADC_CDR[chan];
    SBI(ADC->ADC_CHDR, chan);
    return rslt;
  }
  else {
    SERIAL_ECHOLN("error getAdcReading");
    return 0;
  }
}

void startAdcConversion(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHER, chan);
}

// Convert an Arduino Due pin number to the corresponding ADC channel number
adc_channel_num_t pinToAdcChannel(int pin) {
  if (pin < A0) pin += A0;
  return (adc_channel_num_t) (int)g_APinDescription[pin].ulADCChannelNumber;
}

uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion) {
  if (wait_for_conversion) while (!((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)));
  if ((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)) {
    uint16_t rslt = ADC->ADC_CDR[chan];
    return rslt;
  }
  else {
    SERIAL_ECHOLN("wait freerun");
    return 0;
  }
}

uint16_t getAdcSuperSample(adc_channel_num_t chan) {
  uint16_t rslt = 0;
  for (int i = 0; i < 8; i++) rslt += getAdcFreerun(chan, true);
  return rslt / 4;
}

void setAdcFreerun(void) {
  // ADC_MR_FREERUN_ON: Free Run Mode. It never waits for any trigger.
  ADC->ADC_MR |= ADC_MR_FREERUN_ON | ADC_MR_LOWRES_BITS_12;
}

void stopAdcFreerun(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHDR, chan);
}
#endif // ARDUINO_ARCH_SAM
