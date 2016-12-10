/* **************************************************************************

 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

// **************************************************************************
//
// Description:          *** HAL wrapper ***
//
// Supports platforms : 
//     ARDUINO_ARCH_SAM
//     ARDUINO_ARCH_AVR
// **************************************************************************

#ifndef _HAL_H
#define _HAL_H

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define REFERENCE_F_CPU 16000000 // 16MHz MEGA2560

// timers
#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define REFERENCE_EXTRUDER_TIMER_PRESCALE 64
  #define HAL_REFERENCE_EXTRUDER_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_EXTRUDER_TIMER_PRESCALE) // timer0 of MEGA2560: 16000000 / 64 = 250KHz
  #define REFERENCE_EXTRUDER_TIMER_FREQUENCY (HAL_REFERENCE_EXTRUDER_TIMER_RATE / 200) // note: timer0 is in mode3 (8bit fast PWM), 1.25KHz at start 
#endif

#define REFERENCE_STEPPER_TIMER_PRESCALE 8
#define HAL_REFERENCE_STEPPER_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_STEPPER_TIMER_PRESCALE) // timer1 of MEGA2560: 16000000 / 8 = 2MHz
#define REFERENCE_STEPPER_TIMER_FREQUENCY (HAL_REFERENCE_STEPPER_TIMER_RATE / 2000) // note: timer0 is in mode2 (CTC), 1KHz at start

#define REFERENCE_TEMP_TIMER_PRESCALE 64
#define HAL_REFERENCE_TEMP_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_TEMP_TIMER_PRESCALE) // timer0 of MEGA2560: 16000000 / 64 = 250KHz (sharing with advanced extruder)
#define REFERENCE_TEMP_TIMER_FREQUENCY (HAL_REFERENCE_TEMP_TIMER_RATE / 256) // note: timer0 is in mode3 (8bit fast PWM), 976.5625Hz always 

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#ifdef ARDUINO_ARCH_SAM
  #include "HAL_DUE/HAL_Due.h"
/*
#elif defined(ARDUINO_ARCH_AVR)
  #include "HAL_AVR/HAL_AVR.h"
*/
#else
  #error Unsupported Platform!
#endif

#endif // _HAL_H
