/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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

#ifndef MARLIN_CONFIG_EXTRA_H
#define MARLIN_CONFIG_EXTRA_H

#if !defined(ARDUINO_ARCH_AVR)
  #define USE_HAL
  #define ADDITIONAL_EXPERIMENTAL_FEATURES
  #include "src/HAL/HAL_MarlinConfig_extra.h"
#else
  //#define USE_HAL
  //#define ADDITIONAL_EXPERIMENTAL_FEATURES
  //#include "src/HAL/HAL_MarlinConfig_extra.h"
#endif

#endif // MARLIN_CONFIG_EXTRA_H
