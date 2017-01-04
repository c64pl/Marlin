/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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


#ifndef HAL_SPI_PINS_DUE_H
#define HAL_SPI_PINS_DUE_H

/**
 * Includes
 */

#include <stdint.h>

/**
 * Define SPI Pins: SCK, MISO, MOSI, SS
 */
// Available chip select pins for HW SPI are 4 10 52 77
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) || (SDSS == 77)
  #if (SDSS == 4)
    constexpr uint8_t SPI_PIN = 87;
    constexpr uint8_t SPI_CHAN = 1;
  #elif (SDSS == 10)
    constexpr uint8_t SPI_PIN = 77;
    constexpr uint8_t SPI_CHAN = 0;
  #elif (SDSS == 52)
    constexpr uint8_t SPI_PIN = 86;
    constexpr uint8_t SPI_CHAN = 2;
  #else
    constexpr uint8_t SPI_PIN = 77;
    constexpr uint8_t SPI_CHAN = 0;
  #endif
  constexpr uint8_t SCK_PIN = 76;
  constexpr uint8_t MISO_PIN = 74;
  constexpr uint8_t MOSI_PIN = 75;
#else
  #define SOFTWARE_SPI
  constexpr uint8_t SCK_PIN = 52;
  constexpr uint8_t MISO_PIN = 50;
  constexpr uint8_t MOSI_PIN = 51;
#endif
#define SS_PIN SDSS // A.28, A.29, B.21, C.26, C.29

// define software SPI pins
/** Software SPI chip select pin for the SD */
constexpr int8_t SOFT_SPI_CS_PIN = SS_PIN;
/** Software SPI Master Out Slave In pin */
constexpr uint8_t SOFT_SPI_MOSI_PIN = MOSI_PIN;
/** Software SPI Master In Slave Out pin */
constexpr uint8_t SOFT_SPI_MISO_PIN = MISO_PIN;
/** Software SPI Clock pin */
constexpr uint8_t SOFT_SPI_SCK_PIN = SCK_PIN;

#endif // HAL_SPI_PINS_DUE_H
