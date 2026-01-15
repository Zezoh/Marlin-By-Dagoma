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

/**
 * Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "Marlin.h"
#if ENABLED(SDSUPPORT)

#ifndef sd_card_hardware_h
#define sd_card_hardware_h

#include "macros.h"
#include <avr/io.h>

//==============================================================================
// SD CARD HARDWARE CONFIGURATION
//==============================================================================
/**
 * SPI init rate for SD initialization commands. Must be 5 (F_CPU/64)
 * or 6 (F_CPU/128).
 */
#define SPI_SD_INIT_RATE 5
// Software SPI is disabled - using hardware SPI only
// MEGA_SOFT_SPI disabled - using hardware SPI only

//==============================================================================
// SD2PINMAP - Pin mapping structures and functions
//==============================================================================

/** struct for mapping digital pins */
struct pin_map_t {
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  volatile uint8_t* port;
  uint8_t bit;
};
//------------------------------------------------------------------------------
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Mega

// Two Wire (aka I2C) ports
uint8_t const SDA_PIN = 20;  // D1
uint8_t const SCL_PIN = 21;  // D0

#undef MOSI_PIN
#undef MISO_PIN
#undef SCK_PIN
// SPI port
uint8_t const SS_PIN = 53;    // B0
uint8_t const MOSI_PIN = 51;  // B2
uint8_t const MISO_PIN = 50;  // B3
uint8_t const SCK_PIN = 52;   // B1

static const pin_map_t digitalPinMap[] = {
  {&DDRE, &PINE, &PORTE, 0},  // E0  0
  {&DDRE, &PINE, &PORTE, 1},  // E1  1
  {&DDRE, &PINE, &PORTE, 4},  // E4  2
  {&DDRE, &PINE, &PORTE, 5},  // E5  3
  {&DDRG, &PING, &PORTG, 5},  // G5  4
  {&DDRE, &PINE, &PORTE, 3},  // E3  5
  {&DDRH, &PINH, &PORTH, 3},  // H3  6
  {&DDRH, &PINH, &PORTH, 4},  // H4  7
  {&DDRH, &PINH, &PORTH, 5},  // H5  8
  {&DDRH, &PINH, &PORTH, 6},  // H6  9
  {&DDRB, &PINB, &PORTB, 4},  // B4 10
  {&DDRB, &PINB, &PORTB, 5},  // B5 11
  {&DDRB, &PINB, &PORTB, 6},  // B6 12
  {&DDRB, &PINB, &PORTB, 7},  // B7 13
  {&DDRJ, &PINJ, &PORTJ, 1},  // J1 14
  {&DDRJ, &PINJ, &PORTJ, 0},  // J0 15
  {&DDRH, &PINH, &PORTH, 1},  // H1 16
  {&DDRH, &PINH, &PORTH, 0},  // H0 17
  {&DDRD, &PIND, &PORTD, 3},  // D3 18
  {&DDRD, &PIND, &PORTD, 2},  // D2 19
  {&DDRD, &PIND, &PORTD, 1},  // D1 20
  {&DDRD, &PIND, &PORTD, 0},  // D0 21
  {&DDRA, &PINA, &PORTA, 0},  // A0 22
  {&DDRA, &PINA, &PORTA, 1},  // A1 23
  {&DDRA, &PINA, &PORTA, 2},  // A2 24
  {&DDRA, &PINA, &PORTA, 3},  // A3 25
  {&DDRA, &PINA, &PORTA, 4},  // A4 26
  {&DDRA, &PINA, &PORTA, 5},  // A5 27
  {&DDRA, &PINA, &PORTA, 6},  // A6 28
  {&DDRA, &PINA, &PORTA, 7},  // A7 29
  {&DDRC, &PINC, &PORTC, 7},  // C7 30
  {&DDRC, &PINC, &PORTC, 6},  // C6 31
  {&DDRC, &PINC, &PORTC, 5},  // C5 32
  {&DDRC, &PINC, &PORTC, 4},  // C4 33
  {&DDRC, &PINC, &PORTC, 3},  // C3 34
  {&DDRC, &PINC, &PORTC, 2},  // C2 35
  {&DDRC, &PINC, &PORTC, 1},  // C1 36
  {&DDRC, &PINC, &PORTC, 0},  // C0 37
  {&DDRD, &PIND, &PORTD, 7},  // D7 38
  {&DDRG, &PING, &PORTG, 2},  // G2 39
  {&DDRG, &PING, &PORTG, 1},  // G1 40
  {&DDRG, &PING, &PORTG, 0},  // G0 41
  {&DDRL, &PINL, &PORTL, 7},  // L7 42
  {&DDRL, &PINL, &PORTL, 6},  // L6 43
  {&DDRL, &PINL, &PORTL, 5},  // L5 44
  {&DDRL, &PINL, &PORTL, 4},  // L4 45
  {&DDRL, &PINL, &PORTL, 3},  // L3 46
  {&DDRL, &PINL, &PORTL, 2},  // L2 47
  {&DDRL, &PINL, &PORTL, 1},  // L1 48
  {&DDRL, &PINL, &PORTL, 0},  // L0 49
  {&DDRB, &PINB, &PORTB, 3},  // B3 50
  {&DDRB, &PINB, &PORTB, 2},  // B2 51
  {&DDRB, &PINB, &PORTB, 1},  // B1 52
  {&DDRB, &PINB, &PORTB, 0},  // B0 53
  {&DDRF, &PINF, &PORTF, 0},  // F0 54
  {&DDRF, &PINF, &PORTF, 1},  // F1 55
  {&DDRF, &PINF, &PORTF, 2},  // F2 56
  {&DDRF, &PINF, &PORTF, 3},  // F3 57
  {&DDRF, &PINF, &PORTF, 4},  // F4 58
  {&DDRF, &PINF, &PORTF, 5},  // F5 59
  {&DDRF, &PINF, &PORTF, 6},  // F6 60
  {&DDRF, &PINF, &PORTF, 7},  // F7 61
  {&DDRK, &PINK, &PORTK, 0},  // K0 62
  {&DDRK, &PINK, &PORTK, 1},  // K1 63
  {&DDRK, &PINK, &PORTK, 2},  // K2 64
  {&DDRK, &PINK, &PORTK, 3},  // K3 65
  {&DDRK, &PINK, &PORTK, 4},  // K4 66
  {&DDRK, &PINK, &PORTK, 5},  // K5 67
  {&DDRK, &PINK, &PORTK, 6},  // K6 68
  {&DDRK, &PINK, &PORTK, 7}   // K7 69
};
//------------------------------------------------------------------------------
#elif defined(__AVR_ATmega644P__)\
|| defined(__AVR_ATmega644__)\
|| defined(__AVR_ATmega1284P__)
// Sanguino

// Two Wire (aka I2C) ports
uint8_t const SDA_PIN = 17;  // C1
uint8_t const SCL_PIN = 18;  // C2

// SPI port
uint8_t const SS_PIN = 4;    // B4
uint8_t const MOSI_PIN = 5;  // B5
uint8_t const MISO_PIN = 6;  // B6
uint8_t const SCK_PIN = 7;   // B7

static const pin_map_t digitalPinMap[] = {
  {&DDRB, &PINB, &PORTB, 0},  // B0  0
  {&DDRB, &PINB, &PORTB, 1},  // B1  1
  {&DDRB, &PINB, &PORTB, 2},  // B2  2
  {&DDRB, &PINB, &PORTB, 3},  // B3  3
  {&DDRB, &PINB, &PORTB, 4},  // B4  4
  {&DDRB, &PINB, &PORTB, 5},  // B5  5
  {&DDRB, &PINB, &PORTB, 6},  // B6  6
  {&DDRB, &PINB, &PORTB, 7},  // B7  7
  {&DDRD, &PIND, &PORTD, 0},  // D0  8
  {&DDRD, &PIND, &PORTD, 1},  // D1  9
  {&DDRD, &PIND, &PORTD, 2},  // D2 10
  {&DDRD, &PIND, &PORTD, 3},  // D3 11
  {&DDRD, &PIND, &PORTD, 4},  // D4 12
  {&DDRD, &PIND, &PORTD, 5},  // D5 13
  {&DDRD, &PIND, &PORTD, 6},  // D6 14
  {&DDRD, &PIND, &PORTD, 7},  // D7 15
  {&DDRC, &PINC, &PORTC, 0},  // C0 16
  {&DDRC, &PINC, &PORTC, 1},  // C1 17
  {&DDRC, &PINC, &PORTC, 2},  // C2 18
  {&DDRC, &PINC, &PORTC, 3},  // C3 19
  {&DDRC, &PINC, &PORTC, 4},  // C4 20
  {&DDRC, &PINC, &PORTC, 5},  // C5 21
  {&DDRC, &PINC, &PORTC, 6},  // C6 22
  {&DDRC, &PINC, &PORTC, 7},  // C7 23
  {&DDRA, &PINA, &PORTA, 7},  // A7 24
  {&DDRA, &PINA, &PORTA, 6},  // A6 25
  {&DDRA, &PINA, &PORTA, 5},  // A5 26
  {&DDRA, &PINA, &PORTA, 4},  // A4 27
  {&DDRA, &PINA, &PORTA, 3},  // A3 28
  {&DDRA, &PINA, &PORTA, 2},  // A2 29
  {&DDRA, &PINA, &PORTA, 1},  // A1 30
  {&DDRA, &PINA, &PORTA, 0}   // A0 31
};
//------------------------------------------------------------------------------
#elif defined(__AVR_ATmega32U4__)
// Teensy 2.0

// Two Wire (aka I2C) ports
uint8_t const SDA_PIN = 6;  // D1
uint8_t const SCL_PIN = 5;  // D0

// SPI port
uint8_t const SS_PIN = 0;    // B0
uint8_t const MOSI_PIN = 2;  // B2
uint8_t const MISO_PIN = 3;  // B3
uint8_t const SCK_PIN = 1;   // B1

static const pin_map_t digitalPinMap[] = {
  {&DDRB, &PINB, &PORTB, 0},  // B0  0
  {&DDRB, &PINB, &PORTB, 1},  // B1  1
  {&DDRB, &PINB, &PORTB, 2},  // B2  2
  {&DDRB, &PINB, &PORTB, 3},  // B3  3
  {&DDRB, &PINB, &PORTB, 7},  // B7  4
  {&DDRD, &PIND, &PORTD, 0},  // D0  5
  {&DDRD, &PIND, &PORTD, 1},  // D1  6
  {&DDRD, &PIND, &PORTD, 2},  // D2  7
  {&DDRD, &PIND, &PORTD, 3},  // D3  8
  {&DDRC, &PINC, &PORTC, 6},  // C6  9
  {&DDRC, &PINC, &PORTC, 7},  // C7 10
  {&DDRD, &PIND, &PORTD, 6},  // D6 11
  {&DDRD, &PIND, &PORTD, 7},  // D7 12
  {&DDRB, &PINB, &PORTB, 4},  // B4 13
  {&DDRB, &PINB, &PORTB, 5},  // B5 14
  {&DDRB, &PINB, &PORTB, 6},  // B6 15
  {&DDRF, &PINF, &PORTF, 7},  // F7 16
  {&DDRF, &PINF, &PORTF, 6},  // F6 17
  {&DDRF, &PINF, &PORTF, 5},  // F5 18
  {&DDRF, &PINF, &PORTF, 4},  // F4 19
  {&DDRF, &PINF, &PORTF, 1},  // F1 20
  {&DDRF, &PINF, &PORTF, 0},  // F0 21
  {&DDRD, &PIND, &PORTD, 4},  // D4 22
  {&DDRD, &PIND, &PORTD, 5},  // D5 23
  {&DDRE, &PINE, &PORTE, 6}   // E6 24
};
//------------------------------------------------------------------------------
#elif defined(__AVR_AT90USB646__)\
|| defined(__AVR_AT90USB1286__)
// Teensy++ 1.0 & 2.0

// Two Wire (aka I2C) ports
uint8_t const SDA_PIN = 1;  // D1
uint8_t const SCL_PIN = 0;  // D0

// SPI port
uint8_t const SS_PIN    = 20;    // B0
uint8_t const MOSI_PIN  = 22;    // B2
uint8_t const MISO_PIN  = 23;    // B3
uint8_t const SCK_PIN   = 21;    // B1

static const pin_map_t digitalPinMap[] = {
  {&DDRD, &PIND, &PORTD, 0},  // D0  0
  {&DDRD, &PIND, &PORTD, 1},  // D1  1
  {&DDRD, &PIND, &PORTD, 2},  // D2  2
  {&DDRD, &PIND, &PORTD, 3},  // D3  3
  {&DDRD, &PIND, &PORTD, 4},  // D4  4
  {&DDRD, &PIND, &PORTD, 5},  // D5  5
  {&DDRD, &PIND, &PORTD, 6},  // D6  6
  {&DDRD, &PIND, &PORTD, 7},  // D7  7
  {&DDRE, &PINE, &PORTE, 0},  // E0  8
  {&DDRE, &PINE, &PORTE, 1},  // E1  9
  {&DDRC, &PINC, &PORTC, 0},  // C0 10
  {&DDRC, &PINC, &PORTC, 1},  // C1 11
  {&DDRC, &PINC, &PORTC, 2},  // C2 12
  {&DDRC, &PINC, &PORTC, 3},  // C3 13
  {&DDRC, &PINC, &PORTC, 4},  // C4 14
  {&DDRC, &PINC, &PORTC, 5},  // C5 15
  {&DDRC, &PINC, &PORTC, 6},  // C6 16
  {&DDRC, &PINC, &PORTC, 7},  // C7 17
  {&DDRE, &PINE, &PORTE, 6},  // E6 18
  {&DDRE, &PINE, &PORTE, 7},  // E7 19
  {&DDRB, &PINB, &PORTB, 0},  // B0 20
  {&DDRB, &PINB, &PORTB, 1},  // B1 21
  {&DDRB, &PINB, &PORTB, 2},  // B2 22
  {&DDRB, &PINB, &PORTB, 3},  // B3 23
  {&DDRB, &PINB, &PORTB, 4},  // B4 24
  {&DDRB, &PINB, &PORTB, 5},  // B5 25
  {&DDRB, &PINB, &PORTB, 6},  // B6 26
  {&DDRB, &PINB, &PORTB, 7},  // B7 27
  {&DDRA, &PINA, &PORTA, 0},  // A0 28
  {&DDRA, &PINA, &PORTA, 1},  // A1 29
  {&DDRA, &PINA, &PORTA, 2},  // A2 30
  {&DDRA, &PINA, &PORTA, 3},  // A3 31
  {&DDRA, &PINA, &PORTA, 4},  // A4 32
  {&DDRA, &PINA, &PORTA, 5},  // A5 33
  {&DDRA, &PINA, &PORTA, 6},  // A6 34
  {&DDRA, &PINA, &PORTA, 7},  // A7 35
  {&DDRE, &PINE, &PORTE, 4},  // E4 36
  {&DDRE, &PINE, &PORTE, 5},  // E5 37
  {&DDRF, &PINF, &PORTF, 0},  // F0 38
  {&DDRF, &PINF, &PORTF, 1},  // F1 39
  {&DDRF, &PINF, &PORTF, 2},  // F2 40
  {&DDRF, &PINF, &PORTF, 3},  // F3 41
  {&DDRF, &PINF, &PORTF, 4},  // F4 42
  {&DDRF, &PINF, &PORTF, 5},  // F5 43
  {&DDRF, &PINF, &PORTF, 6},  // F6 44
  {&DDRF, &PINF, &PORTF, 7}   // F7 45
};
//------------------------------------------------------------------------------
#elif defined(__AVR_ATmega168__)\
||defined(__AVR_ATmega168P__)\
||defined(__AVR_ATmega328P__)
// 168 and 328 Arduinos

// Two Wire (aka I2C) ports
uint8_t const SDA_PIN = 18;  // C4
uint8_t const SCL_PIN = 19;  // C5

// SPI port
uint8_t const SS_PIN = 10;    // B2
uint8_t const MOSI_PIN = 11;  // B3
uint8_t const MISO_PIN = 12;  // B4
uint8_t const SCK_PIN = 13;   // B5

static const pin_map_t digitalPinMap[] = {
  {&DDRD, &PIND, &PORTD, 0},  // D0  0
  {&DDRD, &PIND, &PORTD, 1},  // D1  1
  {&DDRD, &PIND, &PORTD, 2},  // D2  2
  {&DDRD, &PIND, &PORTD, 3},  // D3  3
  {&DDRD, &PIND, &PORTD, 4},  // D4  4
  {&DDRD, &PIND, &PORTD, 5},  // D5  5
  {&DDRD, &PIND, &PORTD, 6},  // D6  6
  {&DDRD, &PIND, &PORTD, 7},  // D7  7
  {&DDRB, &PINB, &PORTB, 0},  // B0  8
  {&DDRB, &PINB, &PORTB, 1},  // B1  9
  {&DDRB, &PINB, &PORTB, 2},  // B2 10
  {&DDRB, &PINB, &PORTB, 3},  // B3 11
  {&DDRB, &PINB, &PORTB, 4},  // B4 12
  {&DDRB, &PINB, &PORTB, 5},  // B5 13
  {&DDRC, &PINC, &PORTC, 0},  // C0 14
  {&DDRC, &PINC, &PORTC, 1},  // C1 15
  {&DDRC, &PINC, &PORTC, 2},  // C2 16
  {&DDRC, &PINC, &PORTC, 3},  // C3 17
  {&DDRC, &PINC, &PORTC, 4},  // C4 18
  {&DDRC, &PINC, &PORTC, 5}   // C5 19
};
#elif defined(__AVR_ATmega1281__)
// Waspmote

// Two Wire (aka I2C) ports
uint8_t const SDA_PIN = 41;
uint8_t const SCL_PIN = 40;


#undef MOSI_PIN
#undef MISO_PIN
// SPI port
uint8_t const SS_PIN = 16;    // B0
uint8_t const MOSI_PIN = 11;  // B2
uint8_t const MISO_PIN = 12;  // B3
uint8_t const SCK_PIN = 10;   // B1

static const pin_map_t digitalPinMap[] = {
  {&DDRE, &PINE, &PORTE, 0}, // E0 0
  {&DDRE, &PINE, &PORTE, 1}, // E1 1
  {&DDRE, &PINE, &PORTE, 3}, // E3 2
  {&DDRE, &PINE, &PORTE, 4}, // E4 3
  {&DDRC, &PINC, &PORTC, 4}, // C4 4
  {&DDRC, &PINC, &PORTC, 5}, // C5 5
  {&DDRC, &PINC, &PORTC, 6}, // C6 6
  {&DDRC, &PINC, &PORTC, 7}, // C7 7
  {&DDRA, &PINA, &PORTA, 2}, // A2 8
  {&DDRA, &PINA, &PORTA, 3}, // A3 9
  {&DDRA, &PINA, &PORTA, 4}, // A4 10
  {&DDRD, &PIND, &PORTD, 5}, // D5 11
  {&DDRD, &PIND, &PORTD, 6}, // D6 12
  {&DDRC, &PINC, &PORTC, 1}, // C1 13
  {&DDRF, &PINF, &PORTF, 1}, // F1 14
  {&DDRF, &PINF, &PORTF, 2}, // F2 15
  {&DDRF, &PINF, &PORTF, 3}, // F3 16
  {&DDRF, &PINF, &PORTF, 4}, // F4 17
  {&DDRF, &PINF, &PORTF, 5}, // F5 18
  {&DDRF, &PINF, &PORTF, 6}, // F6 19
  {&DDRF, &PINF, &PORTF, 7}, // F7 20
  {&DDRF, &PINF, &PORTF, 0}, // F0 21
  {&DDRA, &PINA, &PORTA, 1}, // A1 22
  {&DDRD, &PIND, &PORTD, 7}, // D7 23
  {&DDRE, &PINE, &PORTE, 5}, // E5 24
  {&DDRA, &PINA, &PORTA, 6}, // A6 25
  {&DDRE, &PINE, &PORTE, 2}, // E2 26
  {&DDRA, &PINA, &PORTA, 5}, // A5 27
  {&DDRC, &PINC, &PORTC, 0}, // C0 28
  {&DDRB, &PINB, &PORTB, 0}, // B0 29
  {&DDRB, &PINB, &PORTB, 1}, // B1 30
  {&DDRB, &PINB, &PORTB, 2}, // B2 31
  {&DDRB, &PINB, &PORTB, 3}, // B3 32
  {&DDRB, &PINB, &PORTB, 4}, // B4 33
  {&DDRB, &PINB, &PORTB, 5}, // B5 34
  {&DDRA, &PINA, &PORTA, 0}, // A0 35
  {&DDRB, &PINB, &PORTB, 6}, // B6 36
  {&DDRB, &PINB, &PORTB, 7}, // B7 37
  {&DDRE, &PINE, &PORTE, 6}, // E6 38
  {&DDRE, &PINE, &PORTE, 7}, // E7 39
  {&DDRD, &PIND, &PORTD, 0}, // D0 40
  {&DDRD, &PIND, &PORTD, 1}, // D1 41
  {&DDRC, &PINC, &PORTC, 3}, // C3 42
  {&DDRD, &PIND, &PORTD, 2}, // D2 43
  {&DDRD, &PIND, &PORTD, 3}, // D3 44
  {&DDRA, &PINA, &PORTA, 7}, // A7 45
  {&DDRC, &PINC, &PORTC, 2}, // C2 46
  {&DDRD, &PIND, &PORTD, 4}, // D4 47
  {&DDRG, &PING, &PORTG, 2}, // G2 48
  {&DDRG, &PING, &PORTG, 1}, // G1 49
  {&DDRG, &PING, &PORTG, 0}, // G0 50
};
#else  // defined(__AVR_ATmega1280__)
#error unknown chip
#endif  // defined(__AVR_ATmega1280__)
//------------------------------------------------------------------------------
static const uint8_t digitalPinCount = COUNT(digitalPinMap);

uint8_t badPinNumber(void)
  __attribute__((error("Pin number is too large or not a constant")));

static inline __attribute__((always_inline))
  bool getPinMode(uint8_t pin) {
  if (__builtin_constant_p(pin) && pin < digitalPinCount) {
    return (*digitalPinMap[pin].ddr >> digitalPinMap[pin].bit) & 1;
  }
  else {
    return badPinNumber();
  }
}
static inline __attribute__((always_inline))
  void setPinMode(uint8_t pin, uint8_t mode) {
  if (__builtin_constant_p(pin) && pin < digitalPinCount) {
    if (mode) {
      SBI(*digitalPinMap[pin].ddr, digitalPinMap[pin].bit);
    }
    else {
      CBI(*digitalPinMap[pin].ddr, digitalPinMap[pin].bit);
    }
  }
  else {
    badPinNumber();
  }
}
static inline __attribute__((always_inline))
  bool fastDigitalRead(uint8_t pin) {
  if (__builtin_constant_p(pin) && pin < digitalPinCount) {
    return (*digitalPinMap[pin].pin >> digitalPinMap[pin].bit) & 1;
  }
  else {
    return badPinNumber();
  }
}
static inline __attribute__((always_inline))
  void fastDigitalWrite(uint8_t pin, uint8_t value) {
  if (__builtin_constant_p(pin) && pin < digitalPinCount) {
    if (value) {
      SBI(*digitalPinMap[pin].port, digitalPinMap[pin].bit);
    }
    else {
      CBI(*digitalPinMap[pin].port, digitalPinMap[pin].bit);
    }
  }
  else {
    badPinNumber();
  }
}

//==============================================================================
// SD2CARD - SD card hardware interface class
//==============================================================================

/**
 * \file
 * \brief Sd2Card class for V2 SD/SDHC cards
 */

// SD card commands and structures (from SD Specifications Part 1, Physical Layer, Version 3.01)
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
uint8_t const CMD0 = 0X00;
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
uint8_t const CMD8 = 0X08;
/** SEND_CSD - read the Card Specific Data (CSD register) */
uint8_t const CMD9 = 0X09;
/** SEND_CID - read the card identification information (CID register) */
uint8_t const CMD10 = 0X0A;
/** STOP_TRANSMISSION - end multiple block read sequence */
uint8_t const CMD12 = 0X0C;
/** SEND_STATUS - read the card status register */
uint8_t const CMD13 = 0X0D;
/** READ_SINGLE_BLOCK - read a single data block from the card */
uint8_t const CMD17 = 0X11;
/** READ_MULTIPLE_BLOCK - read a multiple data blocks from the card */
uint8_t const CMD18 = 0X12;
/** WRITE_BLOCK - write a single data block to the card */
uint8_t const CMD24 = 0X18;
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
uint8_t const CMD25 = 0X19;
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
uint8_t const CMD32 = 0X20;
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous range to be erased*/
uint8_t const CMD33 = 0X21;
/** ERASE - erase all previously selected blocks */
uint8_t const CMD38 = 0X26;
/** APP_CMD - escape for application specific command */
uint8_t const CMD55 = 0X37;
/** READ_OCR - read the OCR register of a card */
uint8_t const CMD58 = 0X3A;
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be pre-erased before writing */
uint8_t const ACMD23 = 0X17;
/** SD_SEND_OP_COMD - Sends host capacity support information and activates the card's initialization process */
uint8_t const ACMD41 = 0X29;
//------------------------------------------------------------------------------
/** status for card in the ready state */
uint8_t const R1_READY_STATE = 0X00;
/** status for card in the idle state */
uint8_t const R1_IDLE_STATE = 0X01;
/** status bit for illegal command */
uint8_t const R1_ILLEGAL_COMMAND = 0X04;
/** start data token for read or write single block*/
uint8_t const DATA_START_BLOCK = 0XFE;
/** stop token for write multiple blocks*/
uint8_t const STOP_TRAN_TOKEN = 0XFD;
/** start data token for write multiple blocks*/
uint8_t const WRITE_MULTIPLE_TOKEN = 0XFC;
/** mask for data response tokens after a write block operation */
uint8_t const DATA_RES_MASK = 0X1F;
/** write data accepted token */
uint8_t const DATA_RES_ACCEPTED = 0X05;
//------------------------------------------------------------------------------
/** Card IDentification (CID) register */
typedef struct CID {
  // byte 0
  unsigned char mid;
  // byte 1-2
  char oid[2];
  // byte 3-7
  char pnm[5];
  // byte 8
  unsigned char prv_m : 4;
  unsigned char prv_n : 4;
  // byte 9-12
  uint32_t psn;
  // byte 13
  unsigned char mdt_year_high : 4;
  unsigned char reserved : 4;
  // byte 14
  unsigned char mdt_month : 4;
  unsigned char mdt_year_low : 4;
  // byte 15
  unsigned char always1 : 1;
  unsigned char crc : 7;
} cid_t;
//------------------------------------------------------------------------------
/** CSD for version 1.00 cards */
typedef struct CSDV1 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  unsigned char taac;
  // byte 2
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  unsigned char c_size_high : 2;
  unsigned char reserved2 : 2;
  unsigned char dsr_imp : 1;
  unsigned char read_blk_misalign : 1;
  unsigned char write_blk_misalign : 1;
  unsigned char read_bl_partial : 1;
  // byte 7
  unsigned char c_size_mid;
  // byte 8
  unsigned char vdd_r_curr_max : 3;
  unsigned char vdd_r_curr_min : 3;
  unsigned char c_size_low : 2;
  // byte 9
  unsigned char c_size_mult_high : 2;
  unsigned char vdd_w_cur_max : 3;
  unsigned char vdd_w_curr_min : 3;
  // byte 10
  unsigned char sector_size_high : 6;
  unsigned char erase_blk_en : 1;
  unsigned char c_size_mult_low : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  unsigned char sector_size_low : 1;
  // byte 12
  unsigned char write_bl_len_high : 2;
  unsigned char r2w_factor : 3;
  unsigned char reserved3 : 2;
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved4 : 5;
  unsigned char write_partial : 1;
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved5: 2;
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  unsigned char file_format_grp : 1;
  // byte 15
  unsigned char always1 : 1;
  unsigned char crc : 7;
} csd1_t;
//------------------------------------------------------------------------------
/** CSD for version 2.00 cards */
typedef struct CSDV2 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  unsigned char taac;
  // byte 2
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  unsigned char reserved2 : 4;
  unsigned char dsr_imp : 1;
  unsigned char read_blk_misalign : 1;
  unsigned char write_blk_misalign : 1;
  unsigned char read_bl_partial : 1;
  // byte 7
  unsigned char reserved3 : 2;
  unsigned char c_size_high : 6;
  // byte 8
  unsigned char c_size_mid;
  // byte 9
  unsigned char c_size_low;
  // byte 10
  unsigned char sector_size_high : 6;
  unsigned char erase_blk_en : 1;
  unsigned char reserved4 : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  unsigned char sector_size_low : 1;
  // byte 12
  unsigned char write_bl_len_high : 2;
  unsigned char r2w_factor : 3;
  unsigned char reserved5 : 2;
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved6 : 5;
  unsigned char write_partial : 1;
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved7: 2;
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  unsigned char file_format_grp : 1;
  // byte 15
  unsigned char always1 : 1;
  unsigned char crc : 7;
} csd2_t;
//------------------------------------------------------------------------------
/** union of old and new style CSD register */
union csd_t {
  csd1_t v1;
  csd2_t v2;
};

//------------------------------------------------------------------------------
// SPI speed is F_CPU/2^(1 + index), 0 <= index <= 6
/** Set SCK to max rate of F_CPU/2. See Sd2Card::setSckRate(). */
uint8_t const SPI_FULL_SPEED = 0;
/** Set SCK rate to F_CPU/4. See Sd2Card::setSckRate(). */
uint8_t const SPI_HALF_SPEED = 1;
/** Set SCK rate to F_CPU/8. See Sd2Card::setSckRate(). */
uint8_t const SPI_QUARTER_SPEED = 2;
/** Set SCK rate to F_CPU/16. See Sd2Card::setSckRate(). */
uint8_t const SPI_EIGHTH_SPEED = 3;
/** Set SCK rate to F_CPU/32. See Sd2Card::setSckRate(). */
uint8_t const SPI_SIXTEENTH_SPEED = 4;
//------------------------------------------------------------------------------
/** init timeout ms */
uint16_t const SD_INIT_TIMEOUT = 2000;
/** erase timeout ms */
uint16_t const SD_ERASE_TIMEOUT = 10000;
/** read timeout ms */
uint16_t const SD_READ_TIMEOUT = 300;
/** write time out ms */
uint16_t const SD_WRITE_TIMEOUT = 600;
//------------------------------------------------------------------------------
// SD card errors
/** timeout error for command CMD0 (initialize card in SPI mode) */
uint8_t const SD_CARD_ERROR_CMD0 = 0X1;
/** CMD8 was not accepted - not a valid SD card*/
uint8_t const SD_CARD_ERROR_CMD8 = 0X2;
/** card returned an error response for CMD12 (write stop) */
uint8_t const SD_CARD_ERROR_CMD12 = 0X3;
/** card returned an error response for CMD17 (read block) */
uint8_t const SD_CARD_ERROR_CMD17 = 0X4;
/** card returned an error response for CMD18 (read multiple block) */
uint8_t const SD_CARD_ERROR_CMD18 = 0X5;
/** card returned an error response for CMD24 (write block) */
uint8_t const SD_CARD_ERROR_CMD24 = 0X6;
/**  WRITE_MULTIPLE_BLOCKS command failed */
uint8_t const SD_CARD_ERROR_CMD25 = 0X7;
/** card returned an error response for CMD58 (read OCR) */
uint8_t const SD_CARD_ERROR_CMD58 = 0X8;
/** SET_WR_BLK_ERASE_COUNT failed */
uint8_t const SD_CARD_ERROR_ACMD23 = 0X9;
/** ACMD41 initialization process timeout */
uint8_t const SD_CARD_ERROR_ACMD41 = 0XA;
/** card returned a bad CSR version field */
uint8_t const SD_CARD_ERROR_BAD_CSD = 0XB;
/** erase block group command failed */
uint8_t const SD_CARD_ERROR_ERASE = 0XC;
/** card not capable of single block erase */
uint8_t const SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0XD;
/** Erase sequence timed out */
uint8_t const SD_CARD_ERROR_ERASE_TIMEOUT = 0XE;
/** card returned an error token instead of read data */
uint8_t const SD_CARD_ERROR_READ = 0XF;
/** read CID or CSD failed */
uint8_t const SD_CARD_ERROR_READ_REG = 0X10;
/** timeout while waiting for start of read data */
uint8_t const SD_CARD_ERROR_READ_TIMEOUT = 0X11;
/** card did not accept STOP_TRAN_TOKEN */
uint8_t const SD_CARD_ERROR_STOP_TRAN = 0X12;
/** card returned an error token as a response to a write operation */
uint8_t const SD_CARD_ERROR_WRITE = 0X13;
// SD_CARD_ERROR_WRITE_BLOCK_ZERO (0X14) removed - unused
/** card did not go ready for a multiple block write */
uint8_t const SD_CARD_ERROR_WRITE_MULTIPLE = 0X15;
/** card returned an error to a CMD13 status check after a write */
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X16;
/** timeout occurred during write programming */
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT = 0X17;
/** incorrect rate selected */
uint8_t const SD_CARD_ERROR_SCK_RATE = 0X18;
/** init() not called */
uint8_t const SD_CARD_ERROR_INIT_NOT_CALLED = 0X19;
/** crc check error */
uint8_t const SD_CARD_ERROR_CRC = 0X20;
//------------------------------------------------------------------------------
// card types
/** Standard capacity V1 SD card */
uint8_t const SD_CARD_TYPE_SD1  = 1;
/** Standard capacity V2 SD card */
uint8_t const SD_CARD_TYPE_SD2  = 2;
/** High Capacity SD card */
uint8_t const SD_CARD_TYPE_SDHC = 3;
/**
 * define SOFTWARE_SPI to use bit-bang SPI
 */
//------------------------------------------------------------------------------
#if MEGA_SOFT_SPI && (defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
  #define SOFTWARE_SPI
#elif USE_SOFTWARE_SPI
  #define SOFTWARE_SPI
#endif  // MEGA_SOFT_SPI
//------------------------------------------------------------------------------
// SPI pin definitions - do not edit here - change in SdFatConfig.h
//
#if DISABLED(SOFTWARE_SPI)
  // hardware pin defs
  /** The default chip select pin for the SD card is SS. */
  uint8_t const  SD_CHIP_SELECT_PIN = SS_PIN;
  // The following three pins must not be redefined for hardware SPI.
  /** SPI Master Out Slave In pin */
  uint8_t const  SPI_MOSI_PIN = MOSI_PIN;
  /** SPI Master In Slave Out pin */
  uint8_t const  SPI_MISO_PIN = MISO_PIN;
  /** SPI Clock pin */
  uint8_t const  SPI_SCK_PIN = SCK_PIN;

#else  // SOFTWARE_SPI

  /** SPI chip select pin */
  uint8_t const SD_CHIP_SELECT_PIN = SOFT_SPI_CS_PIN;
  /** SPI Master Out Slave In pin */
  uint8_t const SPI_MOSI_PIN = SOFT_SPI_MOSI_PIN;
  /** SPI Master In Slave Out pin */
  uint8_t const SPI_MISO_PIN = SOFT_SPI_MISO_PIN;
  /** SPI Clock pin */
  uint8_t const SPI_SCK_PIN = SOFT_SPI_SCK_PIN;
#endif  // SOFTWARE_SPI
//------------------------------------------------------------------------------
/**
 * \class Sd2Card
 * \brief Raw access to SD and SDHC flash memory cards.
 */
class Sd2Card {
 public:
  /** Construct an instance of Sd2Card. */
  Sd2Card() : errorCode_(SD_CARD_ERROR_INIT_NOT_CALLED), type_(0) {}
  uint32_t cardSize();
  bool erase(uint32_t firstBlock, uint32_t lastBlock);
  bool eraseSingleBlockEnable();
  /**
   *  Set SD error code.
   *  \param[in] code value for error code.
   */
  void error(uint8_t code) {errorCode_ = code;}
  /**
   * \return error code for last error. See Sd2Card.h for a list of error codes.
   */
  int errorCode() const {return errorCode_;}
  /** \return error data for last error. */
  int errorData() const {return status_;}
  /**
   * Initialize an SD flash memory card with default clock rate and chip
   * select pin.  See sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin).
   *
   * \return true for success or false for failure.
   */
  bool init(uint8_t sckRateID = SPI_FULL_SPEED,
            uint8_t chipSelectPin = SD_CHIP_SELECT_PIN);
  bool readBlock(uint32_t block, uint8_t* dst);
  /**
   * Read a card's CID register. The CID contains card identification
   * information such as Manufacturer ID, Product name, Product serial
   * number and Manufacturing date.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  bool readCID(cid_t* cid) {
    return readRegister(CMD10, cid);
  }
  /**
   * Read a card's CSD register. The CSD contains Card-Specific Data that
   * provides information regarding access to the card's contents.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  bool readCSD(csd_t* csd) {
    return readRegister(CMD9, csd);
  }
  bool readData(uint8_t* dst);
  bool readStart(uint32_t blockNumber);
  bool readStop();
  bool setSckRate(uint8_t sckRateID);
  /** Return the card type: SD V1, SD V2 or SDHC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC.
   */
  int type() const {return type_;}
  bool writeBlock(uint32_t blockNumber, const uint8_t* src);
  bool writeData(const uint8_t* src);
  bool writeStart(uint32_t blockNumber, uint32_t eraseCount);
  bool writeStop();
 private:
  //----------------------------------------------------------------------------
  uint8_t chipSelectPin_;
  uint8_t errorCode_;
  uint8_t spiRate_;
  uint8_t status_;
  uint8_t type_;
  // private functions
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);

  bool readData(uint8_t* dst, uint16_t count);
  bool readRegister(uint8_t cmd, void* buf);
  void chipSelectHigh();
  void chipSelectLow();
  void type(uint8_t value) {type_ = value;}
  bool waitNotBusy(uint16_t timeoutMillis);
  bool writeData(uint8_t token, const uint8_t* src);
};

#endif  // sd_card_hardware_h

#endif
