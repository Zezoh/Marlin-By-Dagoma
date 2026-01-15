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

#ifndef ULTRALCD_H
#define ULTRALCD_H

// LCD support removed - stub file to maintain compilation compatibility
// No display functionality available

// Dummy functions to maintain compatibility
inline void lcd_init() {}
inline void lcd_update() {}
inline void lcd_setstatus(const char* message, const bool persist=false) {}
inline void lcd_setstatuspgm(const char* message, const uint8_t level=0) {}
inline void lcd_setalertstatuspgm(const char* message) {}
inline void lcd_reset_alert_level() {}
inline void lcd_buzz(long duration, uint16_t freq) {}
inline void lcd_quick_feedback() {}
inline bool lcd_detected() { return false; }
inline void lcd_buttons_update() {}
inline bool lcd_hasstatus() { return false; }
inline bool lcd_clicked() { return false; }
inline void lcd_ignore_click(bool b=true) {}
inline void lcd_setcontrast(uint8_t value) {}

#define LCD_MESSAGEPGM(x) do {} while(0)
#define LCD_ALERTMESSAGEPGM(x) do {} while(0)

#endif // ULTRALCD_H
