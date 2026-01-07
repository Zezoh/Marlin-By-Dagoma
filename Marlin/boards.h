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

#ifndef BOARDS_H
#define BOARDS_H

#define BOARD_UNKNOWN -1

#define BOARD_RAMPS_OLD         3    // MEGA/RAMPS up to 1.2
#define BOARD_RAMPS_13_EFB      33   // RAMPS 1.3 (Power outputs: Extruder, Fan, Bed)
#define BOARD_RAMPS_13_EEB      34   // RAMPS 1.3 (Power outputs: Extruder0, Extruder1, Bed)
#define BOARD_RAMPS_13_EFF      35   // RAMPS 1.3 (Power outputs: Extruder, Fan, Fan)
#define BOARD_RAMPS_13_EEF      36   // RAMPS 1.3 (Power outputs: Extruder0, Extruder1, Fan)
#define BOARD_RAMPS_13_SF       38   // RAMPS 1.3 (Power outputs: Spindle, Controller Fan)
#define BOARD_RAMPS_14_EFB      43   // RAMPS 1.4 (Power outputs: Extruder, Fan, Bed)
#define BOARD_RAMPS_14_EEB      44   // RAMPS 1.4 (Power outputs: Extruder0, Extruder1, Bed)
#define BOARD_RAMPS_14_EFF      45   // RAMPS 1.4 (Power outputs: Extruder, Fan, Fan)
#define BOARD_RAMPS_14_EEF      46   // RAMPS 1.4 (Power outputs: Extruder0, Extruder1, Fan)
#define BOARD_RAMPS_14_SF       48   // RAMPS 1.4 (Power outputs: Spindle, Controller Fan)
#define BOARD_MKS_BASE          40   // MKS BASE 1.0
#define BOARD_MKS_13            47   // MKS v1.3 or 1.4 (maybe higher)
#define MB(board) (MOTHERBOARD==BOARD_##board)

#endif //__BOARDS_H
