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
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "Marlin.h"
#include "planner.h"
#if ENABLED(PID_ADD_EXTRUSION_RATE)
  #include "stepper.h"
#endif

// Public interface
void tp_init();
void manage_heater();

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  float analog2widthFil();
  int widthFil_to_size_ratio();
#endif

#if HOTENDS == 1
  #define HOTEND_INDEX 0
#else
  #define HOTEND_INDEX e
#endif

// Temperature state variables
extern int target_temperature[4];
extern float current_temperature[4];
#if ENABLED(SHOW_TEMP_ADC_VALUES)
  extern int current_temperature_raw[4];
  extern int current_temperature_bed_raw;
#endif
extern int target_temperature_bed;
extern float current_temperature_bed;
#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  extern float redundant_temperature;
#endif

#if HAS_CONTROLLERFAN
  extern unsigned char soft_pwm_bed;
#endif

// PID parameters
#if ENABLED(PIDTEMP)
  #if ENABLED(PID_PARAMS_PER_HOTEND)
    extern float Kp[HOTENDS], Ki[HOTENDS], Kd[HOTENDS];
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      extern float Kc[HOTENDS];
    #endif
    #define PID_PARAM(param, e) param[e]
  #else
    extern float Kp, Ki, Kd;
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      extern float Kc;
    #endif
    #define PID_PARAM(param, e) param
  #endif
  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);
#endif

#if ENABLED(PIDTEMPBED)
  extern float bedKp, bedKi, bedKd;
#endif

#if ENABLED(BABYSTEPPING)
  extern volatile int babystepsTodo[3];
#endif

// Inline temperature accessors (degrees Celsius)
FORCE_INLINE float degHotend(uint8_t e) { return current_temperature[HOTEND_INDEX]; }
FORCE_INLINE float degBed() { return current_temperature_bed; }

#if ENABLED(SHOW_TEMP_ADC_VALUES)
  FORCE_INLINE float rawHotendTemp(uint8_t e) { return current_temperature_raw[HOTEND_INDEX]; }
  FORCE_INLINE float rawBedTemp() { return current_temperature_bed_raw; }
#endif

FORCE_INLINE float degTargetHotend(uint8_t e) { return target_temperature[HOTEND_INDEX]; }
FORCE_INLINE float degTargetBed() { return target_temperature_bed; }

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  void start_watching_heater(int e = 0);
#endif

FORCE_INLINE void setTargetHotend(const float& celsius, uint8_t e) {
  target_temperature[HOTEND_INDEX] = celsius;
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    start_watching_heater(HOTEND_INDEX);
  #endif
}
FORCE_INLINE void setTargetBed(const float& celsius) { target_temperature_bed = celsius; }

FORCE_INLINE bool isHeatingHotend(uint8_t e) {
  return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
}
FORCE_INLINE bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }

FORCE_INLINE bool isCoolingHotend(uint8_t e) {
  return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
}
FORCE_INLINE bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }

// Per-hotend convenience routines
#define HOTEND_ROUTINES(NR) \
  FORCE_INLINE float degHotend##NR() { return degHotend(NR); } \
  FORCE_INLINE float degTargetHotend##NR() { return degTargetHotend(NR); } \
  FORCE_INLINE void setTargetHotend##NR(const float c) { setTargetHotend(c, NR); } \
  FORCE_INLINE bool isHeatingHotend##NR() { return isHeatingHotend(NR); } \
  FORCE_INLINE bool isCoolingHotend##NR() { return isCoolingHotend(NR); }
HOTEND_ROUTINES(0);
#if HOTENDS > 1
  HOTEND_ROUTINES(1);
#else
  #define setTargetHotend1(c) do{}while(0)
#endif
#if HOTENDS > 2
  HOTEND_ROUTINES(2);
#else
  #define setTargetHotend2(c) do{}while(0)
#endif
#if HOTENDS > 3
  HOTEND_ROUTINES(3);
#else
  #define setTargetHotend3(c) do{}while(0)
#endif

int getHeaterPower(int heater);
void disable_all_heaters();
void updatePID();

#if ENABLED(PIDTEMP)
  void PID_autotune(float temp, int hotend, int ncycles, bool set_result=false);
#endif

void setExtruderAutoFanState(int pin, bool state);
void checkExtruderAutoFans();

FORCE_INLINE void autotempShutdown() {
  #if ENABLED(AUTOTEMP)
    if (autotemp_enabled) {
      autotemp_enabled = false;
      if (degTargetHotend(active_extruder) > autotemp_min)
        setTargetHotend(0, active_extruder);
    }
  #endif
}

// Z-probe magic (piezo-based bed leveling)
#if ENABLED(Z_MIN_MAGIC)
  extern float z_magic_threshold;
  extern float z_magic_raw_value;
  extern float z_magic_previous;
  extern float z_magic_bias;
  extern float z_magic_bias_delta;
  extern bool z_magic_hit_flag;

  extern bool enable_z_magic_probe;
  extern bool enable_z_magic_tap;
  extern bool log_z_magic_raw_value;
  void reset_z_magic();
#endif

#endif // TEMPERATURE_H
