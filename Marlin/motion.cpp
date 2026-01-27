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
 * motion.cpp - Consolidated motion control implementation
 * 
 * This file consolidates implementations from:
 * 
 * - planner.cpp
 *   Copyright (c) 2009-2011 Simen Svale Skogsrud (Grbl)
 *   Buffers movement commands and manages the acceleration profile plan
 * 
 * - stepper.cpp  
 *   Copyright (c) 2009-2011 Simen Svale Skogsrud (Grbl)
 *   Stepper motor driver: executes motion plans using stepper motors
 * 
 * - stepper_indirection.cpp
 *   Copyright (c) 2015 Dominik Wenger
 *   Stepper motor driver indirection for SPI/I2C drivers
 * 
 * - vector_3.cpp
 *   Copyright (c) 2012 Lars Brubaker
 *   Vector library for bed leveling
 * 
 * - qr_solve.cpp
 *   QR factorization for bed leveling
 */

#include "Marlin.h"
#include "motion.h"
#include "temperature.h"
#include "language.h"
#include "cardreader.h"

#include <math.h>
#include <stdlib.h>

// MESH_BED_LEVELING removed - not supported for Delta

#include "speed_lookuptable.h"

millis_t minsegmenttime;
float max_feedrate[NUM_AXIS]; // Max speeds in mm per minute
float axis_steps_per_unit[NUM_AXIS];
unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
float max_xy_jerk;          // The largest speed change requiring no acceleration
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  // Transform required to compensate for bed level
  matrix_3x3 plan_bed_level_matrix = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
#endif // AUTO_BED_LEVELING_FEATURE

#if ENABLED(AUTOTEMP)
  float autotemp_max = 250;
  float autotemp_min = 210;
  float autotemp_factor = 0.1;
  bool autotemp_enabled = false;
#endif

#if ENABLED(FAN_SOFT_PWM)
  extern unsigned char fanSpeedSoftPwm[FAN_COUNT];
#endif

//===========================================================================
//============ semi-private variables, used in inline functions =============
//===========================================================================

block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//============================ private variables ============================
//===========================================================================

// The current position of the tool in absolute steps
long position[NUM_AXIS];               // Rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[NUM_AXIS]; // Speed of previous path line segment
static float previous_nominal_speed;   // Nominal speed of previous path line segment

uint8_t g_uc_extruder_last_move[EXTRUDERS] = { 0 };

#ifdef XY_FREQUENCY_LIMIT
  // Used for the frequency limit
  #define MAX_FREQ_TIME (1000000.0/XY_FREQUENCY_LIMIT)
  // Old direction bits. Used for speed calculations
  static unsigned char old_direction_bits = 0;
  // Segment times (in µs). Used for speed calculations
  static long axis_segment_time[2][3] = { {MAX_FREQ_TIME + 1, 0, 0}, {MAX_FREQ_TIME + 1, 0, 0} };
#endif

// DUAL_X_CARRIAGE removed - Delta-only firmware

// The minimal step rate ensures calculations stay within limits
// and avoid the most unreasonably slow step rates.
// For AVR this is F_CPU / 500000 (32 for 16MHz, 40 for 20MHz)
#define MINIMAL_STEP_RATE ((F_CPU) / 500000UL)

//===========================================================================
//================================ functions ================================
//===========================================================================

FORCE_INLINE int8_t next_block_index(int8_t block_index) { return block_inc_mod(block_index, 1); }
FORCE_INLINE int8_t prev_block_index(int8_t block_index) { return block_dec_mod(block_index, 1); }

FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration) {
  if (acceleration == 0) return 0;
  return (target_rate * target_rate - initial_rate * initial_rate) / (acceleration * 2);
}

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) {
  if (acceleration == 0) return 0;
  return (acceleration * 2 * distance - initial_rate * initial_rate + final_rate * final_rate) / (acceleration * 4);
}

void calculate_trapezoid_for_block(block_t* block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = lround(block->nominal_rate * entry_factor),
                final_rate = lround(block->nominal_rate * exit_factor);

  // Ensure nominal_rate meets minimum first
  NOLESS(block->nominal_rate, (unsigned long)MINIMAL_STEP_RATE);
  
  // Legacy check against supposed timer overflow. However calc_timer() already
  // should protect against it. But removing this code produces judder in direction-switching
  // moves. This is because the current discrete stepping math diverges from physical motion under
  // constant acceleration when acceleration is large compared to initial/final_rate.
  NOLESS(initial_rate, (unsigned long)MINIMAL_STEP_RATE);  // Enforce the minimum speed
  NOLESS(final_rate, (unsigned long)MINIMAL_STEP_RATE);
  NOMORE(initial_rate, block->nominal_rate);               // Initial/final should not exceed nominal
  NOMORE(final_rate, block->nominal_rate);

  long acceleration = block->acceleration_st;
  // Aims to fully reach nominal and final rates
  int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps = ceil(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Calculate accel / braking time in order to reach the final_rate exactly
  // at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = lround(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    if (accelerate_steps < 0) accelerate_steps = 0;
    if (accelerate_steps > (int32_t)block->step_event_count) accelerate_steps = block->step_event_count;
    decelerate_steps = block->step_event_count - accelerate_steps;
  }

  #if ENABLED(ADVANCE)
    volatile long initial_advance = block->advance * entry_factor * entry_factor;
    volatile long final_advance = block->advance * exit_factor * exit_factor;
  #endif

  CRITICAL_SECTION_START;
  if (!block->busy) {
    block->accelerate_before = accelerate_steps;
    block->decelerate_start = block->step_event_count - decelerate_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
    #if ENABLED(ADVANCE)
      block->initial_advance = initial_advance;
      block->final_advance = final_advance;
    #endif
  }
  CRITICAL_SECTION_END;
}

FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!current || !next) return;
  UNUSED(previous);

  if (current->entry_speed != current->max_entry_speed) {
    if (!current->nominal_length_flag && current->max_entry_speed > next->entry_speed) {
      current->entry_speed = min(current->max_entry_speed,
                                 max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
    }
    else {
      current->entry_speed = current->max_entry_speed;
    }
    current->recalculate_flag = true;
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;

  CRITICAL_SECTION_START;
    unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  if (block_dec_mod(block_buffer_head, tail) > 3) { // moves queued
    block_index = block_dec_mod(block_buffer_head, 3);
    block_t* block[3] = { NULL, NULL, NULL };
    while (block_index != tail) {
      block_index = prev_block_index(block_index);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!previous || current->nominal_length_flag) return;
  UNUSED(next);

  if (previous->entry_speed < current->entry_speed) {
    float entry_speed = min(current->entry_speed,
                            max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
    if (current->entry_speed != entry_speed) {
      current->entry_speed = entry_speed;
      current->recalculate_flag = true;
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t* block[3] = { NULL, NULL, NULL };

  while (block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0], block[1], block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t* current;
  block_t* next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      if (current->recalculate_flag || next->recalculate_flag) {
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        current->recalculate_flag = false;
      }
    }
    block_index = next_block_index(block_index);
  }
  if (next) {
    float nom = next->nominal_speed;
    // Calculate minimum planner speed for this block based on acceleration
    // Only calculate if we have a valid millimeters value to prevent division by zero
    float minimum_planner_speed;
    if (next->millimeters > 0) {
      float steps_per_mm = next->step_event_count / next->millimeters;
      minimum_planner_speed = sqrt(0.5f * next->acceleration / steps_per_mm);
    } else {
      minimum_planner_speed = 0.05f; // Fallback to safe default
    }
    calculate_trapezoid_for_block(next, next->entry_speed / nom, minimum_planner_speed / nom);
    next->recalculate_flag = false;
  }
}

void planner_recalculate() {
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
  previous_nominal_speed = 0.0;
}


#if ENABLED(AUTOTEMP)
  void getHighESpeed() {
    static float oldt = 0;

    if (!autotemp_enabled) return;
    if (degTargetHotend0() + 2 < autotemp_min) return; // probably temperature set to zero.

    float high = 0.0;
    uint8_t block_index = block_buffer_tail;

    while (block_index != block_buffer_head) {
      block_t* block = &block_buffer[block_index];
      if (block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS]) {
        float se = (float)block->steps[E_AXIS] / block->step_event_count * block->nominal_speed; // mm/sec;
        NOLESS(high, se);
      }
      block_index = next_block_index(block_index);
    }

    float t = autotemp_min + high * autotemp_factor;
    t = constrain(t, autotemp_min, autotemp_max);
    if (oldt > t) {
      t *= (1 - (AUTOTEMP_OLDWEIGHT));
      t += (AUTOTEMP_OLDWEIGHT) * oldt;
    }
    oldt = t;
    setTargetHotend0(t);
  }
#endif //AUTOTEMP

void check_axes_activity() {
  unsigned char axis_active[NUM_AXIS] = { 0 },
                tail_fan_speed[FAN_COUNT];

  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) tail_fan_speed[i] = fanSpeeds[i];
  #endif

  #if ENABLED(BARICUDA)
    unsigned char tail_valve_pressure = baricuda_valve_pressure,
                  tail_e_to_p_pressure = baricuda_e_to_p_pressure;
  #endif

  block_t* block;

  if (blocks_queued()) {

    uint8_t block_index = block_buffer_tail;

    #if FAN_COUNT > 0
      for (uint8_t i = 0; i < FAN_COUNT; i++) tail_fan_speed[i] = block_buffer[block_index].fan_speed[i];
    #endif

    #if ENABLED(BARICUDA)
      block = &block_buffer[block_index];
      tail_valve_pressure = block->valve_pressure;
      tail_e_to_p_pressure = block->e_to_p_pressure;
    #endif

    while (block_index != block_buffer_head) {
      block = &block_buffer[block_index];
      for (int i = 0; i < NUM_AXIS; i++) if (block->steps[i]) axis_active[i]++;
      block_index = next_block_index(block_index);
    }
  }
  #if ENABLED(DISABLE_X)
    if (!axis_active[X_AXIS]) disable_x();
  #endif
  #if ENABLED(DISABLE_Y)
    if (!axis_active[Y_AXIS]) disable_y();
  #endif
  #if ENABLED(DISABLE_Z)
    if (!axis_active[Z_AXIS]) disable_z();
  #endif
  #if ENABLED(DISABLE_E)
    if (!axis_active[E_AXIS]) {
      disable_e0();
      disable_e1();
      disable_e2();
      disable_e3();
    }
  #endif

  #if FAN_COUNT > 0

    #if defined(FAN_MIN_PWM)
      #define CALC_FAN_SPEED(f) (tail_fan_speed[f] ? ( FAN_MIN_PWM + (tail_fan_speed[f] * (255 - FAN_MIN_PWM)) / 255 ) : 0)
    #else
      #define CALC_FAN_SPEED(f) tail_fan_speed[f]
    #endif

    #ifdef FAN_KICKSTART_TIME

      static millis_t fan_kick_end[FAN_COUNT] = { 0 };

      #define KICKSTART_FAN(f) \
        if (tail_fan_speed[f]) { \
          millis_t ms = millis(); \
          if (fan_kick_end[f] == 0) { \
            fan_kick_end[f] = ms + FAN_KICKSTART_TIME; \
            tail_fan_speed[f] = 255; \
          } else { \
            if (PENDING(ms, fan_kick_end[f])) { \
              tail_fan_speed[f] = 255; \
            } \
          } \
        } else { \
          fan_kick_end[f] = 0; \
        }

      #if HAS_FAN0
        KICKSTART_FAN(0);
      #endif
      #if HAS_FAN1
        KICKSTART_FAN(1);
      #endif
      #if HAS_FAN2
        KICKSTART_FAN(2);
      #endif

    #endif //FAN_KICKSTART_TIME

    #if ENABLED(FAN_SOFT_PWM)
      #if HAS_FAN0
        fanSpeedSoftPwm[0] = CALC_FAN_SPEED(0);
      #endif
      #if HAS_FAN1
        fanSpeedSoftPwm[1] = CALC_FAN_SPEED(1);
      #endif
      #if HAS_FAN2
        fanSpeedSoftPwm[2] = CALC_FAN_SPEED(2);
      #endif
    #else
      #if HAS_FAN0
        analogWrite(FAN_PIN, CALC_FAN_SPEED(0));
      #endif
      #if HAS_FAN1
        analogWrite(FAN1_PIN, CALC_FAN_SPEED(1));
      #endif
      #if HAS_FAN2
        analogWrite(FAN2_PIN, CALC_FAN_SPEED(2));
      #endif
    #endif

  #endif // FAN_COUNT > 0

  #if ENABLED(AUTOTEMP)
    getHighESpeed();
  #endif

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      analogWrite(HEATER_1_PIN, tail_valve_pressure);
    #endif
    #if HAS_HEATER_2
      analogWrite(HEATER_2_PIN, tail_e_to_p_pressure);
    #endif
  #endif
}


float junction_deviation = 0.1;
// Add a new linear movement to the buffer. steps[X_AXIS], _y and _z is the absolute position in
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  void plan_buffer_line(float x, float y, float z, const float& e, float feed_rate, const uint8_t extruder)
#else
  void plan_buffer_line(const float& x, const float& y, const float& z, const float& e, float feed_rate, const uint8_t extruder)
#endif  // AUTO_BED_LEVELING_FEATURE
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head) idle();

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    apply_rotation_xyz(plan_bed_level_matrix, x, y, z);
  #endif

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[NUM_AXIS];
  target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
  target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);

  long dx = target[X_AXIS] - position[X_AXIS],
       dy = target[Y_AXIS] - position[Y_AXIS],
       dz = target[Z_AXIS] - position[Z_AXIS];

  // DRYRUN ignores all temperature constraints and assures that the extruder is instantly satisfied
  if (DEBUGGING(DRYRUN))
    position[E_AXIS] = target[E_AXIS];

  long de = target[E_AXIS] - position[E_AXIS];

  #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
    if (de) {
      if (degHotend(extruder) < extrude_min_temp) {
        position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
        de = 0; // no difference
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
      }
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de) > axis_steps_per_unit[E_AXIS] * (EXTRUDE_MAXLENGTH)) {
          position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
          de = 0; // no difference
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
        }
      #endif
    }
  #endif

  // Prepare to set up new block
  block_t* block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
  // default non-h-bot planning
  block->steps[X_AXIS] = labs(dx);
  block->steps[Y_AXIS] = labs(dy);
  block->steps[Z_AXIS] = labs(dz);

  block->steps[E_AXIS] = labs(de);
  block->steps[E_AXIS] *= volumetric_multiplier[extruder];
  block->steps[E_AXIS] *= extruder_multiplier[extruder];
  block->steps[E_AXIS] /= 100;
  block->step_event_count = max(block->steps[X_AXIS], max(block->steps[Y_AXIS], max(block->steps[Z_AXIS], block->steps[E_AXIS])));

  // Bail if this is a zero-length block
  if (block->step_event_count <= MIN_STEPS_PER_SEGMENT) return;

  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) block->fan_speed[i] = fanSpeeds[i];
  #endif

  #if ENABLED(BARICUDA)
    block->valve_pressure = baricuda_valve_pressure;
    block->e_to_p_pressure = baricuda_e_to_p_pressure;
  #endif

  // Compute direction bits for this block
  uint8_t db = 0;
  if (dx < 0) SBI(db, X_AXIS);
  if (dy < 0) SBI(db, Y_AXIS);
  if (dz < 0) SBI(db, Z_AXIS);
  if (de < 0) SBI(db, E_AXIS);
  block->direction_bits = db;

  block->active_extruder = extruder;

  //enable active axes
  if (block->steps[X_AXIS]) enable_x();
  if (block->steps[Y_AXIS]) enable_y();
  #if DISABLED(Z_LATE_ENABLE)
    if (block->steps[Z_AXIS]) enable_z();
  #endif

  // Enable extruder(s)
  if (block->steps[E_AXIS]) {
    if (DISABLE_INACTIVE_EXTRUDER) { //enable only selected extruder

      for (int i = 0; i < EXTRUDERS; i++)
        if (g_uc_extruder_last_move[i] > 0) g_uc_extruder_last_move[i]--;

      switch(extruder) {
        case 0:
          enable_e0();
          g_uc_extruder_last_move[0] = (BLOCK_BUFFER_SIZE) * 2;
          #if EXTRUDERS > 1
            if (g_uc_extruder_last_move[1] == 0) disable_e1();
            #if EXTRUDERS > 2
              if (g_uc_extruder_last_move[2] == 0) disable_e2();
              #if EXTRUDERS > 3
                if (g_uc_extruder_last_move[3] == 0) disable_e3();
              #endif
            #endif
          #endif
        break;
        #if EXTRUDERS > 1
          case 1:
            enable_e1();
            g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
            if (g_uc_extruder_last_move[0] == 0) disable_e0();
            #if EXTRUDERS > 2
              if (g_uc_extruder_last_move[2] == 0) disable_e2();
              #if EXTRUDERS > 3
                if (g_uc_extruder_last_move[3] == 0) disable_e3();
              #endif
            #endif
          break;
          #if EXTRUDERS > 2
            case 2:
              enable_e2();
              g_uc_extruder_last_move[2] = (BLOCK_BUFFER_SIZE) * 2;
              if (g_uc_extruder_last_move[0] == 0) disable_e0();
              if (g_uc_extruder_last_move[1] == 0) disable_e1();
              #if EXTRUDERS > 3
                if (g_uc_extruder_last_move[3] == 0) disable_e3();
              #endif
            break;
            #if EXTRUDERS > 3
              case 3:
                enable_e3();
                g_uc_extruder_last_move[3] = (BLOCK_BUFFER_SIZE) * 2;
                if (g_uc_extruder_last_move[0] == 0) disable_e0();
                if (g_uc_extruder_last_move[1] == 0) disable_e1();
                if (g_uc_extruder_last_move[2] == 0) disable_e2();
              break;
            #endif // EXTRUDERS > 3
          #endif // EXTRUDERS > 2
        #endif // EXTRUDERS > 1
      }
    }
    else { // enable all
      enable_e0();
      enable_e1();
      enable_e2();
      enable_e3();
    }
  }

  if (block->steps[E_AXIS])
    NOLESS(feed_rate, minimumfeedrate);
  else
    NOLESS(feed_rate, mintravelfeedrate);

  /**
   * This part of the code calculates the total length of the movement.
   */
  float delta_mm[4];
  delta_mm[X_AXIS] = dx / axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = dy / axis_steps_per_unit[Y_AXIS];
  delta_mm[Z_AXIS] = dz / axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = (de / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[extruder] * extruder_multiplier[extruder] / 100.0;

  if (block->steps[X_AXIS] <= MIN_STEPS_PER_SEGMENT && block->steps[Y_AXIS] <= MIN_STEPS_PER_SEGMENT && block->steps[Z_AXIS] <= MIN_STEPS_PER_SEGMENT) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else {
    block->millimeters = sqrt(
      delta_mm[X_AXIS] * delta_mm[X_AXIS] + delta_mm[Y_AXIS] * delta_mm[Y_AXIS] + delta_mm[Z_AXIS] * delta_mm[Z_AXIS]
    );
  }
  float inverse_millimeters = 1.0 / block->millimeters;
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ENABLED(OLD_SLOWDOWN) || ENABLED(SLOWDOWN)
    bool mq = moves_queued > 1 && moves_queued < (BLOCK_BUFFER_SIZE) / 2;
    #if ENABLED(OLD_SLOWDOWN)
      if (mq) feed_rate *= 2.0 * moves_queued / (BLOCK_BUFFER_SIZE);
    #endif
    #if ENABLED(SLOWDOWN)
      unsigned long segment_time = lround(1000000.0 / inverse_second);
      if (mq && segment_time < minsegmenttime) {
        inverse_second = 1000000.0 / (segment_time + lround(2 * (minsegmenttime - segment_time) / moves_queued));
        #ifdef XY_FREQUENCY_LIMIT
          segment_time = lround(1000000.0 / inverse_second);
        #endif
      }
    #endif
  #endif

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static float filwidth_e_count = 0, filwidth_delay_dist = 0;

    //FMM update ring buffer used for delay with filament measurements
    if (extruder == FILAMENT_SENSOR_EXTRUDER_NUM && filwidth_delay_index2 >= 0) {  //only for extruder with filament sensor and if ring buffer is initialized

      const int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;

      // increment counters with next move in e axis
      filwidth_e_count += delta_mm[E_AXIS];
      filwidth_delay_dist += delta_mm[E_AXIS];

      // Only get new measurements on forward E movement
      if (filwidth_e_count > 0.0001) {

        // Loop the delay distance counter (modulus by the mm length)
        while (filwidth_delay_dist >= MMD_MM) filwidth_delay_dist -= MMD_MM;

        // Convert into an index into the measurement array
        filwidth_delay_index1 = (int)(filwidth_delay_dist / 10.0 + 0.0001);

        // If the index has changed (must have gone forward)...
        if (filwidth_delay_index1 != filwidth_delay_index2) {
          filwidth_e_count = 0; // Reset the E movement counter
          int8_t meas_sample = widthFil_to_size_ratio() - 100; // Subtract 100 to reduce magnitude - to store in a signed char
          do {
            filwidth_delay_index2 = (filwidth_delay_index2 + 1) % MMD_CM; // The next unused slot
            measurement_delay[filwidth_delay_index2] = meas_sample;       // Store the measurement
          } while (filwidth_delay_index1 != filwidth_delay_index2);       // More slots to fill?
        }
      }
    }
  #endif

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for (int i = 0; i < NUM_AXIS; i++) {
    current_speed[i] = delta_mm[i] * inverse_second;
    float cs = fabs(current_speed[i]), mf = max_feedrate[i];
    if (cs > mf) speed_factor = min(speed_factor, mf / cs);
  }

  // Max segement time in us.
  #ifdef XY_FREQUENCY_LIMIT

    // Check and limit the xy direction change frequency
    unsigned char direction_change = block->direction_bits ^ old_direction_bits;
    old_direction_bits = block->direction_bits;
    segment_time = lround((float)segment_time / speed_factor);

    long xs0 = axis_segment_time[X_AXIS][0],
         xs1 = axis_segment_time[X_AXIS][1],
         xs2 = axis_segment_time[X_AXIS][2],
         ys0 = axis_segment_time[Y_AXIS][0],
         ys1 = axis_segment_time[Y_AXIS][1],
         ys2 = axis_segment_time[Y_AXIS][2];

    if (TEST(direction_change, X_AXIS)) {
      xs2 = axis_segment_time[X_AXIS][2] = xs1;
      xs1 = axis_segment_time[X_AXIS][1] = xs0;
      xs0 = 0;
    }
    xs0 = axis_segment_time[X_AXIS][0] = xs0 + segment_time;

    if (TEST(direction_change, Y_AXIS)) {
      ys2 = axis_segment_time[Y_AXIS][2] = axis_segment_time[Y_AXIS][1];
      ys1 = axis_segment_time[Y_AXIS][1] = axis_segment_time[Y_AXIS][0];
      ys0 = 0;
    }
    ys0 = axis_segment_time[Y_AXIS][0] = ys0 + segment_time;

    long max_x_segment_time = max(xs0, max(xs1, xs2)),
         max_y_segment_time = max(ys0, max(ys1, ys2)),
         min_xy_segment_time = min(max_x_segment_time, max_y_segment_time);
    if (min_xy_segment_time < MAX_FREQ_TIME) {
      float low_sf = speed_factor * min_xy_segment_time / (MAX_FREQ_TIME);
      speed_factor = min(speed_factor, low_sf);
    }
  #endif // XY_FREQUENCY_LIMIT

  // Correct the speed
  if (speed_factor < 1.0) {
    for (unsigned char i = 0; i < NUM_AXIS; i++) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;
  unsigned long bsx = block->steps[X_AXIS], bsy = block->steps[Y_AXIS], bsz = block->steps[Z_AXIS], bse = block->steps[E_AXIS];
  if (bsx == 0 && bsy == 0 && bsz == 0) {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else if (bse == 0) {
    block->acceleration_st = ceil(travel_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  // Limit acceleration per axis
  unsigned long acc_st = block->acceleration_st,
                xsteps = axis_steps_per_sqr_second[X_AXIS],
                ysteps = axis_steps_per_sqr_second[Y_AXIS],
                zsteps = axis_steps_per_sqr_second[Z_AXIS],
                esteps = axis_steps_per_sqr_second[E_AXIS],
                allsteps = block->step_event_count;
  if (xsteps < (acc_st * bsx) / allsteps) acc_st = (xsteps * allsteps) / bsx;
  if (ysteps < (acc_st * bsy) / allsteps) acc_st = (ysteps * allsteps) / bsy;
  if (zsteps < (acc_st * bsz) / allsteps) acc_st = (zsteps * allsteps) / bsz;
  if (esteps < (acc_st * bse) / allsteps) acc_st = (esteps * allsteps) / bse;

  block->acceleration_st = acc_st;
  block->acceleration = acc_st / steps_per_mm;
  block->acceleration_rate = (long)(acc_st * 16777216.0 / (F_CPU / 8.0));

  // The minimum possible speed is the average speed for
  // the first / last step at current acceleration limit
  float minimum_planner_speed = sqrt(0.5f * block->acceleration / steps_per_mm);

  // Start with a safe speed
  float vmax_junction = max_xy_jerk / 2;
  float vmax_junction_factor = 1.0;
  float mz2 = max_z_jerk / 2, me2 = max_e_jerk / 2;
  float csz = current_speed[Z_AXIS], cse = current_speed[E_AXIS];
  if (fabs(csz) > mz2) vmax_junction = min(vmax_junction, mz2);
  if (fabs(cse) > me2) vmax_junction = min(vmax_junction, me2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float dsx = current_speed[X_AXIS] - previous_speed[X_AXIS],
          dsy = current_speed[Y_AXIS] - previous_speed[Y_AXIS],
          dsz = fabs(csz - previous_speed[Z_AXIS]),
          dse = fabs(cse - previous_speed[E_AXIS]),
          jerk = sqrt(dsx * dsx + dsy * dsy);

    vmax_junction = block->nominal_speed;
    if (jerk > max_xy_jerk) vmax_junction_factor = max_xy_jerk / jerk;
    if (dsz > max_z_jerk) vmax_junction_factor = min(vmax_junction_factor, max_z_jerk / dsz);
    if (dse > max_e_jerk) vmax_junction_factor = min(vmax_junction_factor, max_e_jerk / dse);

    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor);
  }
  block->max_entry_speed = vmax_junction;

  // Ensure minimum_planner_speed accounts for jerk-based safe speed
  NOLESS(minimum_planner_speed, safe_speed);

  float v_allowable = max_allowable_speed(-block->acceleration, minimum_planner_speed, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);
  block->nominal_length_flag = (block->nominal_speed <= v_allowable);
  block->recalculate_flag = true;

  for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = current_speed[i];
  previous_nominal_speed = block->nominal_speed;

  #if ENABLED(ADVANCE)
    // Calculate advance rate
    if (!bse || (!bsx && !bsy && !bsz)) {
      block->advance_rate = 0;
      block->advance = 0;
    }
    else {
      long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration_st);
      float advance = ((STEPS_PER_CUBIC_MM_E) * (EXTRUDER_ADVANCE_K)) * (cse * cse * (EXTRUSION_AREA) * (EXTRUSION_AREA)) * 256;
      block->advance = advance;
      block->advance_rate = acc_dist ? advance / (float)acc_dist : 0;
    }
  #endif

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);
  block_buffer_head = next_buffer_head;
  for (int i = 0; i < NUM_AXIS; i++) position[i] = target[i];

  planner_recalculate();
  st_wake_up();
}

#if ENABLED(AUTO_BED_LEVELING_FEATURE) && DISABLED(DELTA)

  /**
   * Get the XYZ position of the steppers as a vector_3.
   *
   * On CORE machines XYZ is derived from ABC.
   */
  vector_3 plan_get_position() {
    vector_3 position = vector_3(st_get_axis_position_mm(X_AXIS), st_get_axis_position_mm(Y_AXIS), st_get_axis_position_mm(Z_AXIS));

    //position.debug("in plan_get position");
    //plan_bed_level_matrix.debug("in plan_get_position");
    matrix_3x3 inverse = matrix_3x3::transpose(plan_bed_level_matrix);
    //inverse.debug("in plan_get inverse");
    position.apply_rotation(inverse);
    //position.debug("after rotation");

    return position;
  }

#endif // AUTO_BED_LEVELING_FEATURE && !DELTA

/**
 * Directly set the planner XYZ position (hence the stepper positions).
 *
 * On CORE machines stepper ABC will be translated from the given XYZ.
 */
#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  void plan_set_position(float x, float y, float z, const float& e)
#else
  void plan_set_position(const float& x, const float& y, const float& z, const float& e)
#endif // AUTO_BED_LEVELING_FEATURE
  {
    #if ENABLED(AUTO_BED_LEVELING_FEATURE)
      apply_rotation_xyz(plan_bed_level_matrix, x, y, z);
    #endif

    long nx = position[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]),
         ny = position[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]),
         nz = position[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]),
         ne = position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
    st_set_position(nx, ny, nz, ne);
    previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.

    for (int i = 0; i < NUM_AXIS; i++) previous_speed[i] = 0.0;
  }

void plan_set_e_position(const float& e) {
  position[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
  st_set_e_position(position[E_AXIS]);
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates() {
  for (int i = 0; i < NUM_AXIS; i++)
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
}

//===========================================================================
//============================= Stepper Implementation ======================
//===========================================================================

block_t* current_block;  // A pointer to the block currently being traced

#if ENABLED(HAS_Z_MIN_PROBE)
  volatile bool z_probe_is_active = false;
#endif

//===========================================================================
//============================= private variables ===========================
//===========================================================================

static unsigned char out_bits = 0;
static unsigned int cleaning_buffer_counter;

// Z_DUAL_ENDSTOPS removed - Delta-only firmware

static long counter_x, counter_y, counter_z, counter_e;
volatile static unsigned long step_events_completed;

#if ENABLED(ADVANCE)
  static long advance_rate, advance, final_advance = 0;
  static long old_advance = 0;
  static long e_steps[4];
#endif

static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate;
static uint8_t step_loops;
static uint8_t step_loops_nominal;
static unsigned short OCR1A_nominal;

volatile long endstops_trigsteps[3] = { 0 };
volatile long endstops_stepsTotal, endstops_stepsDone;
static volatile char endstop_hit_bits = 0;

// Z_DUAL_ENDSTOPS removed - simplified endstop bits type
static byte old_endstop_bits = 0;

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  bool abort_on_endstop_hit = false;
#endif

#if ENABLED(EMERGENCY_STOP)
  bool trigger_emergency_stop = false;
#endif

#if HAS_MOTOR_CURRENT_PWM
  #ifndef PWM_MOTOR_CURRENT
    #define PWM_MOTOR_CURRENT DEFAULT_PWM_MOTOR_CURRENT
  #endif
  const int motor_current_setting[3] = PWM_MOTOR_CURRENT;
#endif

static bool check_endstops = true;
static bool check_endstops_global =
  #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
    false
  #else
    true
  #endif
;

volatile long count_position[NUM_AXIS] = { 0 }; // Positions of stepper motors, in step units
volatile signed char count_direction[NUM_AXIS] = { 1 };


//===========================================================================
//================================ functions ================================
//===========================================================================

// Delta-only firmware - simplified stepper macros (DUAL_X_CARRIAGE removed)
#define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
#define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)

// Delta-only firmware - simplified Y stepper macros (Y_DUAL_STEPPER_DRIVERS removed)
#define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
#define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)

// Delta-only firmware - simplified Z stepper macros (Z_DUAL_STEPPER_DRIVERS removed)
#define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
#define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)

#define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %A1, %A2 \n\t" \
                 "add %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r0 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (charIn1), \
                 "d" (intIn2) \
                 : \
                 "r26" \
               )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)

void enable_endstops(bool check) { check_endstops = check; }

void enable_endstops_globally(bool check) { check_endstops_global = check_endstops = check; }

void endstops_not_homing() { check_endstops = check_endstops_global; }

void endstops_hit_on_purpose() { endstop_hit_bits = 0; }

void checkHitEndstops() {
  if (endstop_hit_bits) {
    #define _SET_STOP_CHAR(A,C) ;

    #define _ENDSTOP_HIT_ECHO(A,C) do{ \
      SERIAL_ECHOPAIR(" " STRINGIFY(A) ":", endstops_trigsteps[A ##_AXIS] / axis_steps_per_unit[A ##_AXIS]); \
      _SET_STOP_CHAR(A,C); }while(0)

    #define _ENDSTOP_HIT_TEST(A,C) \
      if (TEST(endstop_hit_bits, A ##_MIN) || TEST(endstop_hit_bits, A ##_MAX)) \
        _ENDSTOP_HIT_ECHO(A,C)

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
    _ENDSTOP_HIT_TEST(X, 'X');
    _ENDSTOP_HIT_TEST(Y, 'Y');
    _ENDSTOP_HIT_TEST(Z, 'Z');

    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      #define P_AXIS Z_AXIS
      if (TEST(endstop_hit_bits, Z_MIN_PROBE)) _ENDSTOP_HIT_ECHO(P, 'P');
    #endif
    SERIAL_EOL;

    endstops_hit_on_purpose();

    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED) && ENABLED(SDSUPPORT)
      if (abort_on_endstop_hit) {
        card.sdprinting = false;
        card.closefile();
        quickStop();
        disable_all_heaters(); // switch off all heaters.
      }
    #endif
  }
}



// Check endstops - Called from ISR!
inline void update_endstops() {

  // Z_DUAL_ENDSTOPS removed - simplified endstop bits type
  byte current_endstop_bits = 0;

  #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
  #define _ENDSTOP_INVERTING(AXIS, MINMAX) AXIS ##_## MINMAX ##_ENDSTOP_INVERTING
  #define _AXIS(AXIS) AXIS ##_AXIS
  #define _ENDSTOP_HIT(AXIS) SBI(endstop_hit_bits, _ENDSTOP(AXIS, MIN))
  #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX

  // SET_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define SET_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_endstop_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != _ENDSTOP_INVERTING(AXIS, MINMAX)))
  // COPY_BIT: copy the value of COPY_BIT to BIT in bits
  #define COPY_BIT(bits, COPY_BIT, BIT) SET_BIT(bits, BIT, TEST(bits, COPY_BIT))
  // TEST_ENDSTOP: test the old and the current status of an endstop
  #define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits, ENDSTOP) && TEST(old_endstop_bits, ENDSTOP))

  #define _SET_TRIGSTEPS(AXIS) endstops_trigsteps[_AXIS(AXIS)] = count_position[_AXIS(AXIS)]

  #define UPDATE_ENDSTOP(AXIS,MINMAX) do { \
      SET_ENDSTOP_BIT(AXIS, MINMAX); \
      if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX)) && current_block->steps[_AXIS(AXIS)] > 0) { \
        _SET_TRIGSTEPS(AXIS); \
        _ENDSTOP_HIT(AXIS); \
        step_events_completed = current_block->step_event_count; \
      } \
    } while(0)

  if (TEST(out_bits, X_AXIS))   // stepping along -X axis
    { // -direction
      // Delta-only firmware - DUAL_X_CARRIAGE code removed
      #if HAS_X_MIN
        UPDATE_ENDSTOP(X, MIN);
      #endif
    }
    else { // +direction
      #if HAS_X_MAX
        UPDATE_ENDSTOP(X, MAX);
      #endif
    }

    if (TEST(out_bits, Y_AXIS))   // -direction
      { // -direction
        #if HAS_Y_MIN
          UPDATE_ENDSTOP(Y, MIN);
        #endif
      }
      else { // +direction
        #if HAS_Y_MAX
          UPDATE_ENDSTOP(Y, MAX);
        #endif
      }

    if (TEST(out_bits, Z_AXIS))
      { // z -direction
        #if HAS_Z_MIN
          // Z_DUAL_ENDSTOPS removed - Delta-only firmware
          #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) && ENABLED(HAS_Z_MIN_PROBE)
            if (z_probe_is_active) UPDATE_ENDSTOP(Z, MIN);
          #else
            UPDATE_ENDSTOP(Z, MIN);
          #endif
        #endif

        #if ENABLED(HAS_Z_MIN_PROBE)
          #if ENABLED(EMERGENCY_STOP)
            #if ENABLED(DELTA) && ENABLED(ONE_BUTTON)
              if ( (READ(ONE_BUTTON_PIN) ^ ONE_BUTTON_INVERTING) ) {
                SET_BIT(current_endstop_bits, Z_MIN_PROBE, 1 );
                trigger_emergency_stop = true;
              }
            #elif ENABLED(SUMMON_PRINT_PAUSE)
              if ( (READ(SUMMON_PRINT_PAUSE_PIN) ^ SUMMON_PRINT_PAUSE_INVERTING) ) {
                SET_BIT(current_endstop_bits, Z_MIN_PROBE, 1 );
                trigger_emergency_stop = true;
              }
            #endif

            if (TEST_ENDSTOP(_ENDSTOP(Z, MIN_PROBE)) && current_block->steps[_AXIS(Z)] > 0) {
              _SET_TRIGSTEPS(Z);
              _ENDSTOP_HIT(Z);
              step_events_completed = current_block->step_event_count;
            }

            if (TEST_ENDSTOP(Z_MIN_PROBE)) SBI(endstop_hit_bits, Z_MIN_PROBE);

            if (trigger_emergency_stop) {
              old_endstop_bits = current_endstop_bits;
              return;
            }
          #endif
        #endif

        #if ENABLED(Z_MIN_PROBE_ENDSTOP) && DISABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) && ENABLED(HAS_Z_MIN_PROBE)
          #if ENABLED( Z_MIN_MAGIC )
              if ( z_probe_is_active ) {
                if (z_magic_hit_flag) {
                  reset_z_magic();
                  SET_BIT(current_endstop_bits, Z_MIN_PROBE, 1 );
                  old_endstop_bits = current_endstop_bits;
                }
                else {
                  SET_BIT(current_endstop_bits, Z_MIN_PROBE, 0 );
                }

                if (TEST_ENDSTOP(_ENDSTOP(Z, MIN_PROBE)) && current_block->steps[_AXIS(Z)] > 0) {
                  _SET_TRIGSTEPS(Z);
                  _ENDSTOP_HIT(Z);
                  step_events_completed = current_block->step_event_count;
                }

                if (TEST_ENDSTOP(Z_MIN_PROBE)) SBI(endstop_hit_bits, Z_MIN_PROBE);

              } // END z_probe_is_active
          #else
            if (z_probe_is_active) {
              UPDATE_ENDSTOP(Z, MIN_PROBE);
              if (TEST_ENDSTOP(Z_MIN_PROBE)) SBI(endstop_hit_bits, Z_MIN_PROBE);
            }
          #endif
        #endif
      }
      else { // z +direction
        #if HAS_Z_MAX
          // Z_DUAL_ENDSTOPS removed - Delta-only firmware
          UPDATE_ENDSTOP(Z, MAX);
        #endif // Z_MAX_PIN
      }
  old_endstop_bits = current_endstop_bits;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The speed over time graph forms a TRAPEZOID. The slope of acceleration is calculated by
//    v = u + t
//  where 't' is the accumulated timer values of the steps so far.
//
//  The Stepper ISR dynamically executes acceleration, deceleration, and cruising according to the block parameters.
//    - Start at block->initial_rate.
//    - Accelerate while step_events_completed < block->accelerate_before.
//    - Cruise while step_events_completed < block->decelerate_start.
//    - Decelerate after that, until all steps are completed.
//    - Reset the trapezoid generator.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  NOMORE(step_rate, MAX_STEP_FREQUENCY);

  if (step_rate > 20000) {
    step_rate = (step_rate >> 2) & 0x3fff;
    step_loops = 4;
  }
  else if (step_rate > 10000) {
    step_rate = (step_rate >> 1) & 0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  NOLESS(step_rate, (unsigned short)MINIMAL_STEP_RATE);
  step_rate -= MINIMAL_STEP_RATE;
  
  unsigned short timer;
  if (step_rate >= (8 * 256)) {
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address + 2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else {
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate) >> 1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
  }
  
  if (timer < 100) {
    timer = 100;
    MYSERIAL.print(MSG_STEPPER_TOO_HIGH);
    MYSERIAL.println(step_rate);
  }
  return timer;
}

/**
 * Set the stepper direction of each axis
 */
void set_stepper_direction() {

  #define SET_STEP_DIR(AXIS) \
    if (TEST(out_bits, AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }

  SET_STEP_DIR(X); // A
  SET_STEP_DIR(Y); // B
  SET_STEP_DIR(Z); // C

  #if DISABLED(ADVANCE)
    if (TEST(out_bits, E_AXIS)) {
      REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif //!ADVANCE
}

FORCE_INLINE void trapezoid_generator_reset() {
  static int8_t last_extruder = -1;

  if (current_block->direction_bits != out_bits || current_block->active_extruder != last_extruder) {
    out_bits = current_block->direction_bits;
    last_extruder = current_block->active_extruder;
    set_stepper_direction();
  }

  #if ENABLED(ADVANCE)
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;
  #endif
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  unsigned short initial_timer = calc_timer(acc_step_rate);
  // Initialize ac/deceleration time as if half the time passed.
  // This smooths the speed change shock between segments.
  acceleration_time = deceleration_time = initial_timer / 2;
  OCR1A = initial_timer;
}

ISR(TIMER1_COMPA_vect) {
  if (cleaning_buffer_counter) {
    current_block = NULL;
    plan_discard_current_block();
    #ifdef SD_FINISHED_RELEASECOMMAND
      if ((cleaning_buffer_counter == 1) && (SD_FINISHED_STEPPERRELEASE)) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    cleaning_buffer_counter--;
    OCR1A = 200;
    return;
  }

  if (!current_block) {
    current_block = plan_get_current_block();
    if (current_block) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_z = counter_e = counter_x;
      step_events_completed = 0;

      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_z();
          OCR1A = 2000;
          return;
        }
      #endif
    }
    else {
      OCR1A = 2000;
    }
  }

  if (current_block != NULL) {
    #if ENABLED(HAS_Z_MIN_PROBE)
      if (check_endstops || z_probe_is_active) update_endstops();
    #else
      if (check_endstops) update_endstops();
    #endif

    for (int8_t i = 0; i < step_loops; i++) {
      #ifndef USBCON
        customizedSerial.checkRx(); // Check for serial chars.
      #endif

      #if ENABLED(ADVANCE)
        counter_e += current_block->steps[E_AXIS];
        if (counter_e > 0) {
          counter_e -= current_block->step_event_count;
          e_steps[current_block->active_extruder] += TEST(out_bits, E_AXIS) ? -1 : 1;
        }
      #endif //ADVANCE

      #define _COUNTER(axis) counter_## axis
      #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
      #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

      #define STEP_ADD(axis, AXIS) \
        _COUNTER(axis) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(axis) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }

      STEP_ADD(x,X);
      STEP_ADD(y,Y);
      STEP_ADD(z,Z);
      #if DISABLED(ADVANCE)
        STEP_ADD(e,E);
      #endif

      #define STEP_IF_COUNTER(axis, AXIS) \
        if (_COUNTER(axis) > 0) { \
          _COUNTER(axis) -= current_block->step_event_count; \
          count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
          _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
        }

      STEP_IF_COUNTER(x, X);
      STEP_IF_COUNTER(y, Y);
      STEP_IF_COUNTER(z, Z);
      #if DISABLED(ADVANCE)
        STEP_IF_COUNTER(e, E);
      #endif

      step_events_completed++;
      if (step_events_completed >= current_block->step_event_count) break;
    }
    unsigned short timer;
    unsigned short step_rate;
    // Are we in acceleration phase?
    if (step_events_completed < (unsigned long)current_block->accelerate_before) {
      MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      NOMORE(acc_step_rate, current_block->nominal_rate);
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;
      deceleration_time = 0; // Reset since we're doing acceleration first.

      #if ENABLED(ADVANCE)
        advance += advance_rate * step_loops;
        e_steps[current_block->active_extruder] += ((advance >> 8) - old_advance);
        old_advance = advance >> 8;
      #endif
    }
    // Are we in deceleration phase?
    else if (step_events_completed >= (unsigned long)current_block->decelerate_start) {
      MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if (step_rate < acc_step_rate) {
        step_rate = acc_step_rate - step_rate;
        NOLESS(step_rate, current_block->final_rate);
      }
      else
        step_rate = current_block->final_rate;

      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;

      #if ENABLED(ADVANCE)
        advance -= advance_rate * step_loops;
        NOLESS(advance, final_advance);

        // Do E steps + advance steps
        uint32_t advance_whole = advance >> 8;
        e_steps[current_block->active_extruder] += advance_whole - old_advance;
        old_advance = advance_whole;
      #endif //ADVANCE
    }
    // We're in cruising phase
    else {
      OCR1A = OCR1A_nominal;
      step_loops = step_loops_nominal;
      // Prepare for deceleration
      acc_step_rate = current_block->nominal_rate;
      deceleration_time = OCR1A_nominal / 2;
    }

    OCR1A = (OCR1A < (TCNT1 + 16)) ? (TCNT1 + 16) : OCR1A;

    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  }
}

#if ENABLED(ADVANCE)
  unsigned char old_OCR0A;
  ISR(TIMER0_COMPA_vect) {
    old_OCR0A += 52;
    OCR0A = old_OCR0A;

    #define STEP_E_ONCE(INDEX) \
      if (e_steps[INDEX] != 0) { \
        E## INDEX ##_STEP_WRITE(INVERT_E_STEP_PIN); \
        if (e_steps[INDEX] < 0) { \
          E## INDEX ##_DIR_WRITE(INVERT_E## INDEX ##_DIR); \
          e_steps[INDEX]++; \
        } \
        else if (e_steps[INDEX] > 0) { \
          E## INDEX ##_DIR_WRITE(!INVERT_E## INDEX ##_DIR); \
          e_steps[INDEX]--; \
        } \
        E## INDEX ##_STEP_WRITE(!INVERT_E_STEP_PIN); \
      }

    for (unsigned char i = 0; i < 4; i++) {
      STEP_E_ONCE(0);
      #if EXTRUDERS > 1
        STEP_E_ONCE(1);
        #if EXTRUDERS > 2
          STEP_E_ONCE(2);
          #if EXTRUDERS > 3
            STEP_E_ONCE(3);
          #endif
        #endif
      #endif
    }
  }
#endif // ADVANCE

void st_init() {
  microstep_init();

  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    // Y_DUAL_STEPPER_DRIVERS removed - Delta-only firmware
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    // Z_DUAL_STEPPER_DRIVERS removed - Delta-only firmware
  #endif
  #if HAS_E0_DIR
    E0_DIR_INIT;
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT;
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT;
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT;
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_X2_ENABLE
    X2_ENABLE_INIT;
    if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
    // Y_DUAL_STEPPER_DRIVERS removed - Delta-only firmware
  #endif
  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    // Z_DUAL_STEPPER_DRIVERS removed - Delta-only firmware
  #endif
  #if HAS_E0_ENABLE
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E1_ENABLE
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E2_ENABLE
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E3_ENABLE
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif

  #if HAS_X_MIN
    SET_INPUT(X_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_XMIN)
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_Y_MIN
    SET_INPUT(Y_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_YMIN)
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_Z_MIN
    SET_INPUT(Z_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_Z2_MIN
    SET_INPUT(Z2_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      WRITE(Z2_MIN_PIN,HIGH);
    #endif
  #endif

  #if HAS_X_MAX
    #if ENABLED(DELTA_EXTRA)
      pinMode(X_MAX_PIN, INPUT_PULLUP);
    #else
      SET_INPUT(X_MAX_PIN);
      #if ENABLED(ENDSTOPPULLUP_XMAX)
        WRITE(X_MAX_PIN,HIGH);
      #endif
    #endif
  #endif

  #if HAS_Y_MAX
    #if ENABLED(DELTA_EXTRA)
      pinMode(Y_MAX_PIN, INPUT_PULLUP);
    #else
      SET_INPUT(Y_MAX_PIN);
      #if ENABLED(ENDSTOPPULLUP_YMAX)
        WRITE(Y_MAX_PIN,HIGH);
      #endif
    #endif
  #endif

  #if HAS_Z_MAX
    #if ENABLED(DELTA_EXTRA)
      pinMode(Z_MAX_PIN, INPUT_PULLUP);
    #else
      SET_INPUT(Z_MAX_PIN);
      #if ENABLED(ENDSTOPPULLUP_ZMAX)
        WRITE(Z_MAX_PIN,HIGH);
      #endif
    #endif
  #endif

  #if HAS_Z2_MAX
    SET_INPUT(Z2_MAX_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZMAX)
      WRITE(Z2_MAX_PIN,HIGH);
    #endif
  #endif

  #if HAS_Z_PROBE && ENABLED(Z_MIN_PROBE_ENDSTOP)
    SET_INPUT(Z_MIN_PROBE_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZMIN_PROBE)
      WRITE(Z_MIN_PROBE_PIN,HIGH);
    #endif
  #endif

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  #define E_AXIS_INIT(NUM) AXIS_INIT(e## NUM, E## NUM, E)

  // Initialize Step Pins
  #if HAS_X_STEP
    AXIS_INIT(x, X, X);
  #endif
  #if HAS_X2_STEP
    AXIS_INIT(x, X2, X);
  #endif
  #if HAS_Y_STEP
    // Y_DUAL_STEPPER_DRIVERS removed - Delta-only firmware
    AXIS_INIT(y, Y, Y);
  #endif
  #if HAS_Z_STEP
    // Z_DUAL_STEPPER_DRIVERS removed - Delta-only firmware
    AXIS_INIT(z, Z, Z);
  #endif
  #if HAS_E0_STEP
    E_AXIS_INIT(0);
  #endif
  #if HAS_E1_STEP
    E_AXIS_INIT(1);
  #endif
  #if HAS_E2_STEP
    E_AXIS_INIT(2);
  #endif
  #if HAS_E3_STEP
    E_AXIS_INIT(3);
  #endif

  CBI(TCCR1B, WGM13);
  SBI(TCCR1B, WGM12);
  CBI(TCCR1A, WGM11);
  CBI(TCCR1A, WGM10);

  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #if ENABLED(ADVANCE)
    #if defined(TCCR0A) && defined(WGM01)
      CBI(TCCR0A, WGM01);
      CBI(TCCR0A, WGM00);
    #endif
    e_steps[0] = e_steps[1] = e_steps[2] = e_steps[3] = 0;
    SBI(TIMSK0, OCIE0A);
  #endif

  enable_endstops(true);
  sei();
  set_stepper_direction();
}


/**
 * Block until all buffered steps are executed
 */
void st_synchronize() { while (blocks_queued()) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 *
 * This allows st_get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void st_set_position(const long& x, const long& y, const long& z, const long& e) {
  CRITICAL_SECTION_START;

  // default non-h-bot planning
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;

  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long& e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
long st_get_position(AxisEnum axis) {
  CRITICAL_SECTION_START;
  long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/**
 * Get an axis position according to stepper position
 */
float st_get_axis_position_mm(AxisEnum axis) {
  float axis_steps;
  axis_steps = st_get_position(axis);
  return axis_steps / axis_steps_per_unit[axis];
}

void finishAndDisableSteppers() {
  st_synchronize();
  disable_all_steppers();
}

void quickStop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (blocks_queued()) plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

#if ENABLED(BABYSTEPPING)

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void babystep(const uint8_t axis, const bool direction) {

    #define _ENABLE(axis) enable_## axis()
    #define _READ_DIR(AXIS) AXIS ##_DIR_READ
    #define _INVERT_DIR(AXIS) INVERT_## AXIS ##_DIR
    #define _APPLY_DIR(AXIS, INVERT) AXIS ##_APPLY_DIR(INVERT, true)

    #define BABYSTEP_AXIS(axis, AXIS, INVERT) { \
        _ENABLE(axis); \
        uint8_t old_pin = _READ_DIR(AXIS); \
        _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^direction^INVERT); \
        _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true); \
        delayMicroseconds(2); \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true); \
        _APPLY_DIR(AXIS, old_pin); \
      }

    switch (axis) {

      case X_AXIS:
        BABYSTEP_AXIS(x, X, false);
        break;

      case Y_AXIS:
        BABYSTEP_AXIS(y, Y, false);
        break;

      case Z_AXIS: {

        #if DISABLED(DELTA)

          BABYSTEP_AXIS(z, Z, BABYSTEP_INVERT_Z);

        #else // DELTA

          bool z_direction = direction ^ BABYSTEP_INVERT_Z;

          enable_x();
          enable_y();
          enable_z();
          uint8_t old_x_dir_pin = X_DIR_READ,
                  old_y_dir_pin = Y_DIR_READ,
                  old_z_dir_pin = Z_DIR_READ;
          X_DIR_WRITE(INVERT_X_DIR ^ z_direction);
          Y_DIR_WRITE(INVERT_Y_DIR ^ z_direction);
          Z_DIR_WRITE(INVERT_Z_DIR ^ z_direction);
          X_STEP_WRITE(!INVERT_X_STEP_PIN);
          Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          delayMicroseconds(2);
          X_STEP_WRITE(INVERT_X_STEP_PIN);
          Y_STEP_WRITE(INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);
          X_DIR_WRITE(old_x_dir_pin);
          Y_DIR_WRITE(old_y_dir_pin);
          Z_DIR_WRITE(old_z_dir_pin);
        #endif

      } break;

      default: break;
    }
  }

#endif //BABYSTEPPING

void microstep_init() {
  #if HAS_MICROSTEPS_E1
    pinMode(E1_MS1_PIN, OUTPUT);
    pinMode(E1_MS2_PIN, OUTPUT);
  #endif

  #if HAS_MICROSTEPS
    pinMode(X_MS1_PIN, OUTPUT);
    pinMode(X_MS2_PIN, OUTPUT);
    pinMode(Y_MS1_PIN, OUTPUT);
    pinMode(Y_MS2_PIN, OUTPUT);
    pinMode(Z_MS1_PIN, OUTPUT);
    pinMode(Z_MS2_PIN, OUTPUT);
    pinMode(E0_MS1_PIN, OUTPUT);
    pinMode(E0_MS2_PIN, OUTPUT);
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  #endif
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2) {
  if (ms1 >= 0) switch (driver) {
    case 0: digitalWrite(X_MS1_PIN, ms1); break;
    case 1: digitalWrite(Y_MS1_PIN, ms1); break;
    case 2: digitalWrite(Z_MS1_PIN, ms1); break;
    case 3: digitalWrite(E0_MS1_PIN, ms1); break;
    #if HAS_MICROSTEPS_E1
      case 4: digitalWrite(E1_MS1_PIN, ms1); break;
    #endif
  }
  if (ms2 >= 0) switch (driver) {
    case 0: digitalWrite(X_MS2_PIN, ms2); break;
    case 1: digitalWrite(Y_MS2_PIN, ms2); break;
    case 2: digitalWrite(Z_MS2_PIN, ms2); break;
    case 3: digitalWrite(E0_MS2_PIN, ms2); break;
    #if PIN_EXISTS(E1_MS2)
      case 4: digitalWrite(E1_MS2_PIN, ms2); break;
    #endif
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode) {
  switch (stepping_mode) {
    case 1: microstep_ms(driver, MICROSTEP1); break;
    case 2: microstep_ms(driver, MICROSTEP2); break;
    case 4: microstep_ms(driver, MICROSTEP4); break;
    case 8: microstep_ms(driver, MICROSTEP8); break;
    case 16: microstep_ms(driver, MICROSTEP16); break;
  }
}

void microstep_readings() {
  SERIAL_PROTOCOLPGM("MS1,MS2 Pins\n");
  SERIAL_PROTOCOLPGM("X: ");
  SERIAL_PROTOCOL(digitalRead(X_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(X_MS2_PIN));
  SERIAL_PROTOCOLPGM("Y: ");
  SERIAL_PROTOCOL(digitalRead(Y_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(Y_MS2_PIN));
  SERIAL_PROTOCOLPGM("Z: ");
  SERIAL_PROTOCOL(digitalRead(Z_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(Z_MS2_PIN));
  SERIAL_PROTOCOLPGM("E0: ");
  SERIAL_PROTOCOL(digitalRead(E0_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(E0_MS2_PIN));
  #if HAS_MICROSTEPS_E1
    SERIAL_PROTOCOLPGM("E1: ");
    SERIAL_PROTOCOL(digitalRead(E1_MS1_PIN));
    SERIAL_PROTOCOLLN(digitalRead(E1_MS2_PIN));
  #endif
}

// Z_DUAL_ENDSTOPS removed - Delta-only firmware

//===========================================================================
//===================== Vector 3 Implementation =============================
//===========================================================================

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

vector_3::vector_3() : x(0), y(0), z(0) { }

vector_3::vector_3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) { }

vector_3 vector_3::cross(vector_3 left, vector_3 right) {
  return vector_3(left.y * right.z - left.z * right.y,
                  left.z * right.x - left.x * right.z,
                  left.x * right.y - left.y * right.x);
}

vector_3 vector_3::operator+(vector_3 v) { return vector_3((x + v.x), (y + v.y), (z + v.z)); }
vector_3 vector_3::operator-(vector_3 v) { return vector_3((x - v.x), (y - v.y), (z - v.z)); }

vector_3 vector_3::get_normal() {
  vector_3 normalized = vector_3(x, y, z);
  normalized.normalize();
  return normalized;
}

float vector_3::get_length() {
  return sqrt(x * x + y * y + z * z);
}

void vector_3::normalize() {
  float len = get_length();
  if (len > 0) {
    float inv_len = 1.0 / len;
    x *= inv_len;
    y *= inv_len;
    z *= inv_len;
  }
}

void vector_3::apply_rotation(matrix_3x3 matrix) {
  float resultX = x * matrix.matrix[3 * 0 + 0] + y * matrix.matrix[3 * 1 + 0] + z * matrix.matrix[3 * 2 + 0];
  float resultY = x * matrix.matrix[3 * 0 + 1] + y * matrix.matrix[3 * 1 + 1] + z * matrix.matrix[3 * 2 + 1];
  float resultZ = x * matrix.matrix[3 * 0 + 2] + y * matrix.matrix[3 * 1 + 2] + z * matrix.matrix[3 * 2 + 2];
  x = resultX;
  y = resultY;
  z = resultZ;
}

void vector_3::debug(const char title[]) {
  SERIAL_PROTOCOL(title);
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL_F(x, 6);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL_F(y, 6);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL_F(z, 6);
  SERIAL_EOL;
}

void apply_rotation_xyz(matrix_3x3 matrix, float& x, float& y, float& z) {
  vector_3 vector = vector_3(x, y, z);
  vector.apply_rotation(matrix);
  x = vector.x;
  y = vector.y;
  z = vector.z;
}

matrix_3x3 matrix_3x3::create_from_rows(vector_3 row_0, vector_3 row_1, vector_3 row_2) {
  matrix_3x3 new_matrix;
  new_matrix.matrix[0] = row_0.x; new_matrix.matrix[1] = row_0.y; new_matrix.matrix[2] = row_0.z;
  new_matrix.matrix[3] = row_1.x; new_matrix.matrix[4] = row_1.y; new_matrix.matrix[5] = row_1.z;
  new_matrix.matrix[6] = row_2.x; new_matrix.matrix[7] = row_2.y; new_matrix.matrix[8] = row_2.z;
  return new_matrix;
}

void matrix_3x3::set_to_identity() {
  matrix[0] = 1; matrix[1] = 0; matrix[2] = 0;
  matrix[3] = 0; matrix[4] = 1; matrix[5] = 0;
  matrix[6] = 0; matrix[7] = 0; matrix[8] = 1;
}

matrix_3x3 matrix_3x3::create_look_at(vector_3 target) {
  vector_3 z_row = target.get_normal();
  vector_3 x_row = vector_3(1, 0, -target.x / target.z).get_normal();
  vector_3 y_row = vector_3::cross(z_row, x_row).get_normal();
  matrix_3x3 rot = matrix_3x3::create_from_rows(x_row, y_row, z_row);
  return rot;
}

matrix_3x3 matrix_3x3::transpose(matrix_3x3 original) {
  matrix_3x3 new_matrix;
  new_matrix.matrix[0] = original.matrix[0]; new_matrix.matrix[1] = original.matrix[3]; new_matrix.matrix[2] = original.matrix[6];
  new_matrix.matrix[3] = original.matrix[1]; new_matrix.matrix[4] = original.matrix[4]; new_matrix.matrix[5] = original.matrix[7];
  new_matrix.matrix[6] = original.matrix[2]; new_matrix.matrix[7] = original.matrix[5]; new_matrix.matrix[8] = original.matrix[8];
  return new_matrix;
}

void matrix_3x3::debug(const char title[]) {
  SERIAL_PROTOCOLLN(title);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (matrix[count] >= 0.0) SERIAL_PROTOCOLCHAR('+');
      SERIAL_PROTOCOL_F(matrix[count], 6);
      SERIAL_PROTOCOLCHAR(' ');
      count++;
    }
    SERIAL_EOL;
  }
}

#endif // AUTO_BED_LEVELING_FEATURE


//===========================================================================
//==================== QR Solve Implementation ==============================
//===========================================================================

#if ENABLED(AUTO_BED_LEVELING_GRID)

int i4_min(int i1, int i2)

/******************************************************************************/
/**
  Purpose:

    I4_MIN returns the smaller of two I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    29 August 2006

  Author:

    John Burkardt

  Parameters:

    Input, int I1, I2, two integers to be compared.

    Output, int I4_MIN, the smaller of I1 and I2.
*/
{
  return (i1 < i2) ? i1 : i2;
}

double r8_epsilon(void)

/******************************************************************************/
/**
  Purpose:

    R8_EPSILON returns the R8 round off unit.

  Discussion:

    R8_EPSILON is a number R which is a power of 2 with the property that,
    to the precision of the computer's arithmetic,
      1 < 1 + R
    but
      1 = ( 1 + R / 2 )

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    01 September 2012

  Author:

    John Burkardt

  Parameters:

    Output, double R8_EPSILON, the R8 round-off unit.
*/
{
  const double value = 2.220446049250313E-016;
  return value;
}

double r8_max(double x, double y)

/******************************************************************************/
/**
  Purpose:

    R8_MAX returns the maximum of two R8's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 May 2006

  Author:

    John Burkardt

  Parameters:

    Input, double X, Y, the quantities to compare.

    Output, double R8_MAX, the maximum of X and Y.
*/
{
  return (y < x) ? x : y;
}

double r8_abs(double x)

/******************************************************************************/
/**
  Purpose:

    R8_ABS returns the absolute value of an R8.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 May 2006

  Author:

    John Burkardt

  Parameters:

    Input, double X, the quantity whose absolute value is desired.

    Output, double R8_ABS, the absolute value of X.
*/
{
  return (x < 0.0) ? -x : x;
}

double r8_sign(double x)

/******************************************************************************/
/**
  Purpose:

    R8_SIGN returns the sign of an R8.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    08 May 2006

  Author:

    John Burkardt

  Parameters:

    Input, double X, the number whose sign is desired.

    Output, double R8_SIGN, the sign of X.
*/
{
  return (x < 0.0) ? -1.0 : 1.0;
}

double r8mat_amax(int m, int n, double a[])

/******************************************************************************/
/**
  Purpose:

    R8MAT_AMAX returns the maximum absolute value entry of an R8MAT.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 September 2012

  Author:

    John Burkardt

  Parameters:

    Input, int M, the number of rows in A.

    Input, int N, the number of columns in A.

    Input, double A[M*N], the M by N matrix.

    Output, double R8MAT_AMAX, the maximum absolute value entry of A.
*/
{
  double value = r8_abs(a[0 + 0 * m]);
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++) {
      NOLESS(value, r8_abs(a[i + j * m]));
    }
  }
  return value;
}

void r8mat_copy(double a2[], int m, int n, double a1[])

/******************************************************************************/
/**
  Purpose:

    R8MAT_COPY_NEW copies one R8MAT to a "new" R8MAT.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    26 July 2008

  Author:

    John Burkardt

  Parameters:

    Input, int M, N, the number of rows and columns.

    Input, double A1[M*N], the matrix to be copied.

    Output, double R8MAT_COPY_NEW[M*N], the copy of A1.
*/
{
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++)
      a2[i + j * m] = a1[i + j * m];
  }
}

/******************************************************************************/

void daxpy(int n, double da, double dx[], int incx, double dy[], int incy)

/******************************************************************************/
/**
  Purpose:

    DAXPY computes constant times a vector plus a vector.

  Discussion:

    This routine uses unrolled loops for increments equal to one.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 March 2007

  Author:

    C version by John Burkardt

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch, Pete Stewart,
    LINPACK User's Guide,
    SIAM, 1979.

    Charles Lawson, Richard Hanson, David Kincaid, Fred Krogh,
    Basic Linear Algebra Subprograms for Fortran Usage,
    Algorithm 539,
    ACM Transactions on Mathematical Software,
    Volume 5, Number 3, September 1979, pages 308-323.

  Parameters:

    Input, int N, the number of elements in DX and DY.

    Input, double DA, the multiplier of DX.

    Input, double DX[*], the first vector.

    Input, int INCX, the increment between successive entries of DX.

    Input/output, double DY[*], the second vector.
    On output, DY[*] has been replaced by DY[*] + DA * DX[*].

    Input, int INCY, the increment between successive entries of DY.
*/
{
  if (n <= 0 || da == 0.0) return;

  int i, ix, iy, m;
  /**
    Code for unequal increments or equal increments
    not equal to 1.
  */
  if (incx != 1 || incy != 1) {
    if (0 <= incx)
      ix = 0;
    else
      ix = (- n + 1) * incx;
    if (0 <= incy)
      iy = 0;
    else
      iy = (- n + 1) * incy;
    for (i = 0; i < n; i++) {
      dy[iy] = dy[iy] + da * dx[ix];
      ix = ix + incx;
      iy = iy + incy;
    }
  }
  /**
    Code for both increments equal to 1.
  */
  else {
    m = n % 4;
    for (i = 0; i < m; i++)
      dy[i] = dy[i] + da * dx[i];
    for (i = m; i < n; i = i + 4) {
      dy[i  ] = dy[i  ] + da * dx[i  ];
      dy[i + 1] = dy[i + 1] + da * dx[i + 1];
      dy[i + 2] = dy[i + 2] + da * dx[i + 2];
      dy[i + 3] = dy[i + 3] + da * dx[i + 3];
    }
  }
}
/******************************************************************************/

double ddot(int n, double dx[], int incx, double dy[], int incy)

/******************************************************************************/
/**
  Purpose:

    DDOT forms the dot product of two vectors.

  Discussion:

    This routine uses unrolled loops for increments equal to one.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 March 2007

  Author:

    C version by John Burkardt

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch, Pete Stewart,
    LINPACK User's Guide,
    SIAM, 1979.

    Charles Lawson, Richard Hanson, David Kincaid, Fred Krogh,
    Basic Linear Algebra Subprograms for Fortran Usage,
    Algorithm 539,
    ACM Transactions on Mathematical Software,
    Volume 5, Number 3, September 1979, pages 308-323.

  Parameters:

    Input, int N, the number of entries in the vectors.

    Input, double DX[*], the first vector.

    Input, int INCX, the increment between successive entries in DX.

    Input, double DY[*], the second vector.

    Input, int INCY, the increment between successive entries in DY.

    Output, double DDOT, the sum of the product of the corresponding
    entries of DX and DY.
*/
{

  if (n <= 0) return 0.0;

  int i, m;
  double dtemp = 0.0;

  /**
    Code for unequal increments or equal increments
    not equal to 1.
  */
  if (incx != 1 || incy != 1) {
    int ix = (incx >= 0) ? 0 : (-n + 1) * incx,
        iy = (incy >= 0) ? 0 : (-n + 1) * incy;
    for (i = 0; i < n; i++) {
      dtemp += dx[ix] * dy[iy];
      ix = ix + incx;
      iy = iy + incy;
    }
  }
  /**
    Code for both increments equal to 1.
  */
  else {
    m = n % 5;
    for (i = 0; i < m; i++)
      dtemp += dx[i] * dy[i];
    for (i = m; i < n; i = i + 5) {
      dtemp += dx[i] * dy[i]
              + dx[i + 1] * dy[i + 1]
              + dx[i + 2] * dy[i + 2]
              + dx[i + 3] * dy[i + 3]
              + dx[i + 4] * dy[i + 4];
    }
  }
  return dtemp;
}
/******************************************************************************/

double dnrm2(int n, double x[], int incx)

/******************************************************************************/
/**
  Purpose:

    DNRM2 returns the euclidean norm of a vector.

  Discussion:

     DNRM2 ( X ) = sqrt ( X' * X )

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 March 2007

  Author:

    C version by John Burkardt

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch, Pete Stewart,
    LINPACK User's Guide,
    SIAM, 1979.

    Charles Lawson, Richard Hanson, David Kincaid, Fred Krogh,
    Basic Linear Algebra Subprograms for Fortran Usage,
    Algorithm 539,
    ACM Transactions on Mathematical Software,
    Volume 5, Number 3, September 1979, pages 308-323.

  Parameters:

    Input, int N, the number of entries in the vector.

    Input, double X[*], the vector whose norm is to be computed.

    Input, int INCX, the increment between successive entries of X.

    Output, double DNRM2, the Euclidean norm of X.
*/
{
  double norm;
  if (n < 1 || incx < 1)
    norm = 0.0;
  else if (n == 1)
    norm = r8_abs(x[0]);
  else {
    double scale = 0.0, ssq = 1.0;
    int ix = 0;
    for (int i = 0; i < n; i++) {
      if (x[ix] != 0.0) {
        double absxi = r8_abs(x[ix]);
        if (scale < absxi) {
          ssq = 1.0 + ssq * (scale / absxi) * (scale / absxi);
          scale = absxi;
        }
        else
          ssq = ssq + (absxi / scale) * (absxi / scale);
      }
      ix += incx;
    }
    norm = scale * sqrt(ssq);
  }
  return norm;
}
/******************************************************************************/

void dqrank(double a[], int lda, int m, int n, double tol, int* kr,
            int jpvt[], double qraux[])

/******************************************************************************/
/**
  Purpose:

    DQRANK computes the QR factorization of a rectangular matrix.

  Discussion:

    This routine is used in conjunction with DQRLSS to solve
    overdetermined, underdetermined and singular linear systems
    in a least squares sense.

    DQRANK uses the LINPACK subroutine DQRDC to compute the QR
    factorization, with column pivoting, of an M by N matrix A.
    The numerical rank is determined using the tolerance TOL.

    Note that on output, ABS ( A(1,1) ) / ABS ( A(KR,KR) ) is an estimate
    of the condition number of the matrix of independent columns,
    and of R.  This estimate will be <= 1/TOL.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    21 April 2012

  Author:

    C version by John Burkardt.

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch, Pete Stewart,
    LINPACK User's Guide,
    SIAM, 1979,
    ISBN13: 978-0-898711-72-1,
    LC: QA214.L56.

  Parameters:

    Input/output, double A[LDA*N].  On input, the matrix whose
    decomposition is to be computed.  On output, the information from DQRDC.
    The triangular matrix R of the QR factorization is contained in the
    upper triangle and information needed to recover the orthogonal
    matrix Q is stored below the diagonal in A and in the vector QRAUX.

    Input, int LDA, the leading dimension of A, which must
    be at least M.

    Input, int M, the number of rows of A.

    Input, int N, the number of columns of A.

    Input, double TOL, a relative tolerance used to determine the
    numerical rank.  The problem should be scaled so that all the elements
    of A have roughly the same absolute accuracy, EPS.  Then a reasonable
    value for TOL is roughly EPS divided by the magnitude of the largest
    element.

    Output, int *KR, the numerical rank.

    Output, int JPVT[N], the pivot information from DQRDC.
    Columns JPVT(1), ..., JPVT(KR) of the original matrix are linearly
    independent to within the tolerance TOL and the remaining columns
    are linearly dependent.

    Output, double QRAUX[N], will contain extra information defining
    the QR factorization.
*/
{
  double work[n];

  for (int i = 0; i < n; i++)
    jpvt[i] = 0;

  int job = 1;

  dqrdc(a, lda, m, n, qraux, jpvt, work, job);

  *kr = 0;
  int k = i4_min(m, n);
  for (int j = 0; j < k; j++) {
    if (r8_abs(a[j + j * lda]) <= tol * r8_abs(a[0 + 0 * lda]))
      return;
    *kr = j + 1;
  }
}
/******************************************************************************/

void dqrdc(double a[], int lda, int n, int p, double qraux[], int jpvt[],
           double work[], int job)

/******************************************************************************/
/**
  Purpose:

    DQRDC computes the QR factorization of a real rectangular matrix.

  Discussion:

    DQRDC uses Householder transformations.

    Column pivoting based on the 2-norms of the reduced columns may be
    performed at the user's option.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 June 2005

  Author:

    C version by John Burkardt.

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch and Pete Stewart,
    LINPACK User's Guide,
    SIAM, (Society for Industrial and Applied Mathematics),
    3600 University City Science Center,
    Philadelphia, PA, 19104-2688.
    ISBN 0-89871-172-X

  Parameters:

    Input/output, double A(LDA,P).  On input, the N by P matrix
    whose decomposition is to be computed.  On output, A contains in
    its upper triangle the upper triangular matrix R of the QR
    factorization.  Below its diagonal A contains information from
    which the orthogonal part of the decomposition can be recovered.
    Note that if pivoting has been requested, the decomposition is not that
    of the original matrix A but that of A with its columns permuted
    as described by JPVT.

    Input, int LDA, the leading dimension of the array A.  LDA must
    be at least N.

    Input, int N, the number of rows of the matrix A.

    Input, int P, the number of columns of the matrix A.

    Output, double QRAUX[P], contains further information required
    to recover the orthogonal part of the decomposition.

    Input/output, integer JPVT[P].  On input, JPVT contains integers that
    control the selection of the pivot columns.  The K-th column A(*,K) of A
    is placed in one of three classes according to the value of JPVT(K).
      > 0, then A(K) is an initial column.
      = 0, then A(K) is a free column.
      < 0, then A(K) is a final column.
    Before the decomposition is computed, initial columns are moved to
    the beginning of the array A and final columns to the end.  Both
    initial and final columns are frozen in place during the computation
    and only free columns are moved.  At the K-th stage of the
    reduction, if A(*,K) is occupied by a free column it is interchanged
    with the free column of largest reduced norm.  JPVT is not referenced
    if JOB == 0.  On output, JPVT(K) contains the index of the column of the
    original matrix that has been interchanged into the K-th column, if
    pivoting was requested.

    Workspace, double WORK[P].  WORK is not referenced if JOB == 0.

    Input, int JOB, initiates column pivoting.
    0, no pivoting is done.
    nonzero, pivoting is done.
*/
{
  int jp;
  int j;
  int lup;
  int maxj;
  double maxnrm, nrmxl, t, tt;

  int pl = 1, pu = 0;
  /**
    If pivoting is requested, rearrange the columns.
  */
  if (job != 0) {
    for (j = 1; j <= p; j++) {
      int swapj = (0 < jpvt[j - 1]);
      jpvt[j - 1] = (jpvt[j - 1] < 0) ? -j : j;
      if (swapj) {
        if (j != pl)
          dswap(n, a + 0 + (pl - 1)*lda, 1, a + 0 + (j - 1), 1);
        jpvt[j - 1] = jpvt[pl - 1];
        jpvt[pl - 1] = j;
        pl++;
      }
    }
    pu = p;
    for (j = p; 1 <= j; j--) {
      if (jpvt[j - 1] < 0) {
        jpvt[j - 1] = -jpvt[j - 1];
        if (j != pu) {
          dswap(n, a + 0 + (pu - 1)*lda, 1, a + 0 + (j - 1)*lda, 1);
          jp = jpvt[pu - 1];
          jpvt[pu - 1] = jpvt[j - 1];
          jpvt[j - 1] = jp;
        }
        pu = pu - 1;
      }
    }
  }
  /**
    Compute the norms of the free columns.
  */
  for (j = pl; j <= pu; j++)
    qraux[j - 1] = dnrm2(n, a + 0 + (j - 1) * lda, 1);
  for (j = pl; j <= pu; j++)
    work[j - 1] = qraux[j - 1];
  /**
    Perform the Householder reduction of A.
  */
  lup = i4_min(n, p);
  for (int l = 1; l <= lup; l++) {
    /**
      Bring the column of largest norm into the pivot position.
    */
    if (pl <= l && l < pu) {
      maxnrm = 0.0;
      maxj = l;
      for (j = l; j <= pu; j++) {
        if (maxnrm < qraux[j - 1]) {
          maxnrm = qraux[j - 1];
          maxj = j;
        }
      }
      if (maxj != l) {
        dswap(n, a + 0 + (l - 1)*lda, 1, a + 0 + (maxj - 1)*lda, 1);
        qraux[maxj - 1] = qraux[l - 1];
        work[maxj - 1] = work[l - 1];
        jp = jpvt[maxj - 1];
        jpvt[maxj - 1] = jpvt[l - 1];
        jpvt[l - 1] = jp;
      }
    }
    /**
      Compute the Householder transformation for column L.
    */
    qraux[l - 1] = 0.0;
    if (l != n) {
      nrmxl = dnrm2(n - l + 1, a + l - 1 + (l - 1) * lda, 1);
      if (nrmxl != 0.0) {
        if (a[l - 1 + (l - 1)*lda] != 0.0)
          nrmxl = nrmxl * r8_sign(a[l - 1 + (l - 1) * lda]);
        dscal(n - l + 1, 1.0 / nrmxl, a + l - 1 + (l - 1)*lda, 1);
        a[l - 1 + (l - 1)*lda] = 1.0 + a[l - 1 + (l - 1) * lda];
        /**
          Apply the transformation to the remaining columns, updating the norms.
        */
        for (j = l + 1; j <= p; j++) {
          t = -ddot(n - l + 1, a + l - 1 + (l - 1) * lda, 1, a + l - 1 + (j - 1) * lda, 1)
              / a[l - 1 + (l - 1) * lda];
          daxpy(n - l + 1, t, a + l - 1 + (l - 1)*lda, 1, a + l - 1 + (j - 1)*lda, 1);
          if (pl <= j && j <= pu) {
            if (qraux[j - 1] != 0.0) {
              tt = 1.0 - pow(r8_abs(a[l - 1 + (j - 1) * lda]) / qraux[j - 1], 2);
              tt = r8_max(tt, 0.0);
              t = tt;
              tt = 1.0 + 0.05 * tt * pow(qraux[j - 1] / work[j - 1], 2);
              if (tt != 1.0)
                qraux[j - 1] = qraux[j - 1] * sqrt(t);
              else {
                qraux[j - 1] = dnrm2(n - l, a + l + (j - 1) * lda, 1);
                work[j - 1] = qraux[j - 1];
              }
            }
          }
        }
        /**
          Save the transformation.
        */
        qraux[l - 1] = a[l - 1 + (l - 1) * lda];
        a[l - 1 + (l - 1)*lda] = -nrmxl;
      }
    }
  }
}
/******************************************************************************/

int dqrls(double a[], int lda, int m, int n, double tol, int* kr, double b[],
          double x[], double rsd[], int jpvt[], double qraux[], int itask)

/******************************************************************************/
/**
  Purpose:

    DQRLS factors and solves a linear system in the least squares sense.

  Discussion:

    The linear system may be overdetermined, underdetermined or singular.
    The solution is obtained using a QR factorization of the
    coefficient matrix.

    DQRLS can be efficiently used to solve several least squares
    problems with the same matrix A.  The first system is solved
    with ITASK = 1.  The subsequent systems are solved with
    ITASK = 2, to avoid the recomputation of the matrix factors.
    The parameters KR, JPVT, and QRAUX must not be modified
    between calls to DQRLS.

    DQRLS is used to solve in a least squares sense
    overdetermined, underdetermined and singular linear systems.
    The system is A*X approximates B where A is M by N.
    B is a given M-vector, and X is the N-vector to be computed.
    A solution X is found which minimimzes the sum of squares (2-norm)
    of the residual,  A*X - B.

    The numerical rank of A is determined using the tolerance TOL.

    DQRLS uses the LINPACK subroutine DQRDC to compute the QR
    factorization, with column pivoting, of an M by N matrix A.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    10 September 2012

  Author:

    C version by John Burkardt.

  Reference:

    David Kahaner, Cleve Moler, Steven Nash,
    Numerical Methods and Software,
    Prentice Hall, 1989,
    ISBN: 0-13-627258-4,
    LC: TA345.K34.

  Parameters:

    Input/output, double A[LDA*N], an M by N matrix.
    On input, the matrix whose decomposition is to be computed.
    In a least squares data fitting problem, A(I,J) is the
    value of the J-th basis (model) function at the I-th data point.
    On output, A contains the output from DQRDC.  The triangular matrix R
    of the QR factorization is contained in the upper triangle and
    information needed to recover the orthogonal matrix Q is stored
    below the diagonal in A and in the vector QRAUX.

    Input, int LDA, the leading dimension of A.

    Input, int M, the number of rows of A.

    Input, int N, the number of columns of A.

    Input, double TOL, a relative tolerance used to determine the
    numerical rank.  The problem should be scaled so that all the elements
    of A have roughly the same absolute accuracy EPS.  Then a reasonable
    value for TOL is roughly EPS divided by the magnitude of the largest
    element.

    Output, int *KR, the numerical rank.

    Input, double B[M], the right hand side of the linear system.

    Output, double X[N], a least squares solution to the linear
    system.

    Output, double RSD[M], the residual, B - A*X.  RSD may
    overwrite B.

    Workspace, int JPVT[N], required if ITASK = 1.
    Columns JPVT(1), ..., JPVT(KR) of the original matrix are linearly
    independent to within the tolerance TOL and the remaining columns
    are linearly dependent.  ABS ( A(1,1) ) / ABS ( A(KR,KR) ) is an estimate
    of the condition number of the matrix of independent columns,
    and of R.  This estimate will be <= 1/TOL.

    Workspace, double QRAUX[N], required if ITASK = 1.

    Input, int ITASK.
    1, DQRLS factors the matrix A and solves the least squares problem.
    2, DQRLS assumes that the matrix A was factored with an earlier
       call to DQRLS, and only solves the least squares problem.

    Output, int DQRLS, error code.
    0:  no error
    -1: LDA < M   (fatal error)
    -2: N < 1     (fatal error)
    -3: ITASK < 1 (fatal error)
*/
{
  int ind;
  if (lda < m) {
    /*fprintf ( stderr, "\n" );
    fprintf ( stderr, "DQRLS - Fatal error!\n" );
    fprintf ( stderr, "  LDA < M.\n" );*/
    ind = -1;
    return ind;
  }

  if (n <= 0) {
    /*fprintf ( stderr, "\n" );
    fprintf ( stderr, "DQRLS - Fatal error!\n" );
    fprintf ( stderr, "  N <= 0.\n" );*/
    ind = -2;
    return ind;
  }

  if (itask < 1) {
    /*fprintf ( stderr, "\n" );
    fprintf ( stderr, "DQRLS - Fatal error!\n" );
    fprintf ( stderr, "  ITASK < 1.\n" );*/
    ind = -3;
    return ind;
  }

  ind = 0;
  /**
    Factor the matrix.
  */
  if (itask == 1)
    dqrank(a, lda, m, n, tol, kr, jpvt, qraux);
  /**
    Solve the least-squares problem.
  */
  dqrlss(a, lda, m, n, *kr, b, x, rsd, jpvt, qraux);
  return ind;
}
/******************************************************************************/

void dqrlss(double a[], int lda, int m, int n, int kr, double b[], double x[],
            double rsd[], int jpvt[], double qraux[])

/******************************************************************************/
/**
  Purpose:

    DQRLSS solves a linear system in a least squares sense.

  Discussion:

    DQRLSS must be preceded by a call to DQRANK.

    The system is to be solved is
      A * X = B
    where
      A is an M by N matrix with rank KR, as determined by DQRANK,
      B is a given M-vector,
      X is the N-vector to be computed.

    A solution X, with at most KR nonzero components, is found which
    minimizes the 2-norm of the residual (A*X-B).

    Once the matrix A has been formed, DQRANK should be
    called once to decompose it.  Then, for each right hand
    side B, DQRLSS should be called once to obtain the
    solution and residual.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    10 September 2012

  Author:

    C version by John Burkardt

  Parameters:

    Input, double A[LDA*N], the QR factorization information
    from DQRANK.  The triangular matrix R of the QR factorization is
    contained in the upper triangle and information needed to recover
    the orthogonal matrix Q is stored below the diagonal in A and in
    the vector QRAUX.

    Input, int LDA, the leading dimension of A, which must
    be at least M.

    Input, int M, the number of rows of A.

    Input, int N, the number of columns of A.

    Input, int KR, the rank of the matrix, as estimated by DQRANK.

    Input, double B[M], the right hand side of the linear system.

    Output, double X[N], a least squares solution to the
    linear system.

    Output, double RSD[M], the residual, B - A*X.  RSD may
    overwrite B.

    Input, int JPVT[N], the pivot information from DQRANK.
    Columns JPVT[0], ..., JPVT[KR-1] of the original matrix are linearly
    independent to within the tolerance TOL and the remaining columns
    are linearly dependent.

    Input, double QRAUX[N], auxiliary information from DQRANK
    defining the QR factorization.
*/
{
  int i;
  int info;
  int j;
  int job;
  int k;
  double t;

  if (kr != 0) {
    job = 110;
    info = dqrsl(a, lda, m, kr, qraux, b, rsd, rsd, x, rsd, rsd, job); UNUSED(info);
  }

  for (i = 0; i < n; i++)
    jpvt[i] = - jpvt[i];

  for (i = kr; i < n; i++)
    x[i] = 0.0;

  for (j = 1; j <= n; j++) {
    if (jpvt[j - 1] <= 0) {
      k = - jpvt[j - 1];
      jpvt[j - 1] = k;

      while (k != j) {
        t = x[j - 1];
        x[j - 1] = x[k - 1];
        x[k - 1] = t;
        jpvt[k - 1] = -jpvt[k - 1];
        k = jpvt[k - 1];
      }
    }
  }
}
/******************************************************************************/

int dqrsl(double a[], int lda, int n, int k, double qraux[], double y[],
          double qy[], double qty[], double b[], double rsd[], double ab[], int job)

/******************************************************************************/
/**
  Purpose:

    DQRSL computes transformations, projections, and least squares solutions.

  Discussion:

    DQRSL requires the output of DQRDC.

    For K <= min(N,P), let AK be the matrix

      AK = ( A(JPVT[0]), A(JPVT(2)), ..., A(JPVT(K)) )

    formed from columns JPVT[0], ..., JPVT(K) of the original
    N by P matrix A that was input to DQRDC.  If no pivoting was
    done, AK consists of the first K columns of A in their
    original order.  DQRDC produces a factored orthogonal matrix Q
    and an upper triangular matrix R such that

      AK = Q * (R)
               (0)

    This information is contained in coded form in the arrays
    A and QRAUX.

    The parameters QY, QTY, B, RSD, and AB are not referenced
    if their computation is not requested and in this case
    can be replaced by dummy variables in the calling program.
    To save storage, the user may in some cases use the same
    array for different parameters in the calling sequence.  A
    frequently occurring example is when one wishes to compute
    any of B, RSD, or AB and does not need Y or QTY.  In this
    case one may identify Y, QTY, and one of B, RSD, or AB, while
    providing separate arrays for anything else that is to be
    computed.

    Thus the calling sequence

      dqrsl ( a, lda, n, k, qraux, y, dum, y, b, y, dum, 110, info )

    will result in the computation of B and RSD, with RSD
    overwriting Y.  More generally, each item in the following
    list contains groups of permissible identifications for
    a single calling sequence.

      1. (Y,QTY,B) (RSD) (AB) (QY)

      2. (Y,QTY,RSD) (B) (AB) (QY)

      3. (Y,QTY,AB) (B) (RSD) (QY)

      4. (Y,QY) (QTY,B) (RSD) (AB)

      5. (Y,QY) (QTY,RSD) (B) (AB)

      6. (Y,QY) (QTY,AB) (B) (RSD)

    In any group the value returned in the array allocated to
    the group corresponds to the last member of the group.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    07 June 2005

  Author:

    C version by John Burkardt.

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch and Pete Stewart,
    LINPACK User's Guide,
    SIAM, (Society for Industrial and Applied Mathematics),
    3600 University City Science Center,
    Philadelphia, PA, 19104-2688.
    ISBN 0-89871-172-X

  Parameters:

    Input, double A[LDA*P], contains the output of DQRDC.

    Input, int LDA, the leading dimension of the array A.

    Input, int N, the number of rows of the matrix AK.  It must
    have the same value as N in DQRDC.

    Input, int K, the number of columns of the matrix AK.  K
    must not be greater than min(N,P), where P is the same as in the
    calling sequence to DQRDC.

    Input, double QRAUX[P], the auxiliary output from DQRDC.

    Input, double Y[N], a vector to be manipulated by DQRSL.

    Output, double QY[N], contains Q * Y, if requested.

    Output, double QTY[N], contains Q' * Y, if requested.

    Output, double B[K], the solution of the least squares problem
      minimize norm2 ( Y - AK * B),
    if its computation has been requested.  Note that if pivoting was
    requested in DQRDC, the J-th component of B will be associated with
    column JPVT(J) of the original matrix A that was input into DQRDC.

    Output, double RSD[N], the least squares residual Y - AK * B,
    if its computation has been requested.  RSD is also the orthogonal
    projection of Y onto the orthogonal complement of the column space
    of AK.

    Output, double AB[N], the least squares approximation Ak * B,
    if its computation has been requested.  AB is also the orthogonal
    projection of Y onto the column space of A.

    Input, integer JOB, specifies what is to be computed.  JOB has
    the decimal expansion ABCDE, with the following meaning:

      if A != 0, compute QY.
      if B != 0, compute QTY.
      if C != 0, compute QTY and B.
      if D != 0, compute QTY and RSD.
      if E != 0, compute QTY and AB.

    Note that a request to compute B, RSD, or AB automatically triggers
    the computation of QTY, for which an array must be provided in the
    calling sequence.

    Output, int DQRSL, is zero unless the computation of B has
    been requested and R is exactly singular.  In this case, INFO is the
    index of the first zero diagonal element of R, and B is left unaltered.
*/
{
  int cab;
  int cb;
  int cqty;
  int cqy;
  int cr;
  int i;
  int info;
  int j;
  int jj;
  int ju;
  double t;
  double temp;
  /**
    Set INFO flag.
  */
  info = 0;

  /**
    Determine what is to be computed.
  */
  cqy  = ( job / 10000        != 0);
  cqty = ((job % 10000)       != 0);
  cb   = ((job %  1000) / 100 != 0);
  cr   = ((job %   100) /  10 != 0);
  cab  = ((job %    10)       != 0);
  ju = i4_min(k, n - 1);

  /**
    Special action when N = 1.
  */
  if (ju == 0) {
    if (cqy)
      qy[0] = y[0];
    if (cqty)
      qty[0] = y[0];
    if (cab)
      ab[0] = y[0];
    if (cb) {
      if (a[0 + 0 * lda] == 0.0)
        info = 1;
      else
        b[0] = y[0] / a[0 + 0 * lda];
    }
    if (cr)
      rsd[0] = 0.0;
    return info;
  }
  /**
    Set up to compute QY or QTY.
  */
  if (cqy) {
    for (i = 1; i <= n; i++)
      qy[i - 1] = y[i - 1];
  }
  if (cqty) {
    for (i = 1; i <= n; i++)
      qty[i - 1] = y[i - 1];
  }
  /**
    Compute QY.
  */
  if (cqy) {
    for (jj = 1; jj <= ju; jj++) {
      j = ju - jj + 1;
      if (qraux[j - 1] != 0.0) {
        temp = a[j - 1 + (j - 1) * lda];
        a[j - 1 + (j - 1)*lda] = qraux[j - 1];
        t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, qy + j - 1, 1) / a[j - 1 + (j - 1) * lda];
        daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, qy + j - 1, 1);
        a[j - 1 + (j - 1)*lda] = temp;
      }
    }
  }
  /**
    Compute Q'*Y.
  */
  if (cqty) {
    for (j = 1; j <= ju; j++) {
      if (qraux[j - 1] != 0.0) {
        temp = a[j - 1 + (j - 1) * lda];
        a[j - 1 + (j - 1)*lda] = qraux[j - 1];
        t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, qty + j - 1, 1) / a[j - 1 + (j - 1) * lda];
        daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, qty + j - 1, 1);
        a[j - 1 + (j - 1)*lda] = temp;
      }
    }
  }
  /**
    Set up to compute B, RSD, or AB.
  */
  if (cb) {
    for (i = 1; i <= k; i++)
      b[i - 1] = qty[i - 1];
  }
  if (cab) {
    for (i = 1; i <= k; i++)
      ab[i - 1] = qty[i - 1];
  }
  if (cr && k < n) {
    for (i = k + 1; i <= n; i++)
      rsd[i - 1] = qty[i - 1];
  }
  if (cab && k + 1 <= n) {
    for (i = k + 1; i <= n; i++)
      ab[i - 1] = 0.0;
  }
  if (cr) {
    for (i = 1; i <= k; i++)
      rsd[i - 1] = 0.0;
  }
  /**
    Compute B.
  */
  if (cb) {
    for (jj = 1; jj <= k; jj++) {
      j = k - jj + 1;
      if (a[j - 1 + (j - 1)*lda] == 0.0) {
        info = j;
        break;
      }
      b[j - 1] = b[j - 1] / a[j - 1 + (j - 1) * lda];
      if (j != 1) {
        t = -b[j - 1];
        daxpy(j - 1, t, a + 0 + (j - 1)*lda, 1, b, 1);
      }
    }
  }
  /**
    Compute RSD or AB as required.
  */
  if (cr || cab) {
    for (jj = 1; jj <= ju; jj++) {
      j = ju - jj + 1;
      if (qraux[j - 1] != 0.0) {
        temp = a[j - 1 + (j - 1) * lda];
        a[j - 1 + (j - 1)*lda] = qraux[j - 1];
        if (cr) {
          t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, rsd + j - 1, 1)
              / a[j - 1 + (j - 1) * lda];
          daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, rsd + j - 1, 1);
        }
        if (cab) {
          t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, ab + j - 1, 1)
              / a[j - 1 + (j - 1) * lda];
          daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, ab + j - 1, 1);
        }
        a[j - 1 + (j - 1)*lda] = temp;
      }
    }
  }
  return info;
}
/******************************************************************************/

/******************************************************************************/

void dscal(int n, double sa, double x[], int incx)

/******************************************************************************/
/**
  Purpose:

    DSCAL scales a vector by a constant.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 March 2007

  Author:

    C version by John Burkardt

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch, Pete Stewart,
    LINPACK User's Guide,
    SIAM, 1979.

    Charles Lawson, Richard Hanson, David Kincaid, Fred Krogh,
    Basic Linear Algebra Subprograms for Fortran Usage,
    Algorithm 539,
    ACM Transactions on Mathematical Software,
    Volume 5, Number 3, September 1979, pages 308-323.

  Parameters:

    Input, int N, the number of entries in the vector.

    Input, double SA, the multiplier.

    Input/output, double X[*], the vector to be scaled.

    Input, int INCX, the increment between successive entries of X.
*/
{
  int i;
  int ix;
  int m;

  if (n <= 0) return;

  if (incx == 1) {
    m = n % 5;
    for (i = 0; i < m; i++)
      x[i] = sa * x[i];
    for (i = m; i < n; i = i + 5) {
      x[i]   = sa * x[i];
      x[i + 1] = sa * x[i + 1];
      x[i + 2] = sa * x[i + 2];
      x[i + 3] = sa * x[i + 3];
      x[i + 4] = sa * x[i + 4];
    }
  }
  else {
    if (0 <= incx)
      ix = 0;
    else
      ix = (- n + 1) * incx;
    for (i = 0; i < n; i++) {
      x[ix] = sa * x[ix];
      ix = ix + incx;
    }
  }
}
/******************************************************************************/


void dswap(int n, double x[], int incx, double y[], int incy)

/******************************************************************************/
/**
  Purpose:

    DSWAP interchanges two vectors.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 March 2007

  Author:

    C version by John Burkardt

  Reference:

    Jack Dongarra, Cleve Moler, Jim Bunch, Pete Stewart,
    LINPACK User's Guide,
    SIAM, 1979.

    Charles Lawson, Richard Hanson, David Kincaid, Fred Krogh,
    Basic Linear Algebra Subprograms for Fortran Usage,
    Algorithm 539,
    ACM Transactions on Mathematical Software,
    Volume 5, Number 3, September 1979, pages 308-323.

  Parameters:

    Input, int N, the number of entries in the vectors.

    Input/output, double X[*], one of the vectors to swap.

    Input, int INCX, the increment between successive entries of X.

    Input/output, double Y[*], one of the vectors to swap.

    Input, int INCY, the increment between successive elements of Y.
*/
{
  if (n <= 0) return;

  int i, ix, iy, m;
  double temp;

  if (incx == 1 && incy == 1) {
    m = n % 3;
    for (i = 0; i < m; i++) {
      temp = x[i];
      x[i] = y[i];
      y[i] = temp;
    }
    for (i = m; i < n; i = i + 3) {
      temp = x[i];
      x[i] = y[i];
      y[i] = temp;
      temp = x[i + 1];
      x[i + 1] = y[i + 1];
      y[i + 1] = temp;
      temp = x[i + 2];
      x[i + 2] = y[i + 2];
      y[i + 2] = temp;
    }
  }
  else {
    ix = (incx >= 0) ? 0 : (-n + 1) * incx;
    iy = (incy >= 0) ? 0 : (-n + 1) * incy;
    for (i = 0; i < n; i++) {
      temp = x[ix];
      x[ix] = y[iy];
      y[iy] = temp;
      ix = ix + incx;
      iy = iy + incy;
    }
  }
}
/******************************************************************************/

/******************************************************************************/

void qr_solve(double x[], int m, int n, double a[], double b[])

/******************************************************************************/
/**
  Purpose:

    QR_SOLVE solves a linear system in the least squares sense.

  Discussion:

    If the matrix A has full column rank, then the solution X should be the
    unique vector that minimizes the Euclidean norm of the residual.

    If the matrix A does not have full column rank, then the solution is
    not unique; the vector X will minimize the residual norm, but so will
    various other vectors.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    11 September 2012

  Author:

    John Burkardt

  Reference:

    David Kahaner, Cleve Moler, Steven Nash,
    Numerical Methods and Software,
    Prentice Hall, 1989,
    ISBN: 0-13-627258-4,
    LC: TA345.K34.

  Parameters:

    Input, int M, the number of rows of A.

    Input, int N, the number of columns of A.

    Input, double A[M*N], the matrix.

    Input, double B[M], the right hand side.

    Output, double QR_SOLVE[N], the least squares solution.
*/
{
  double a_qr[n * m], qraux[n], r[m], tol;
  int ind, itask, jpvt[n], kr, lda;

  r8mat_copy(a_qr, m, n, a);
  lda = m;
  tol = r8_epsilon() / r8mat_amax(m, n, a_qr);
  itask = 1;

  ind = dqrls(a_qr, lda, m, n, tol, &kr, b, x, r, jpvt, qraux, itask); UNUSED(ind);
}
/******************************************************************************/

#endif // AUTO_BED_LEVELING_GRID
