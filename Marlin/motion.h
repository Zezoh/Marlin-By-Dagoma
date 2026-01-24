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
 * motion.h - Consolidated motion control: planner, stepper, and bed leveling
 * 
 * This file consolidates the following modules:
 * - planner.h - Motion planning and acceleration profiles
 * - stepper.h - Stepper motor driver and execution
 * - stepper_indirection.h - Stepper driver indirection macros
 * - qr_solve.h - QR factorization for bed leveling
 * - vector_3.h - Vector library for coordinate transformations
 */

#ifndef MOTION_H
#define MOTION_H

#include "Marlin.h"
#include "macros.h"

//===========================================================================
//============================= Vector 3 Types ==============================
//===========================================================================

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

class matrix_3x3;

struct vector_3 {
  float x, y, z;

  vector_3();
  vector_3(float x, float y, float z);

  static vector_3 cross(vector_3 a, vector_3 b);

  vector_3 operator+(vector_3 v);
  vector_3 operator-(vector_3 v);
  void normalize();
  float get_length();
  vector_3 get_normal();

  void debug(const char title[]);

  void apply_rotation(matrix_3x3 matrix);
};

struct matrix_3x3 {
  float matrix[9];

  static matrix_3x3 create_from_rows(vector_3 row_0, vector_3 row_1, vector_3 row_2);
  static matrix_3x3 create_look_at(vector_3 target);
  static matrix_3x3 transpose(matrix_3x3 original);

  void set_to_identity();

  void debug(const char title[]);
};

void apply_rotation_xyz(matrix_3x3 rotationMatrix, float& x, float& y, float& z);

#endif // AUTO_BED_LEVELING_FEATURE

//===========================================================================
//============================= Planner Block Type ==========================
//===========================================================================

typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps[NUM_AXIS];                     // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  #if ENABLED(ADVANCE)
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
  float nominal_speed;                               // The nominal speed for this block in mm/sec
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2

  #if FAN_COUNT > 0
    unsigned long fan_speed[FAN_COUNT];
  #endif

  #if ENABLED(BARICUDA)
    unsigned long valve_pressure;
    unsigned long e_to_p_pressure;
  #endif

  volatile char busy;

} block_t;

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

//===========================================================================
//==================== Stepper Indirection Macros ===========================
//===========================================================================

// X motor
#define X_STEP_INIT SET_OUTPUT(X_STEP_PIN)
#define X_STEP_WRITE(STATE) WRITE(X_STEP_PIN,STATE)
#define X_STEP_READ READ(X_STEP_PIN)

#define X_DIR_INIT SET_OUTPUT(X_DIR_PIN)
#define X_DIR_WRITE(STATE) WRITE(X_DIR_PIN,STATE)
#define X_DIR_READ READ(X_DIR_PIN)

#define X_ENABLE_INIT SET_OUTPUT(X_ENABLE_PIN)
#define X_ENABLE_WRITE(STATE) WRITE(X_ENABLE_PIN,STATE)
#define X_ENABLE_READ READ(X_ENABLE_PIN)

// X2 motor
#define X2_STEP_INIT SET_OUTPUT(X2_STEP_PIN)
#define X2_STEP_WRITE(STATE) WRITE(X2_STEP_PIN,STATE)
#define X2_STEP_READ READ(X2_STEP_PIN)

#define X2_DIR_INIT SET_OUTPUT(X2_DIR_PIN)
#define X2_DIR_WRITE(STATE) WRITE(X2_DIR_PIN,STATE)
#define X2_DIR_READ READ(X2_DIR_PIN)

#define X2_ENABLE_INIT SET_OUTPUT(X2_ENABLE_PIN)
#define X2_ENABLE_WRITE(STATE) WRITE(X2_ENABLE_PIN,STATE)
#define X2_ENABLE_READ READ(X2_ENABLE_PIN)

// Y motor
#define Y_STEP_INIT SET_OUTPUT(Y_STEP_PIN)
#define Y_STEP_WRITE(STATE) WRITE(Y_STEP_PIN,STATE)
#define Y_STEP_READ READ(Y_STEP_PIN)

#define Y_DIR_INIT SET_OUTPUT(Y_DIR_PIN)
#define Y_DIR_WRITE(STATE) WRITE(Y_DIR_PIN,STATE)
#define Y_DIR_READ READ(Y_DIR_PIN)

#define Y_ENABLE_INIT SET_OUTPUT(Y_ENABLE_PIN)
#define Y_ENABLE_WRITE(STATE) WRITE(Y_ENABLE_PIN,STATE)
#define Y_ENABLE_READ READ(Y_ENABLE_PIN)

// Y2 motor
#define Y2_STEP_INIT SET_OUTPUT(Y2_STEP_PIN)
#define Y2_STEP_WRITE(STATE) WRITE(Y2_STEP_PIN,STATE)
#define Y2_STEP_READ READ(Y2_STEP_PIN)

#define Y2_DIR_INIT SET_OUTPUT(Y2_DIR_PIN)
#define Y2_DIR_WRITE(STATE) WRITE(Y2_DIR_PIN,STATE)
#define Y2_DIR_READ READ(Y2_DIR_PIN)

#define Y2_ENABLE_INIT SET_OUTPUT(Y2_ENABLE_PIN)
#define Y2_ENABLE_WRITE(STATE) WRITE(Y2_ENABLE_PIN,STATE)
#define Y2_ENABLE_READ READ(Y2_ENABLE_PIN)

// Z motor
#define Z_STEP_INIT SET_OUTPUT(Z_STEP_PIN)
#define Z_STEP_WRITE(STATE) WRITE(Z_STEP_PIN,STATE)
#define Z_STEP_READ READ(Z_STEP_PIN)

#define Z_DIR_INIT SET_OUTPUT(Z_DIR_PIN)
#define Z_DIR_WRITE(STATE) WRITE(Z_DIR_PIN,STATE)
#define Z_DIR_READ READ(Z_DIR_PIN)

#define Z_ENABLE_INIT SET_OUTPUT(Z_ENABLE_PIN)
#define Z_ENABLE_WRITE(STATE) WRITE(Z_ENABLE_PIN,STATE)
#define Z_ENABLE_READ READ(Z_ENABLE_PIN)

// Z2 motor
#define Z2_STEP_INIT SET_OUTPUT(Z2_STEP_PIN)
#define Z2_STEP_WRITE(STATE) WRITE(Z2_STEP_PIN,STATE)
#define Z2_STEP_READ READ(Z2_STEP_PIN)

#define Z2_DIR_INIT SET_OUTPUT(Z2_DIR_PIN)
#define Z2_DIR_WRITE(STATE) WRITE(Z2_DIR_PIN,STATE)
#define Z2_DIR_READ READ(Z2_DIR_PIN)

#define Z2_ENABLE_INIT SET_OUTPUT(Z2_ENABLE_PIN)
#define Z2_ENABLE_WRITE(STATE) WRITE(Z2_ENABLE_PIN,STATE)
#define Z2_ENABLE_READ READ(Z2_ENABLE_PIN)

// E0 motor
#define E0_STEP_INIT SET_OUTPUT(E0_STEP_PIN)
#define E0_STEP_WRITE(STATE) WRITE(E0_STEP_PIN,STATE)
#define E0_STEP_READ READ(E0_STEP_PIN)

#define E0_DIR_INIT SET_OUTPUT(E0_DIR_PIN)
#define E0_DIR_WRITE(STATE) WRITE(E0_DIR_PIN,STATE)
#define E0_DIR_READ READ(E0_DIR_PIN)

#define E0_ENABLE_INIT SET_OUTPUT(E0_ENABLE_PIN)
#define E0_ENABLE_WRITE(STATE) WRITE(E0_ENABLE_PIN,STATE)
#define E0_ENABLE_READ READ(E0_ENABLE_PIN)

// E1 motor
#define E1_STEP_INIT SET_OUTPUT(E1_STEP_PIN)
#define E1_STEP_WRITE(STATE) WRITE(E1_STEP_PIN,STATE)
#define E1_STEP_READ READ(E1_STEP_PIN)

#define E1_DIR_INIT SET_OUTPUT(E1_DIR_PIN)
#define E1_DIR_WRITE(STATE) WRITE(E1_DIR_PIN,STATE)
#define E1_DIR_READ READ(E1_DIR_PIN)

#define E1_ENABLE_INIT SET_OUTPUT(E1_ENABLE_PIN)
#define E1_ENABLE_WRITE(STATE) WRITE(E1_ENABLE_PIN,STATE)
#define E1_ENABLE_READ READ(E1_ENABLE_PIN)

// E2 motor
#define E2_STEP_INIT SET_OUTPUT(E2_STEP_PIN)
#define E2_STEP_WRITE(STATE) WRITE(E2_STEP_PIN,STATE)
#define E2_STEP_READ READ(E2_STEP_PIN)

#define E2_DIR_INIT SET_OUTPUT(E2_DIR_PIN)
#define E2_DIR_WRITE(STATE) WRITE(E2_DIR_PIN,STATE)
#define E2_DIR_READ READ(E2_DIR_PIN)

#define E2_ENABLE_INIT SET_OUTPUT(E2_ENABLE_PIN)
#define E2_ENABLE_WRITE(STATE) WRITE(E2_ENABLE_PIN,STATE)
#define E2_ENABLE_READ READ(E2_ENABLE_PIN)

// E3 motor
#define E3_STEP_INIT SET_OUTPUT(E3_STEP_PIN)
#define E3_STEP_WRITE(STATE) WRITE(E3_STEP_PIN,STATE)
#define E3_STEP_READ READ(E3_STEP_PIN)

#define E3_DIR_INIT SET_OUTPUT(E3_DIR_PIN)
#define E3_DIR_WRITE(STATE) WRITE(E3_DIR_PIN,STATE)
#define E3_DIR_READ READ(E3_DIR_PIN)

#define E3_ENABLE_INIT SET_OUTPUT(E3_ENABLE_PIN)
#define E3_ENABLE_WRITE(STATE) WRITE(E3_ENABLE_PIN,STATE)
#define E3_ENABLE_READ READ(E3_ENABLE_PIN)

#if EXTRUDERS > 3
  #define E_STEP_WRITE(v) {switch(current_block->active_extruder){case 3:E3_STEP_WRITE(v);break;case 2:E2_STEP_WRITE(v);break;case 1:E1_STEP_WRITE(v);break;default:E0_STEP_WRITE(v);}}
  #define NORM_E_DIR() {switch(current_block->active_extruder){case 3:E3_DIR_WRITE(!INVERT_E3_DIR);break;case 2:E2_DIR_WRITE(!INVERT_E2_DIR);break;case 1:E1_DIR_WRITE(!INVERT_E1_DIR);break;default:E0_DIR_WRITE(!INVERT_E0_DIR);}}
  #define REV_E_DIR() {switch(current_block->active_extruder){case 3:E3_DIR_WRITE(INVERT_E3_DIR);break;case 2:E2_DIR_WRITE(INVERT_E2_DIR);break;case 1:E1_DIR_WRITE(INVERT_E1_DIR);break;default:E0_DIR_WRITE(INVERT_E0_DIR);}}
#elif EXTRUDERS > 2
  #define E_STEP_WRITE(v) {switch(current_block->active_extruder){case 2:E2_STEP_WRITE(v);break;case 1:E1_STEP_WRITE(v);break;default:E0_STEP_WRITE(v);}}
  #define NORM_E_DIR() {switch(current_block->active_extruder){case 2:E2_DIR_WRITE(!INVERT_E2_DIR);break;case 1:E1_DIR_WRITE(!INVERT_E1_DIR);break;default:E0_DIR_WRITE(!INVERT_E0_DIR);}}
  #define REV_E_DIR() {switch(current_block->active_extruder){case 2:E2_DIR_WRITE(INVERT_E2_DIR);break;case 1:E1_DIR_WRITE(INVERT_E1_DIR);break;default:E0_DIR_WRITE(INVERT_E0_DIR);}}
#elif EXTRUDERS > 1
  #define _E_STEP_WRITE(v) {if(current_block->active_extruder==1){E1_STEP_WRITE(v);}else{E0_STEP_WRITE(v);}}
  #define _NORM_E_DIR() {if(current_block->active_extruder==1){E1_DIR_WRITE(!INVERT_E1_DIR);}else{E0_DIR_WRITE(!INVERT_E0_DIR);}}
  #define _REV_E_DIR() {if(current_block->active_extruder==1){E1_DIR_WRITE(INVERT_E1_DIR);}else{E0_DIR_WRITE(INVERT_E0_DIR);}}
  // DUAL_X_CARRIAGE removed - Delta-only firmware
  #define E_STEP_WRITE(v) _E_STEP_WRITE(v)
  #define NORM_E_DIR() _NORM_E_DIR()
  #define REV_E_DIR() _REV_E_DIR()
#else
  #define E_STEP_WRITE(v) E0_STEP_WRITE(v)
  #define NORM_E_DIR() E0_DIR_WRITE(!INVERT_E0_DIR)
  #define REV_E_DIR() E0_DIR_WRITE(INVERT_E0_DIR)
#endif

//===========================================================================
//===================== QR Solve Function Declarations ======================
//===========================================================================

#if ENABLED(AUTO_BED_LEVELING_GRID)

void daxpy(int n, double da, double dx[], int incx, double dy[], int incy);
double ddot(int n, double dx[], int incx, double dy[], int incy);
double dnrm2(int n, double x[], int incx);
void dqrank(double a[], int lda, int m, int n, double tol, int* kr,
            int jpvt[], double qraux[]);
void dqrdc(double a[], int lda, int n, int p, double qraux[], int jpvt[],
           double work[], int job);
int dqrls(double a[], int lda, int m, int n, double tol, int* kr, double b[],
          double x[], double rsd[], int jpvt[], double qraux[], int itask);
void dqrlss(double a[], int lda, int m, int n, int kr, double b[], double x[],
            double rsd[], int jpvt[], double qraux[]);
int dqrsl(double a[], int lda, int n, int k, double qraux[], double y[],
          double qy[], double qty[], double b[], double rsd[], double ab[], int job);
void dscal(int n, double sa, double x[], int incx);
void dswap(int n, double x[], int incx, double y[], int incy);
void qr_solve(double x[], int m, int n, double a[], double b[]);

#endif // AUTO_BED_LEVELING_GRID

//===========================================================================
//===================== Planner Function Declarations =======================
//===========================================================================

// Initialize the motion plan subsystem
void plan_init();

void check_axes_activity();

// Get the number of buffered moves
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;
FORCE_INLINE uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  extern matrix_3x3 plan_bed_level_matrix;
  vector_3 plan_get_position();
  void plan_buffer_line(float x, float y, float z, const float& e, float feed_rate, const uint8_t extruder);
  void plan_set_position(float x, float y, float z, const float& e);
#else
  void plan_buffer_line(const float& x, const float& y, const float& z, const float& e, float feed_rate, const uint8_t extruder);
  void plan_set_position(const float& x, const float& y, const float& z, const float& e);
#endif // AUTO_BED_LEVELING_FEATURE

void plan_set_e_position(const float& e);

extern millis_t minsegmenttime;
extern float max_feedrate[NUM_AXIS];
extern float axis_steps_per_unit[NUM_AXIS];
extern unsigned long max_acceleration_units_per_sq_second[NUM_AXIS];
extern float minimumfeedrate;
extern float acceleration;
extern float retract_acceleration;
extern float travel_acceleration;
extern float max_xy_jerk;
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#if ENABLED(AUTOTEMP)
  extern bool autotemp_enabled;
  extern float autotemp_max;
  extern float autotemp_min;
  extern float autotemp_factor;
#endif

extern block_t block_buffer[BLOCK_BUFFER_SIZE];
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;

FORCE_INLINE bool blocks_queued() { return (block_buffer_head != block_buffer_tail); }

FORCE_INLINE void plan_discard_current_block() {
  if (blocks_queued())
    block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
}

FORCE_INLINE block_t* plan_get_current_block() {
  if (blocks_queued()) {
    block_t* block = &block_buffer[block_buffer_tail];
    block->busy = true;
    return block;
  }
  else
    return NULL;
}

void reset_acceleration_rates();

//===========================================================================
//===================== Stepper Function Declarations =======================
//===========================================================================

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  extern bool abort_on_endstop_hit;
#endif

#if ENABLED(EMERGENCY_STOP)
  extern bool trigger_emergency_stop;
#endif

void st_init();
void st_synchronize();
void st_set_position(const long& x, const long& y, const long& z, const long& e);
void st_set_e_position(const long& e);
long st_get_position(AxisEnum axis);
float st_get_axis_position_mm(AxisEnum axis);
void st_wake_up();

void checkHitEndstops();
void endstops_hit_on_purpose();
void enable_endstops(bool check);
void enable_endstops_globally(bool check);
void endstops_not_homing();
void checkStepperErrors();
void finishAndDisableSteppers();

extern block_t* current_block;

void quickStop();

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
void microstep_mode(uint8_t driver, uint8_t stepping);
void microstep_init();
void microstep_readings();

#if ENABLED(Z_DUAL_ENDSTOPS)
  void In_Homing_Process(bool state);
  void Lock_z_motor(bool state);
  void Lock_z2_motor(bool state);
#endif

#if ENABLED(BABYSTEPPING)
  void babystep(const uint8_t axis, const bool direction);
#endif

#endif // MOTION_H
