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
 * Configuration.h
 *
 * Basic settings such as:
 *
 * - Type of electronics
 * - Type of temperature sensor
 * - Printer geometry
 * - Endstop configuration
 * - LCD controller
 * - Extra features
 *
 * Advanced settings can be found in Configuration_adv.h
 *
 */
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "boards.h"
#include "macros.h"

//===========================================================================
//============================= Dagoma Custom Features ======================
//===========================================================================

// @section filament_change_scripts

//===========================================================================
//==================== Filament Change Scripts (M600) =======================
//===========================================================================
// G-code scripts for automatic filament insertion and extraction operations.
// These scripts are executed when filament change operations are triggered.

#define FILAMENTCHANGE_INSERTION_SCRIPT "M600 I1 U-55 X55 Y-92 W200 Z200"
#define FILAMENTCHANGE_EXTRACTION_SCRIPT "M600 I-1 U-55 X55 Y-92 W200 Z200"

// @section filament_change_motion

//===========================================================================
//==================== Filament Change Motion Settings ======================
//===========================================================================
// Motion parameters for filament change operations including Z movement,
// temperature, and extruder feedrates.

#define FILAMENTCHANGE_TEMPERATURE 200                      // (°C) Default temperature for filament change
#define FILAMENTCHANGE_Z_HOP_MM 10.0                        // (mm) Z lift during filament change
#define FILAMENTCHANGE_DELTA_Z_DOME_SECURITY_DISTANCE 25.0  // (mm) Safety distance from dome on delta printers
#define FILAMENT_CHANGE_E_FEEDRATE 66                       // (mm/s) Base extruder feedrate during filament change

// @section filament_auto_insertion

//===========================================================================
//==================== Filament Auto-Insertion Settings =====================
//===========================================================================
// Parameters controlling automatic filament insertion behavior including
// verification lengths, gaps, and feedrate factors.

// Gap and length settings (mm)
#define FILAMENT_SUCTION_GAP 200                                     // (mm) Gap for filament suction
#define FILAMENT_AUTO_INSERTION_GAP 150                              // (mm) Gap during auto insertion
#define FILAMENT_PRE_INSERTION_LENGTH 40                             // (mm) Pre-insertion extrusion length
#define FILAMENT_AUTO_INSERTION_VERIFICATION_LENGTH_MM 2.0           // (mm) Length for insertion verification
#define FILAMENTCHANGE_AUTO_INSERTION_VERIFICATION_LENGTH_MM 0.5     // (mm) Verification step length
#define FILAMENTCHANGE_AUTO_INSERTION_CONFIRMATION_LENGTH 40         // (mm) Confirmation extrusion length

// Purge settings
#define FILAMENTCHANGE_AUTO_INSERTION_PURGE_LENGTH 30                // (mm) Purge extrusion length
#define FILAMENTCHANGE_AUTO_INSERTION_PURGE_BEFORE_EXTRACTION_LENGTH 5 // (mm) Purge before extraction

// Feedrate factors (multipliers for base feedrate)
#define FILAMENT_PRE_INSERTION_FEEDRATE_FACTOR 0.1                   // Pre-insertion feedrate factor
#define FILAMENT_AUTO_INSERTION_FINAL_FEEDRATE_FACTOR 0.01           // Final insertion feedrate factor
#define FILAMENTCHANGE_AUTO_INSERTION_PREAMBLE_FEEDRATE_FACTOR 0.25  // Preamble feedrate factor
#define FILAMENTCHANGE_AUTO_INSERTION_PURGE_FEEDRATE_FACTOR 0.05     // Purge feedrate factor

// @section filament_ejection

//===========================================================================
//==================== Filament Ejection Settings ===========================
//===========================================================================
// Parameters for filament ejection sequence including retract/extrude
// movements and timing.

#define FIRST_EXTRUDE_BEFORE_EJECTION 10    // (mm) First extrusion before ejection
#define FIRST_RETRACT_BEFORE_EJECTION 4     // (mm) First retraction before ejection
#define SECOND_EXTRUDE_BEFORE_EJECTION 1.5  // (mm) Second extrusion before ejection
#define SECOND_RETRACT_BEFORE_EJECTION 50   // (mm) Second retraction before ejection
#define QUICK_PAUSE_TIMEOUT 2000            // (ms) Timeout for quick pause between ejection steps

// @section z_min_magic

//===========================================================================
//==================== Z Min Magic / Probe Settings =========================
//===========================================================================
// Z Min Magic feature for automatic bed probing and calibration.
// Note: When Z_MIN_MAGIC is enabled with LONG_PRESS_SUPPORT, the firmware
// uses long press detection instead of the tap-tap interface for user input.

#define Z_MIN_MAGIC                   // Enable Z Min Magic feature
#define Z_MAGIC_THRESHOLD -15         // (mm) Threshold value for Z magic detection

// @section user_interface_button

//===========================================================================
//==================== User Interface - Button Settings =====================
//===========================================================================
// Single button interface for printer control operations.
// Note: ONE_BUTTON shares the same pin as SUMMON_PRINT_PAUSE (pin 3).
// The button is used for both manual control and print pause triggering.

#define ONE_BUTTON                    // Enable single button interface
#define ONE_BUTTON_INVERTING true     // Invert button logic (true = active low)

// Long press support provides additional button functionality.
// When enabled with Z_MIN_MAGIC, uses long press instead of tap-tap detection.
#define LONG_PRESS_SUPPORT            // Enable long press detection for additional functions

// @section user_interface_led

//===========================================================================
//==================== User Interface - LED Settings ========================
//===========================================================================
// Status LED for visual feedback during printer operations.

#define ONE_LED                       // Enable status LED
#define ONE_LED_PIN 65                // Pin number for status LED
#define ONE_LED_INVERTING true        // Invert LED logic (true = active low)

// @section print_pause

//===========================================================================
//==================== Print Pause Settings =================================
//===========================================================================
// External trigger for pausing prints (e.g., filament runout, user button).
// Note: Uses the same pin as ONE_BUTTON (SUMMON_PRINT_PAUSE_PIN = 3).

#define SUMMON_PRINT_PAUSE                                // Enable print pause feature
#define SUMMON_PRINT_PAUSE_INVERTING true                 // Invert pause trigger logic
#define SUMMON_PRINT_PAUSE_SCRIPT "M600 U-55 X55 Y-92 Z60" // G-code script for print pause

// @section safety

//===========================================================================
//==================== Safety / Emergency Settings ==========================
//===========================================================================
// Emergency stop and safety features.

#define EMERGENCY_STOP                // Enable emergency stop feature
#define EMERGENCY_STOP_Z_MOVE         // Allow Z movement during emergency stop
#define HEATING_STOP                  // Enable automatic heating stop when idle
#define HEATING_STOP_TIME 600000UL    // (ms) Time before automatic heater shutdown (10 minutes)

// @section fan_control

//===========================================================================
//==================== Fan Control Settings =================================
//===========================================================================
// Mono fan configuration for single fan systems where the part cooling fan
// and hotend fan share the same fan unit.
//
// When IS_MONO_FAN is enabled:
// - Fan activates when hotend temperature exceeds MONO_FAN_MIN_TEMP
// - Fan speed never drops below MONO_FAN_MIN_PWM when active
// - Provides cooling whenever hotend is above threshold (heating, idle, or printing)

#define IS_MONO_FAN                   // Enable mono fan mode (single shared fan)
#define MONO_FAN_MIN_TEMP 50.0        // (°C) Minimum hotend temperature to activate fan
#define MONO_FAN_MIN_PWM 180          // Minimum PWM value when fan is active (0-255)

// @section delta_extras

//===========================================================================
//==================== Delta Printer Extras =================================
//===========================================================================
// Additional features specific to delta printer configuration.

#define DELTA_EXTRA                   // Enable delta-specific extra features

//===========================================================================
//=========================== FIRMWARE INFORMATION ==========================
//===========================================================================
// This firmware is configured for Dagoma Delta printers (Neva series).
// All non-Delta printer types have been removed.
//
// Calibration resources:
// - http://reprap.org/wiki/Calibration
// - http://reprap.org/wiki/Triffid_Hunter%27s_Calibration_Guide
// - http://calculator.josefprusa.cz

// @section info

#if ENABLED(USE_AUTOMATIC_VERSIONING)
  #include "_Version.h"
#else
  #include "Version.h"
#endif

#define STRING_CONFIG_H_AUTHOR "Dagoma"

//===========================================================================
//========================= COMMUNICATION SETTINGS ==========================
//===========================================================================
// Serial port and communication speed configuration.

// @section communication

// Serial port for host communication (0 = default Arduino serial)
#define SERIAL_PORT 0

// Communication baud rate
// :[2400,9600,19200,38400,57600,115200,250000]
#define BAUDRATE 250000

// Bluetooth serial interface (AT90USB devices only)
//#define BLUETOOTH

//===========================================================================
//========================== MOTHERBOARD SELECTION ==========================
//===========================================================================
// Select the electronics board for your printer.

// @section machine

#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_MKS_BASE
#endif

// Custom machine name displayed in LCD "Ready" message
#define CUSTOM_MACHINE_NAME "Neva"

// Unique printer identifier (used by some host software)
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

//===========================================================================
//========================== EXTRUDER CONFIGURATION =========================
//===========================================================================
// Number of extruders and hotend configuration.

// @section extruder_config

#define EXTRUDERS 1
#define HOTENDS 1

// Multi-extruder hotend offsets (uncomment for multiple extruders)
//#define HOTEND_OFFSET_X {0.0, 20.00} // (mm) X offset from extruder 0
//#define HOTEND_OFFSET_Y {0.0, 5.00}  // (mm) Y offset from extruder 0

//===========================================================================
//=========================== POWER SUPPLY SETTINGS =========================
//===========================================================================
// Power supply type selection.

// @section power

// 1 = ATX, 2 = X-Box 360 203W
#define POWER_SUPPLY 1

// Keep power supply off on startup
//#define PS_DEFAULT_OFF

//===========================================================================
//======================== TEMPERATURE SENSOR SETTINGS ======================
//===========================================================================
// Temperature sensor type selection for hotends and heated bed.
// See thermistortables.h for a complete list of supported sensors.

// @section temperature_sensors

// Common thermistor types (4.7k pullup):
// 1  = 100k EPCOS (best choice for most printers)
// 5  = 100k ATC Semitec 104GT-2 (ParCan & J-Head)
// 11 = 100k beta 3950 1%
// 16 = 100k ATC Semitec 104GT-2 Dagoma version (recommended for Dagoma printers)
// 17 = Dagoma custom thermistor

// Thermocouple types:
// -3 = MAX31855, -2 = MAX6675, -1 = AD595

// Dummy sensors (for testing only):
// 998 = Always reads 25°C, 999 = Always reads 100°C

#define TEMP_SENSOR_0 17              // Hotend 0 (Dagoma custom)
#define TEMP_SENSOR_1 0               // Hotend 1 (disabled)
#define TEMP_SENSOR_2 0               // Hotend 2 (disabled)
#define TEMP_SENSOR_3 0               // Hotend 3 (disabled)
#define TEMP_SENSOR_BED 0             // Heated bed (disabled)

// Redundant temperature sensor (use sensor 1 to verify sensor 0)
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10  // (°C) Max allowed difference

//===========================================================================
//====================== TEMPERATURE RESIDENCY SETTINGS =====================
//===========================================================================
// Wait conditions for M109/M190 temperature commands.

// @section temperature_residency

// Hotend residency
#define TEMP_RESIDENCY_TIME 10        // (s) Wait time at target temp
#define TEMP_HYSTERESIS 3             // (°C) Temperature tolerance
#define TEMP_WINDOW 3                 // (°C) Proximity window

// Bed residency
#define TEMP_BED_RESIDENCY_TIME 10    // (s) Wait time at target temp
#define TEMP_BED_HYSTERESIS 3         // (°C) Temperature tolerance
#define TEMP_BED_WINDOW 3             // (°C) Proximity window

//===========================================================================
//======================= TEMPERATURE SAFETY LIMITS =========================
//===========================================================================
// Min/max temperature limits for safety protection.
// MINTEMP protects against disconnected thermistors.
// MAXTEMP protects against overheating.

// @section temperature_limits

// Minimum temperatures (below this = thermistor error)
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define BED_MINTEMP 5

// Maximum temperatures (above this = heater shutdown)
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define BED_MAXTEMP 150

// Report heater power in watts via M105
//#define EXTRUDER_WATTS (12.0*12.0/6.7) // P=U^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=U^2/R

//===========================================================================
//=========================== PID CONTROL SETTINGS ==========================
//===========================================================================
// PID temperature control for hotends.
// Guide: http://reprap.org/wiki/PID_Tuning

// @section pid

#define PIDTEMP                       // Enable PID (comment out for bang-bang)
#define BANG_MAX 255                  // Max power in bang-bang mode (0-255)
#define PID_MAX BANG_MAX              // Max power in PID mode (0-255)
#if ENABLED(PIDTEMP)
  #define PID_FUNCTIONAL_RANGE 10       // (°C) PID active within this temp range
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX
  #define K1 0.95                         // Smoothing factor

  // PID values for Dagoma hotend (E3D-v6 style, 1.75mm)
  #define DEFAULT_Kp 32.48
  #define DEFAULT_Ki 6.40
  #define DEFAULT_Kd 41.25

#endif // PIDTEMP

//===========================================================================
//========================= BED PID CONTROL SETTINGS ========================
//===========================================================================
// PID temperature control for heated bed (optional).

// @section pid_bed

//#define PIDTEMPBED                    // Enable bed PID (comment out for bang-bang)
#define MAX_BED_POWER 255             // Max bed power (0-255)

#if ENABLED(PIDTEMPBED)
  #define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER

  // Default bed PID values (tune with M303 E-1 C8 S90)
  #define DEFAULT_bedKp 10.00
  #define DEFAULT_bedKi 0.023
  #define DEFAULT_bedKd 305.40
#endif // PIDTEMPBED

//===========================================================================
//========================= EXTRUDER SAFETY SETTINGS ========================
//===========================================================================
// Prevent cold extrusion and excessive extrusion length.

// @section extruder_safety

#define PREVENT_DANGEROUS_EXTRUDE     // Require minimum temperature before extruding
//#define PREVENT_LENGTHY_EXTRUDE     // Limit single extrusion length

#define EXTRUDE_MINTEMP 170           // (°C) Minimum temp to allow extrusion
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) // (mm) Max extrusion length

//===========================================================================
//======================= THERMAL RUNAWAY PROTECTION ========================
//===========================================================================
// Protects against fire hazard from thermistor failure or disconnection.
// Tune parameters in Configuration_adv.h if you get false "Thermal Runaway" errors.

// @section thermal_protection

#define THERMAL_PROTECTION_HOTENDS    // Enable for all hotends
#define THERMAL_PROTECTION_BED        // Enable for heated bed

//===========================================================================
//========================= DELTA PRINTER GEOMETRY ==========================
//===========================================================================
// Delta kinematics configuration for Dagoma Neva printer.
// All DELTA_* values must be floating point numbers.

// @section delta_geometry

#define DELTA                         // Enable Delta kinematics

#if ENABLED(DELTA)

  // Motion interpolation - higher values = smoother curves but more CPU load
  // For delta printers, 200 provides good smoothness at typical speeds
  #define DELTA_SEGMENTS_PER_SECOND 200

  // Delta arm geometry (mm)
  #define DELTA_DIAGONAL_ROD 210            // Diagonal rod length (POM + PTFE)
  #define DELTA_SMOOTH_ROD_OFFSET 147.10    // Tower to center distance
  #define DELTA_EFFECTOR_OFFSET 32.11       // Effector joint offset
  #define DELTA_CARRIAGE_OFFSET 21.12       // Carriage joint offset

  // Calculated delta radius
  #define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET - DELTA_EFFECTOR_OFFSET - DELTA_CARRIAGE_OFFSET)

  // Print area radius (avoid tower collisions)
  #define DELTA_PRINTABLE_RADIUS 90.0

#endif

// Enable this option for Toshiba steppers
//#define CONFIG_STEPPERS_TOSHIBA

//===========================================================================
//============================== Endstop Settings ===========================
//===========================================================================

// @section homing

// Specify here all the endstop connectors that are connected to any endstop or probe.
// Almost all printers will be using one per axis. Probes will use one or more of the
// extra connectors. Leave undefined any used for non-endstop and non-probe purposes.
//#define USE_XMIN_PLUG
//#define USE_YMIN_PLUG
//#define USE_ZMIN_PLUG
#define USE_XMAX_PLUG
#define USE_YMAX_PLUG
#define USE_ZMAX_PLUG

// Pull-up resistors for endstops
#define ENDSTOPPULLUPS                // Enable all pullups

#if DISABLED(ENDSTOPPULLUPS)
  // Individual pullup control (ignored if ENDSTOPPULLUPS is enabled)
  //#define ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX
  //#define ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
  //#define ENDSTOPPULLUP_ZMIN_PROBE
#endif

// Endstop signal inversion (true = normally-open, false = normally-closed)
const bool X_MIN_ENDSTOP_INVERTING = true;
const bool Y_MIN_ENDSTOP_INVERTING = true;
const bool Z_MIN_ENDSTOP_INVERTING = true;
const bool X_MAX_ENDSTOP_INVERTING = true;
const bool Y_MAX_ENDSTOP_INVERTING = true;
const bool Z_MAX_ENDSTOP_INVERTING = true;
const bool Z_MIN_PROBE_ENDSTOP_INVERTING = true;

//===========================================================================
//=========================== Z PROBE CONFIGURATION =========================
//===========================================================================
// Z probe settings for auto bed leveling.

// @section z_probe

// Use separate Z probe pin (Z_MIN_PROBE_PIN) for probing only
#define Z_MIN_PROBE_ENDSTOP

// Use Z_MIN_PIN for both homing and probing
//#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN

// Disable the Z probe pin (if using separate Z min endstop)
//#define DISABLE_Z_MIN_PROBE_ENDSTOP

//===========================================================================
//========================== STEPPER DRIVER SETTINGS ========================
//===========================================================================
// Stepper motor enable pins and behavior.

// @section stepper_enable

// Enable pin polarity: 0 = Active Low, 1 = Active High
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

// Disable steppers when idle (may lose position accuracy)
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false
#define DISABLE_INACTIVE_EXTRUDER true

//===========================================================================
//======================== STEPPER DIRECTION SETTINGS =======================
//===========================================================================
// Invert stepper direction if axis moves the wrong way.

// @section stepper_direction

// Axis directions
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

// Extruder directions (true = direct drive, false = geared)
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false

//===========================================================================
//============================= HOMING SETTINGS =============================
//===========================================================================
// Homing direction and position configuration.

// @section homing

// Minimum Z height before homing (for clearance)
//#define MIN_Z_HEIGHT_FOR_HOMING 4   // (mm)
                                    // Be sure you have this distance over your Z_MAX_POS in case.

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
// :[-1,1]
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// X-3.48 Y-6.76 Z-8.46

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// @section machine

// Travel limits after homing (mm)
#define X_MIN_POS -(DELTA_PRINTABLE_RADIUS)
#define Y_MIN_POS -(DELTA_PRINTABLE_RADIUS)
#define Z_MIN_POS 0
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS

//===========================================================================
//======================== FILAMENT RUNOUT SENSOR ===========================
//===========================================================================
// Detects when filament runs out or breaks during printing.
// Triggers FILAMENT_RUNOUT_SCRIPT (usually M600 for filament change).

// @section filament_runout

#define FILAMENT_RUNOUT_SENSOR        // Enable filament runout detection

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  const bool FIL_RUNOUT_INVERTING = true;
  #define ENDSTOPPULLUP_FIL_RUNOUT    // Use internal pullup
  #define FILAMENT_RUNOUT_SCRIPT "M600 U-55 X55 Y-92 Z60"
#endif

//===========================================================================
//========================= AUTO BED LEVELING ===============================
//===========================================================================
// Automatic bed leveling using Z probe.
// Delta printers require grid-based leveling.

// @section bedlevel

#define AUTO_BED_LEVELING_FEATURE     // Enable auto bed leveling
//#define DEBUG_LEVELING_FEATURE      // Enable leveling debug output
//#define Z_MIN_PROBE_REPEATABILITY_TEST

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  // Delta printers REQUIRE grid-based leveling
  #define AUTO_BED_LEVELING_GRID

  // Probing area (circular for delta)
  #define DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS - 40.0)
  #define LEFT_PROBE_BED_POSITION -(DELTA_PROBEABLE_RADIUS)
  #define RIGHT_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS
  #define FRONT_PROBE_BED_POSITION -(DELTA_PROBEABLE_RADIUS)
  #define BACK_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS
  #define MIN_PROBE_EDGE 10           // Minimum edge clearance

  // Grid density (5+ recommended for delta bowl/dome compensation)
  #define AUTO_BED_LEVELING_GRID_POINTS 5

  // Probe offset from nozzle (mm)
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0.0   // X: -left +right
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0.0   // Y: -front +behind
  #define Z_PROBE_OFFSET_FROM_EXTRUDER 0.0   // Z: -below +above

  // Probing speeds and heights
  #define XY_TRAVEL_SPEED 4000        // (mm/min) XY travel speed between probes
  #define Z_RAISE_BEFORE_PROBING 15   // (mm) Z lift before first probe
  #define Z_RAISE_BETWEEN_PROBINGS 5  // (mm) Z lift between probes
  #define Z_RAISE_AFTER_PROBING 15    // (mm) Z lift after last probe

  // End script (for retractable probes)
  //#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10"

  // Probe type selection
  //#define FIX_MOUNTED_PROBE         // Fixed inductive/capacitive probe

  // A Servo Probe can be defined in the servo section below.

  // An Allen Key Probe is currently predefined only in the delta example configurations.

  // Enable if you have a Z probe mounted on a sled like those designed by Charles Bell.
  //#define Z_PROBE_SLED
  //#define SLED_DOCKING_OFFSET 5 // The extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.

  // A Mechanical Probe is any probe that either doesn't deploy or needs manual deployment
  // For example any setup that uses the nozzle itself as a probe.
  #define MECHANICAL_PROBE

  // If you've enabled AUTO_BED_LEVELING_FEATURE and are using the Z Probe for Z Homing,
  // it is highly recommended you also enable Z_SAFE_HOMING below!

#endif // AUTO_BED_LEVELING_FEATURE


// @section homing

// The position of the homing switches
#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

// Manual homing switch locations:
// For deltabots this means top and center of the Cartesian print volume.
#if ENABLED(MANUAL_HOME_POSITIONS)
  #define MANUAL_X_HOME_POS 0
  #define MANUAL_Y_HOME_POS 0
  //#define MANUAL_Z_HOME_POS 0
  //#define MANUAL_Z_HOME_POS 205 // For delta: Distance between nozzle and print surface after homing.
  #define MANUAL_Z_HOME_POS 220 // 2017/03/10 For delta: Distance between nozzle and print surface after homing.
#endif

// Use "Z Safe Homing" to avoid homing with a Z probe outside the bed area.
//
// With this feature enabled:
//
// - Allow Z homing only after X and Y homing AND stepper drivers still enabled.
// - If stepper drivers time out, it will need X and Y homing again before Z homing.
// - Position the Z probe in a defined XY point before Z Homing when homing all axes (G28).
// - Prevent Z homing when the Z probe is outside bed area.
//#define Z_SAFE_HOMING

#if ENABLED(Z_SAFE_HOMING)
  #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)    // X point for Z homing when homing all axis (G28).
  #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)    // Y point for Z homing when homing all axis (G28).
#endif


// @section movement

/**
 * MOVEMENT SETTINGS
 */

#define HOMING_FEEDRATE_XYZ (50*60)
#define HOMING_FEEDRATE_E 0
#define HOMING_FEEDRATE { HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_E }
//#define HOMING_FEEDRATE {50*60, 50*60, 50*60, 0}  // set the homing speeds (mm/min)

// default settings

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0, 80.0, 80.0, 98.0}  // default steps per unit for Ultimaker
//#define DEFAULT_MAX_FEEDRATE          {300, 300, 300, 25}    // (mm/sec)
//#define DEFAULT_MAX_FEEDRATE          {200, 200, 200, 25}    // (mm/sec)
#define DEFAULT_MAX_FEEDRATE          {333, 333, 333, 170}    // (mm/s)
#define DEFAULT_MAX_ACCELERATION      {1000,1000,1000,10000}  // (mm/s²)

// Default acceleration values
#define DEFAULT_ACCELERATION 3000           // (mm/s²) Normal moves
#define DEFAULT_RETRACT_ACCELERATION 3000   // (mm/s²) Retraction moves
#define DEFAULT_TRAVEL_ACCELERATION 3000    // (mm/s²) Travel moves

// Jerk settings (instantaneous speed change without acceleration)
// Note: For delta, XY and Z jerk should be the same
#define DEFAULT_XYJERK 20.0           // (mm/s)
#define DEFAULT_ZJERK 20.0            // (mm/s)
#define DEFAULT_EJERK 20.0            // (mm/s)

//===========================================================================
//========================== CUSTOM M-CODE SETTINGS =========================
//===========================================================================
// Custom G-code command configurations.

// @section custom_codes

#define CUSTOM_M_CODES
#if ENABLED(CUSTOM_M_CODES)
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
    #define Z_PROBE_OFFSET_RANGE_MIN -20
    #define Z_PROBE_OFFSET_RANGE_MAX 20
  #endif
#endif

//===========================================================================
//============================= EEPROM SETTINGS =============================
//===========================================================================
// Persistent storage for printer settings.
// M500 = Save, M501 = Load, M502 = Reset to defaults

// @section eeprom

#define EEPROM_SETTINGS               // Enable EEPROM storage

#if ENABLED(EEPROM_SETTINGS)
  #define EEPROM_CHITCHAT             // Enable serial feedback for EEPROM operations
#endif

//===========================================================================
//========================== HOST COMMUNICATION =============================
//===========================================================================
// Communication settings with host software.

// @section host

//#define DISABLE_HOST_KEEPALIVE      // Disable busy messages to host
#if DISABLED(DISABLE_HOST_KEEPALIVE)
  #define DEFAULT_KEEPALIVE_INTERVAL 2  // (s) Interval between busy messages
#endif

// Debug: Free memory watcher
//#define M100_FREE_MEMORY_WATCHER

//===========================================================================
//=========================== LANGUAGE SETTINGS =============================
//===========================================================================
// Display language configuration.

// @section language

#define LANGUAGE_INCLUDE GENERATE_LANGUAGE_INCLUDE(en)

//===========================================================================
//============================= SD CARD SUPPORT =============================
//===========================================================================
// SD card file storage configuration.

// @section sdcard

#define SDSUPPORT                     // Enable SD card support

// SD Card SPI speed (uncomment to slow down if getting init errors)
//#define SPI_SPEED SPI_HALF_SPEED
//#define SPI_SPEED SPI_QUARTER_SPEED
//#define SPI_SPEED SPI_EIGHTH_SPEED

// SD Card CRC checking
//#define SD_CHECK_AND_RETRY          // Enable CRC checks and retries

//===========================================================================
//============================== FAN PWM SETTINGS ===========================
//===========================================================================
// Fan control method configuration.

// @section fan_pwm

//#define FAST_PWM_FAN                // Hardware PWM (reduces noise but increases FET heating)
#define FAN_SOFT_PWM                  // Software PWM (quieter, lower frequency)
#define SOFT_PWM_SCALE 0              // PWM frequency scale (0 = 128 positions)

//===========================================================================
//========================== MISCELLANEOUS FEATURES =========================
//===========================================================================
// Optional extra features.

// @section extras

// Temperature status LEDs (Blue = cold, Red = hot)
//#define TEMP_STAT_LEDS

// Camera trigger (M240) - Canon RC-1 emulation
//#define PHOTOGRAPH_PIN 23

// Arc fix for SkeinForge compatibility
//#define SF_ARC_FIX

// BariCUDA paste extruder support
//#define BARICUDA

// BlinkM/CyzRgb LED support
//#define BLINKM

//===========================================================================
//========================= FILAMENT DIAMETER ===============================
//===========================================================================
// Default filament diameter for volumetric calculations.
// Dagoma Neva uses 1.75mm filament.

// @section filament

#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75  // (mm) Dagoma uses 1.75mm

//===========================================================================
//============================== INCLUDES ===================================
//===========================================================================

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //CONFIGURATION_H
