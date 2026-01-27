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
 * Configuration_adv.h
 *
 * Advanced settings.
 * Only change these if you know exactly what you're doing.
 * Some of these settings can damage your printer if improperly set!
 *
 * Basic settings can be found in Configuration.h
 *
 */
#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

#include "Conditionals.h"

// @section temperature

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

#if DISABLED(PIDTEMPBED)
  #define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
  #if ENABLED(BED_LIMIT_SWITCHING)
    #define BED_HYSTERESIS 2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS
  #endif
#endif

/**
 * Thermal Protection protects your printer from damage and fire if a
 * thermistor falls out or temperature sensors fail in any way.
 *
 * The issue: If a thermistor falls out or a temperature sensor fails,
 * Marlin can no longer sense the actual temperature. Since a disconnected
 * thermistor reads as a low temperature, the firmware will keep the heater on.
 *
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too long (period),
 * the firmware will halt the machine as a safety precaution.
 *
 * If you get false positives for "Thermal Runaway" increase THERMAL_PROTECTION_HYSTERESIS and/or THERMAL_PROTECTION_PERIOD
 */
#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  #define THERMAL_PROTECTION_PERIOD 40        // Seconds
  #define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

  /**
   * Whenever an M104 or M109 increases the target temperature the firmware will wait for the
   * WATCH_TEMP_PERIOD to expire, and if the temperature hasn't increased by WATCH_TEMP_INCREASE
   * degrees, the machine is halted, requiring a hard reset. This test restarts with any M104/M109,
   * but only if the current temperature is far enough below the target for a reliable test.
   *
   * If you get false positives for "Heating failed" increase WATCH_TEMP_PERIOD and/or decrease WATCH_TEMP_INCREASE
   * WATCH_TEMP_INCREASE should not be below 2.
   */
  #define WATCH_TEMP_PERIOD 20                // Seconds
  #define WATCH_TEMP_INCREASE 2               // Degrees Celsius
#endif

/**
 * Thermal Protection parameters for the bed
 * are like the above for the hotends.
 * WATCH_TEMP_BED_PERIOD and WATCH_TEMP_BED_INCREASE are not imlemented now.
 */
#if ENABLED(THERMAL_PROTECTION_BED)
  #define THERMAL_PROTECTION_BED_PERIOD 20    // Seconds
  #define THERMAL_PROTECTION_BED_HYSTERESIS 2 // Degrees Celsius
#endif

#if ENABLED(PIDTEMP)
  // this adds an experimental additional term to the heating power, proportional to the extrusion speed.
  // if Kc is chosen well, the additional required power due to increased melting should be compensated.
  #define PID_ADD_EXTRUSION_RATE
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    #define DEFAULT_Kc (100) //heating power=Kc*(e_speed)
    #define LPQ_MAX_LEN 50
  #endif
#endif

/**
 * Automatic Temperature:
 * The hotend target temperature is calculated by all the buffered lines of gcode.
 * The maximum buffered steps/sec of the extruder motor is called "se".
 * Start autotemp mode with M109 S<mintemp> B<maxtemp> F<factor>
 * The target temperature is set to mintemp+factor*se[steps/sec] and is limited by
 * mintemp and maxtemp. Turn this off by executing M109 without F*
 * Also, if the temperature is set to a value below mintemp, it will not be changed by autotemp.
 * On an Ultimaker, some initial testing worked with M109 S215 B260 F1 in the start.gcode
 */
#define AUTOTEMP
#if ENABLED(AUTOTEMP)
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

//Show Temperature ADC value
//The M105 command return, besides traditional information, the ADC value read from temperature sensors.
//#define SHOW_TEMP_ADC_VALUES

// @section extruder

//  extruder run-out prevention.
//if the machine is idle, and the temperature over MINTEMP, every couple of SECONDS some filament is extruded
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS 30.
#define EXTRUDER_RUNOUT_ESTEPS 14. //mm filament
#define EXTRUDER_RUNOUT_SPEED 1500.  //extrusion speed
#define EXTRUDER_RUNOUT_EXTRUDE 100

// @section temperature

//These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled
//and turn off after the set amount of seconds from last driver being disabled again
#define CONTROLLERFAN_PIN -1 //Pin used for the fan to cool controller (-1 to disable)
#define CONTROLLERFAN_SECS 60 //How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED 255  // == full speed

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)
//#define FAN_KICKSTART_TIME 100

// This defines the minimal speed for the main fan, run in PWM mode
// to enable uncomment and set minimal PWM speed for reliable running (1-255)
// if fan speed is [1 - (FAN_MIN_PWM-1)] it is set to FAN_MIN_PWM
//#define FAN_MIN_PWM 50

// @section extruder

// Extruder cooling fans
// Configure fan pin outputs to automatically turn on/off when the associated
// extruder temperature is above/below EXTRUDER_AUTO_FAN_TEMPERATURE.
// Multiple extruders can be assigned to the same pin in which case
// the fan will turn on when any selected extruder is above the threshold.
#define EXTRUDER_0_AUTO_FAN_PIN -1
#define EXTRUDER_1_AUTO_FAN_PIN -1
#define EXTRUDER_2_AUTO_FAN_PIN -1
#define EXTRUDER_3_AUTO_FAN_PIN -1
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED 255


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// @section homing

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// @section extras

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.

// NOTE: Z_DUAL_STEPPER_DRIVERS, Y_DUAL_STEPPER_DRIVERS, and DUAL_X_CARRIAGE
// have been removed. This firmware is Delta-only.

// @section homing

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {2, 2, 4}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
//#define QUICK_HOME  //if this is defined, if both x and y are to be homed, a diagonal move will be performed initially.

// When G28 is called, this option will make Y home before X
//#define HOME_Y_BEFORE_X

// @section machine

#define AXIS_RELATIVE_MODES {false, false, false, false}

// @section machine

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// Default stepper release if idle. Set to 0 to deactivate.
// Steppers will shut down DEFAULT_STEPPER_DEACTIVE_TIME seconds after the last move when DISABLE_INACTIVE_? is true.
// Time can be set by M18 and M84.
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X true
#define DISABLE_INACTIVE_Y true
#define DISABLE_INACTIVE_Z true
#define DISABLE_INACTIVE_E true

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// @section extras

// Minimum time in microseconds that a movement needs to take if the buffer is emptied.
// Lower values allow faster segment rates for smoother delta motion.
// 10000 (10ms) allows 100Hz segment rate, good for deltas.
#define DEFAULT_MINSEGMENTTIME        10000

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#define CHDK_DELAY 50 //How long in ms the pin should stay HIGH before going LOW again

// @section sdcard

#if ENABLED(SDSUPPORT)

  // SD card detect pin - disable unless using push button
  #define SD_DETECT_INVERTED

  #define SD_FINISHED_STEPPERRELEASE false  //if sd support and the file is finished: disable steppers?
  #define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.

  #define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display

#endif // SDSUPPORT

// @section more

// The hardware watchdog should reset the microcontroller disabling all outputs, in case the firmware gets stuck and doesn't do temperature regulation.
#define USE_WATCHDOG

// @section lcd

// Babystepping enables the user to control the axis in tiny amounts, independently from the normal printing process
// it can e.g. be used to change z-positions in the print startup phase in real-time
// does not respect endstops!
//#define BABYSTEPPING
#if ENABLED(BABYSTEPPING)
  #define BABYSTEP_XY  //not only z, but also XY in the menu. more clutter, more functions
                       //not implemented for deltabots!
  #define BABYSTEP_INVERT_Z false  //true for inverse movements in Z
  #define BABYSTEP_MULTIPLICATOR 1 //faster movements
#endif

// @section extruder

// extruder advance constant (s2/mm3)
//
// advance (steps) = STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K * cubic mm per second ^ 2
//
// Hooke's law says:    force = k * distance
// Bernoulli's principle says:  v ^ 2 / 2 + g . h + pressure / density = constant
// so: v ^ 2 is proportional to number of steps we advance the extruder
//#define ADVANCE

#if ENABLED(ADVANCE)
  #define EXTRUDER_ADVANCE_K .0
  #define D_FILAMENT 2.85
#endif

// @section extras

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

#define MIN_STEPS_PER_SEGMENT 0 //everything with less than this number of steps will be ignored as move and joined with the next movement

// @section temperature

// Control heater 0 and heater 1 in parallel.
//#define HEATERS_PARALLEL

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section hidden

// The number of linear motions that can be in the plan at any give time.
// For delta printers with high segment rates, a larger buffer prevents stuttering.
#if ENABLED(SDSUPPORT)
  #define BLOCK_BUFFER_SIZE 32   // Larger buffer for smooth delta motion
#else
  #define BLOCK_BUFFER_SIZE 32   // Larger buffer for smooth delta motion
#endif

// @section more

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 10

// Bad Serial-connections can miss a received command by sending an 'ok'
// Therefore some clients abort after 30 seconds in a timeout.
// Some other clients start sending commands while receiving a 'wait'.
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.
//#define NO_TIMEOUTS 1000 // Milliseconds

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.
//#define ADVANCED_OK

// @section fwretract

// Firmware based and LCD controlled retract
// M207 and M208 can be used to define parameters for the retraction.
// The retraction can be called by the slicer using G10 and G11
// until then, intended retractions can be detected by moves that only extrude and the direction.
// the moves are than replaced by the firmware controlled ones.

//#define FWRETRACT  //ONLY PARTIALLY TESTED
#if ENABLED(FWRETRACT)
  #define MIN_RETRACT 0.1                //minimum extruded mm to accept a automatic gcode retraction attempt
  #define RETRACT_LENGTH 3               //default retract length (positive mm)
  #define RETRACT_LENGTH_SWAP 13         //default swap retract length (positive mm), for extruder change
  #define RETRACT_FEEDRATE 45            //default feedrate for retracting (mm/s)
  #define RETRACT_ZLIFT 0                //default retract Z-lift
  #define RETRACT_RECOVER_LENGTH 0       //default additional recover length (mm, added to retract length when recovering)
  #define RETRACT_RECOVER_LENGTH_SWAP 0  //default additional swap recover length (mm, added to retract length when recovering from extruder change)
  #define RETRACT_RECOVER_FEEDRATE 8     //default feedrate for recovering from retraction (mm/s)
#endif

// Add support for experimental filament exchange support M600
#define FILAMENTCHANGEENABLE
#if ENABLED(FILAMENTCHANGEENABLE)
  #define FILAMENTCHANGE_XPOS 0
  #define FILAMENTCHANGE_YPOS 0
  #define FILAMENTCHANGE_ZADD 60
  #define FILAMENTCHANGE_FIRSTRETRACT -4.5
  #define FILAMENTCHANGE_FINALRETRACT -850
  #define AUTO_FILAMENT_CHANGE                //This extrude filament until you press the button
  #define AUTO_FILAMENT_CHANGE_LENGTH 0.04    //Extrusion length on automatic extrusion loop
  #define AUTO_FILAMENT_CHANGE_FEEDRATE 300   //Extrusion feedrate (mm/min) on automatic extrusion loop
#endif

#include "Conditionals.h"
#include "SanityCheck.h"

#endif //CONFIGURATION_ADV_H
