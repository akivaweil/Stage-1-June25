#include "StateMachine/STATES/States_Config.h"
#include "Config/Motor_Config.h"

//═══════════════════════════════════════════════════════════════════════════
//║ ═══════════════════════════════════════════════════════════════════════ ║
//║          📋  SYSTEM CONFIGURATION FILE - STAGE 1 JUNE 25                ║
//║ ═══════════════════════════════════════════════════════════════════════ ║
//═══════════════════════════════════════════════════════════════════════════

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 💾 EEPROM STORAGE CONFIGURATION                                      ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
const int CONFIG_EEPROM_SIZE = 2048;  // EEPROM size for configuration storage
const int CONFIG_OFFSET = 0;          // Configuration starts at beginning of extended EEPROM

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⏱️ TIMING & DELAY CONSTANTS                                          ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long SENSOR_STABILIZATION_DELAY_MS = 30;     // Sensor reading stabilization delay
const unsigned long CLAMP_FEED_MOTOR_DELAY_MS = 100;   // Delay between clamp and feed motor movement
const unsigned long FEED_CLAMP_DELAY_MS = 300;        // Delay after extending feed clamp
const unsigned long CYLINDER_ACTION_DELAY_MS = 150;    // Delay for cylinder actions
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 30; // Sensor verification stabilization delay

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ ROTATION SERVO SETTINGS                                           ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
int ROTATION_SERVO_HOME_POSITION = 27;                           // Servo home position angle
int ROTATION_SERVO_ACTIVE_POSITION = 120;                        // Servo active rotation angle
unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2000;     // Duration to hold active position
unsigned long ROTATION_SERVO_RETURN_DELAY_MS = 150;              // Delay before returning to home
unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS = 500;        // Wait duration at home position
float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = 0.2;       // Early activation offset distance

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 ROTATION CLAMP CONFIGURATION                                      ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 2200;          // Time for clamp to fully extend
float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 2.7;       // Early activation offset distance

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔄 CUT MOTOR TIMING SETTINGS                                         ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long CUT_HOME_TIMEOUT = 5000;                           // Cut motor homing timeout
unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000;              // Recovery operation timeout
unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20;             // Motor state verification delay

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🤖 TRANSFER ARM CONFIGURATION                                        ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long TA_SIGNAL_DURATION = 1000;                         // Transfer arm signal duration
float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = 0.01;          // Early activation offset distance

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚠️ ERROR RECOVERY CONSTANTS                                          ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
const unsigned long CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS = 5000;   // Maximum recovery time (5 seconds)
const float CUT_MOTOR_HOME_RECOVERY_SPEED = 1000;                // Recovery speed (same as homing speed)
const float DECELERATION_DISTANCE_INCHES = 0.2;                  // Maximum deceleration distance

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🎯 STATE OPERATION CONFIGURATION                                     ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
const float FEED_MOTOR_SPEED_MULTIPLIER = 0.6;                   // Speed reduction for NO_2x4 returning sequence
const float FEED_MOTOR_2ND_POSITION = -1.2;                      // Position for 2nd position movement
const float FEED_MOTOR_HOME_POSITION = 1.0;                      // Home position
const float FEED_MOTOR_FINAL_POSITION = -1.2;                    // Final position

const float FEED_MOTOR_FIRST_RUN_START_POSITION = -1.2;          // First run start position (inches)
const float FEED_MOTOR_FIRST_RUN_END_POSITION = 3.4;             // First run end position (inches)
const float FEED_MOTOR_SECOND_RUN_START_POSITION = -1.2;         // Second run start position (inches)
const float FEED_MOTOR_SECOND_RUN_END_POSITION = 2.1;            // Second run end position (inches)

const unsigned long FEED_HOME_TIMEOUT = 30000;                   // Feed motor homing timeout (30 seconds)
