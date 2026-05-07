#include "Config/MotorConfig.h"

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ MOTOR CONFIGURATION                                       ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ ROTATION SERVO                                            ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
// (Servo parameters moved to config.cpp)

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 ROTATION CLAMP                                            ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
// (Rotation clamp parameters moved to config.cpp)

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ 🔄 CUT MOTOR                                                 ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
float CUT_MOTOR_STEPS_PER_INCH = 500.0;
const int CUT_HOMING_DIRECTION = -1;

// Speeds (in inches/sec and inches/sec²)
float CUT_MOTOR_NORMAL_SPEED = 1.28; // inches/sec
float CUT_MOTOR_NORMAL_ACCELERATION = 16.0; // inches/sec²
// No-wood speed = 60% of the normal running speed set in the dashboard.
// If you change the dashboard cut speed, update this to: (new_speed * 0.60).
// Also update ROTATION_CLAMP_NO2X4_EXTRA_DELAY_MS in 05_NOWOOD.cpp:
//   extra_delay_ms = round((CLAMP_TO_SERVO_DISTANCE / NO_WOOD_SPEED - CLAMP_TO_SERVO_DISTANCE / NORMAL_SPEED) * 1000)
//   where CLAMP_TO_SERVO_DISTANCE = servo_activation_in - clamp_activation_in  (currently 7.9 - 7.3 = 0.6 in)
//   e.g. at 1.38 in/s normal: (0.6/0.828 - 0.6/1.38) * 1000 = 725 - 435 = 290 ms
float CUT_MOTOR_NO_WOOD_SPEED = 0.828; // inches/sec (60% of 1.38 in/s)
float CUT_MOTOR_RETURN_SPEED = 30.0; // inches/sec
float CUT_MOTOR_HOMING_SPEED = 2.6; // inches/sec

// Distances & Movement
float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1;
float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4;

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ ⚡ FEED MOTOR                                                 ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
float FEED_MOTOR_STEPS_PER_INCH = 1000.0; 
const int FEED_HOMING_DIRECTION = 1;
float FEED_MOTOR_OFFSET_FROM_SENSOR = 0.15;

// Speeds
float FEED_MOTOR_NORMAL_SPEED = 22000;
float FEED_MOTOR_NORMAL_ACCELERATION = 22000;
float FEED_MOTOR_RETURN_SPEED = 40000;
float FEED_MOTOR_RETURN_ACCELERATION = 30000;
float FEED_MOTOR_HOMING_SPEED = 1500;

// Distances & Movement
float FEED_MOTOR_RETURN_DISTANCE = 0.0;

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ 🤖 TRANSFER ARM                                              ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
// (Transfer arm parameters moved to config.cpp)

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ ⏱️ GENERAL TIMING                                            ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
// (General timing parameters moved to config.cpp)

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ 📐 PRE-CALCULATED STEPS                                      ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
