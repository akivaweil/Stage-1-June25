#include "Config/Config.h"

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

//* ************************************************************************
//* ************************ SERVO CONFIGURATION **************************
//* ************************************************************************
// Rotation servo position settings
const int ROTATION_SERVO_HOME_POSITION = 25;     // Home position (degrees)
const int ROTATION_SERVO_ACTIVE_POSITION = 90;   // Position when activated (degrees)

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor step calculations and travel distances
const float CUT_MOTOR_STEPS_PER_INCH = 500.0;  // 4x increase from 38
const float FEED_MOTOR_STEPS_PER_INCH = 1000.0; // Steps per inch for feed motor
const float CUT_TRAVEL_DISTANCE = 9.1; // inches
const float FEED_TRAVEL_DISTANCE = 3.4; // inches (3.4 for 3 inch squares and 3.25 for 2.65 inch squares)
const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1; // Inches for incremental reverse
const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4; // Max inches for incremental reverse before error

// Motor homing direction constants
const int CUT_HOMING_DIRECTION = -1;  // Direction for cut motor homing
const int FEED_HOMING_DIRECTION = 1; // Direction for feed motor homing

//* ************************************************************************
//* ************************ CUT MOTOR SPEED SETTINGS ********************
//* ************************************************************************
// Normal Cutting Operation (Cutting State)
const float CUT_MOTOR_NORMAL_SPEED = 700;      // Speed for the cutting pass (steps/sec)
const float CUT_MOTOR_NORMAL_ACCELERATION = 17000; // Acceleration for the cutting pass (steps/sec^2)

// Return Stroke (Returning State / End of Cutting State)
const float CUT_MOTOR_RETURN_SPEED = 25000;     // Speed for returning after a cut (steps/sec)

// Homing Operation (Homing State)
const float CUT_MOTOR_HOMING_SPEED = 1500;      // Speed for homing the cut motor (steps/sec)

//* ************************************************************************
//* ************************ FEED MOTOR SPEED SETTINGS *******************
//* ************************************************************************
// Normal Feed Operation (Feed State / Parts of Cutting State)
const float FEED_MOTOR_NORMAL_SPEED = 25000;    // Speed for normal feed moves (steps/sec)
const float FEED_MOTOR_NORMAL_ACCELERATION = 25000; // Acceleration for normal feed (steps/sec^2)

// Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
const float FEED_MOTOR_RETURN_SPEED = 25000;    // Speed for returning to home or start position (steps/sec)
const float FEED_MOTOR_RETURN_ACCELERATION = 30000; // Acceleration for return moves (steps/sec^2)

// Homing Operation (Homing State)
const float FEED_MOTOR_HOMING_SPEED = 2000;     // Speed for homing the feed motor (steps/sec)

//* ************************************************************************
//* ************************ TIMING CONFIGURATION *************************
//* ************************************************************************
// Servo timing configuration
const unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2300; // Time servo stays active
const unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 1800; // Time clamp stays extended

// Cut motor homing timeout
const unsigned long CUT_HOME_TIMEOUT = 5000; // 5 seconds timeout

// Signal timing
const unsigned long TA_SIGNAL_DURATION = 500; // Duration for Transfer Arm signal (ms)

//* ************************************************************************
//* ************************ OPERATIONAL CONSTANTS ***********************
//* ************************************************************************
// Rotation clamp early activation offset
const float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 2.7; // 1.45 for 3 inch squares and 2.7 for 2.65 inch squares

// Rotation servo early activation offset
const float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = 0.4;

// Transfer Arm signal early activation offset  
const float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = 0.3;

//* ************************************************************************
//* ************************ SAFETY CONSTANTS ****************************
//* ************************************************************************
// Rotation servo safety timing
const unsigned long ROTATION_SERVO_EXTENDED_WAIT_THRESHOLD_MS = 3000; // 3 seconds - threshold for extended wait due to failure to suction
const unsigned long ROTATION_SERVO_SAFETY_DELAY_MS = 3000; // 2 seconds - additional safety delay before returning servo to home
