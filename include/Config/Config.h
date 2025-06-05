#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

//* ************************************************************************
//* ************************ SERVO CONFIGURATION **************************
//* ************************************************************************
// Rotation servo position settings
extern const int ROTATION_SERVO_HOME_POSITION;     // Home position (degrees)
extern const int ROTATION_SERVO_ACTIVE_POSITION;   // Position when activated (degrees)

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor step calculations and travel distances
extern const float CUT_MOTOR_STEPS_PER_INCH;  // 4x increase from 38
extern const float FEED_MOTOR_STEPS_PER_INCH; // Steps per inch for feed motor
extern const float CUT_TRAVEL_DISTANCE; // inches
extern const float FEED_TRAVEL_DISTANCE; // inches
extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES; // Inches for incremental reverse
extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES; // Max inches for incremental reverse before error

// Motor homing direction constants
extern const int CUT_HOMING_DIRECTION;
extern const int FEED_HOMING_DIRECTION;

//* ************************************************************************
//* ************************ CUT MOTOR SPEED SETTINGS ********************
//* ************************************************************************
// Normal Cutting Operation (Cutting State)
extern const float CUT_MOTOR_NORMAL_SPEED;      // Speed for the cutting pass (steps/sec)
extern const float CUT_MOTOR_NORMAL_ACCELERATION; // Acceleration for the cutting pass (steps/sec^2)

// Return Stroke (Returning State / End of Cutting State)
extern const float CUT_MOTOR_RETURN_SPEED;     // Speed for returning after a cut (steps/sec)

// Homing Operation (Homing State)
extern const float CUT_MOTOR_HOMING_SPEED;      // Speed for homing the cut motor (steps/sec)

//* ************************************************************************
//* ************************ FEED MOTOR SPEED SETTINGS *******************
//* ************************************************************************
// Normal Feed Operation (Feed State / Parts of Cutting State)
extern const float FEED_MOTOR_NORMAL_SPEED;    // Speed for normal feed moves (steps/sec)
extern const float FEED_MOTOR_NORMAL_ACCELERATION; // Acceleration for normal feed (steps/sec^2)

// Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
extern const float FEED_MOTOR_RETURN_SPEED;    // Speed for returning to home or start position (steps/sec)
extern const float FEED_MOTOR_RETURN_ACCELERATION; // Acceleration for return moves (steps/sec^2)

// Homing Operation (Homing State)
extern const float FEED_MOTOR_HOMING_SPEED;     // Speed for homing the feed motor (steps/sec)

//* ************************************************************************
//* ************************ TIMING CONFIGURATION *************************
//* ************************************************************************
// Servo timing configuration
extern const unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS; // Time servo stays active
extern const unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS; // Time clamp stays extended

// Cut motor homing timeout
extern const unsigned long CUT_HOME_TIMEOUT; // 5 seconds timeout

// Signal timing
extern const unsigned long TA_SIGNAL_DURATION; // Duration for Transfer Arm signal (ms)

//* ************************************************************************
//* ************************ OPERATIONAL CONSTANTS ***********************
//* ************************************************************************
// Rotation clamp early activation offset
extern const float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;

// Rotation servo early activation offset
extern const float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;

#endif // SYSTEM_CONFIG_H 