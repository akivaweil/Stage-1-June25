#include "StateMachine/STATES/States_Config.h"

//* ************************************************************************
//* ************************ STATES CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

//* ************************************************************************
//* ************************ SERVO CONFIGURATION **************************
//* ************************************************************************
// Rotation servo position settings
int ROTATION_SERVO_HOME_POSITION = 12;  // Now adjustable via dashboard
int ROTATION_SERVO_ACTIVE_POSITION = 105;  // Now adjustable via dashboard

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor step calculations and travel distances
float CUT_MOTOR_STEPS_PER_INCH = 500.0;  // Now adjustable via dashboard
float FEED_MOTOR_STEPS_PER_INCH = 1000.0;  // Now adjustable via dashboard
float CUT_TRAVEL_DISTANCE = 9.2;  // Now adjustable via dashboard
float FEED_TRAVEL_DISTANCE = 3.43;  // Now adjustable via dashboard
float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1;  // Now adjustable via dashboard
float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4;  // Now adjustable via dashboard

// Motor homing direction constants
const int CUT_HOMING_DIRECTION = -1;
const int FEED_HOMING_DIRECTION = 1;

//* ************************************************************************
//* ************************ CUT MOTOR SPEED SETTINGS ********************
//* ************************************************************************
// Normal Cutting Operation (Cutting State)
float CUT_MOTOR_NORMAL_SPEED = 640;  // Now adjustable via dashboard
float CUT_MOTOR_NORMAL_ACCELERATION = 17000;  // Now adjustable via dashboard

// Return Stroke (Returning State / End of Cutting State)
float CUT_MOTOR_RETURN_SPEED = 25000;  // Now adjustable via dashboard

// Homing Operation (Homing State)
float CUT_MOTOR_HOMING_SPEED = 1500;  // Now adjustable via dashboard

//* ************************************************************************
//* ************************ FEED MOTOR SPEED SETTINGS *******************
//* ************************************************************************
// Normal Feed Operation (Feed State / Parts of Cutting State)
float FEED_MOTOR_NORMAL_SPEED = 22000;  // Now adjustable via dashboard
float FEED_MOTOR_NORMAL_ACCELERATION = 22000;  // Now adjustable via dashboard

// Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
float FEED_MOTOR_RETURN_SPEED = 22000;  // Now adjustable via dashboard
float FEED_MOTOR_RETURN_ACCELERATION = 30000;  // Now adjustable via dashboard

// Homing Operation (Homing State)
float FEED_MOTOR_HOMING_SPEED = 2000;  // Now adjustable via dashboard

//* ************************************************************************
//* ************************ TIMING CONFIGURATION *************************
//* ************************************************************************
// Servo timing configuration
unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2400;  // Now adjustable via dashboard

// Rotation clamp timing
unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 2300;  // Now adjustable via dashboard

// Cut motor homing timeout
unsigned long CUT_HOME_TIMEOUT = 5000; // 5 seconds - Now adjustable via dashboard

// Transfer Arm signal timing
unsigned long TA_SIGNAL_DURATION = 500;  // Now adjustable via dashboard

//* ************************************************************************
//* ************************ OPERATIONAL CONSTANTS ***********************
//* ************************************************************************
// Rotation clamp early activation offset
float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 2.7; // Now adjustable via dashboard

// Rotation servo early activation offset
float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = 0.053;  // Now adjustable via dashboard

// Transfer Arm signal early activation offset
float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = 0.01;  // Now adjustable via dashboard

//* ************************************************************************
//* ************************ SAFETY CONSTANTS *****************************
//* ************************************************************************
// Rotation servo safety timing
unsigned long ROTATION_SERVO_EXTENDED_WAIT_THRESHOLD_MS = 3000; // Now adjustable via dashboard
unsigned long ROTATION_SERVO_SAFETY_DELAY_MS = 3000; // Now adjustable via dashboard
unsigned long ROTATION_SERVO_RETURN_DELAY_MS = 150; // Now adjustable via dashboard 

//* ************************************************************************
//* ************************ MOTOR CONTROL CONSTANTS *********************
//* ************************************************************************
// Position and movement constants
long LARGE_POSITION_VALUE = 10000; // Now adjustable via dashboard
float FEED_MOTOR_RETURN_DISTANCE = 0.0;  // Now adjustable via dashboard
float FEED_MOTOR_OFFSET_FROM_SENSOR = 0.5;  // Now adjustable via dashboard

//* ************************************************************************
//* ************************ TIMING CONSTANTS *****************************
//* ************************************************************************
// Motor operation timing
unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000; // Now adjustable via dashboard
unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20; // Now adjustable via dashboard
unsigned long SENSOR_STABILIZATION_DELAY_MS = 30; // Now adjustable via dashboard
float SUCTION_SENSOR_CHECK_DISTANCE_INCHES = 0.2; // Now adjustable via dashboard

//* ************************************************************************
//* ******************** PRE-CALCULATED STEP VALUES ***********************
//* ************************************************************************
// Pre-calculated step values for cutting state to avoid repeated calculations
const long SUCTION_SENSOR_CHECK_DISTANCE_STEPS = SUCTION_SENSOR_CHECK_DISTANCE_INCHES * CUT_MOTOR_STEPS_PER_INCH;
const long ROTATION_CLAMP_ACTIVATION_POSITION_STEPS = (CUT_TRAVEL_DISTANCE - ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH;
const long ROTATION_SERVO_ACTIVATION_POSITION_STEPS = (CUT_TRAVEL_DISTANCE - ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH;
const long TA_SIGNAL_ACTIVATION_POSITION_STEPS = (CUT_TRAVEL_DISTANCE - TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH;