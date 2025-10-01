#ifndef STATES_CONFIG_H
#define STATES_CONFIG_H

//* ************************************************************************
//* ************************ STATES CONFIGURATION ************************
//* ************************************************************************

// Servo Configuration
extern int ROTATION_SERVO_HOME_POSITION;
extern int ROTATION_SERVO_ACTIVE_POSITION;

// Motor Configuration
extern float CUT_MOTOR_STEPS_PER_INCH;
extern float FEED_MOTOR_STEPS_PER_INCH;
extern float CUT_TRAVEL_DISTANCE;
extern float FEED_TRAVEL_DISTANCE;
extern float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
extern float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
extern const int CUT_HOMING_DIRECTION;
extern const int FEED_HOMING_DIRECTION;

// Cut Motor Speed Settings
extern float CUT_MOTOR_NORMAL_SPEED;
extern float CUT_MOTOR_NORMAL_ACCELERATION;
extern float CUT_MOTOR_RETURN_SPEED;
extern float CUT_MOTOR_HOMING_SPEED;

// Feed Motor Speed Settings
extern float FEED_MOTOR_NORMAL_SPEED;
extern float FEED_MOTOR_NORMAL_ACCELERATION;
extern float FEED_MOTOR_RETURN_SPEED;
extern float FEED_MOTOR_RETURN_ACCELERATION;
extern float FEED_MOTOR_HOMING_SPEED;

// Timing Configuration
extern unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
extern unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS;
extern unsigned long CUT_HOME_TIMEOUT;
extern unsigned long TA_SIGNAL_DURATION;

// Operational Constants
extern float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
extern float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
extern float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;

// Safety Constants
extern unsigned long ROTATION_SERVO_EXTENDED_WAIT_THRESHOLD_MS;
extern unsigned long ROTATION_SERVO_SAFETY_DELAY_MS;
extern unsigned long ROTATION_SERVO_RETURN_DELAY_MS;

//* ************************************************************************
//* ************************ MOTOR CONTROL CONSTANTS *********************
//* ************************************************************************
// Position and movement constants
extern long LARGE_POSITION_VALUE;
extern float FEED_MOTOR_RETURN_DISTANCE;
extern float FEED_MOTOR_OFFSET_FROM_SENSOR;

//* ************************************************************************
//* ************************ TIMING CONSTANTS *****************************
//* ************************************************************************
// Motor operation timing
extern unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS;
extern unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS;
extern unsigned long SENSOR_STABILIZATION_DELAY_MS;
extern float SUCTION_SENSOR_CHECK_DISTANCE_INCHES;

//* ************************************************************************
//* ******************** PRE-CALCULATED STEP VALUES ***********************
//* ************************************************************************
// Pre-calculated step values for cutting state to avoid repeated calculations
extern const long SUCTION_SENSOR_CHECK_DISTANCE_STEPS;
extern const long ROTATION_CLAMP_ACTIVATION_POSITION_STEPS;
extern const long ROTATION_SERVO_ACTIVATION_POSITION_STEPS;
extern const long TA_SIGNAL_ACTIVATION_POSITION_STEPS;

#endif // STATES_CONFIG_H 