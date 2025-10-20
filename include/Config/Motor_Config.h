#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION ***************************
//* ************************************************************************

//* ************************************************************************
//* ************************ [⚙️] ROTATION SERVO **************************
//* ************************************************************************
extern int ROTATION_SERVO_HOME_POSITION;
extern int ROTATION_SERVO_ACTIVE_POSITION;

extern unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
extern unsigned long ROTATION_SERVO_RETURN_DELAY_MS;
extern unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS;

extern float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;

//* ************************************************************************
//* ************************ [🔧] ROTATION CLAMP **************************
//* ************************************************************************
extern unsigned long ROTATION_CLAMP_RETRACT_DELAY_MS;
extern float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;

//* ************************************************************************
//* ************************ [🔄] CUT MOTOR ********************************
//* ************************************************************************
extern float CUT_MOTOR_STEPS_PER_INCH;
extern const int CUT_HOMING_DIRECTION;

// Speeds (defaults - can be modified via dashboard)
extern float CUT_MOTOR_NORMAL_SPEED;
extern float CUT_MOTOR_NORMAL_ACCELERATION;
extern float CUT_MOTOR_NO_WOOD_SPEED;
extern float CUT_MOTOR_RETURN_SPEED;
extern float CUT_MOTOR_HOMING_SPEED;

// Distances & Movement (defaults - can be modified via dashboard)
extern float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
extern float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
extern float SUCTION_SENSOR_CHECK_DISTANCE_INCHES;

// Timing
extern unsigned long CUT_HOME_TIMEOUT;
extern unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS;
extern unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS;

//* ************************************************************************
//* ************************ [⚡] FEED MOTOR *******************************
//* ************************************************************************
extern float FEED_MOTOR_STEPS_PER_INCH;
extern const int FEED_HOMING_DIRECTION;
extern const float FEED_MOTOR_OFFSET_FROM_SENSOR;

// Speeds (defaults - can be modified via dashboard)
extern float FEED_MOTOR_NORMAL_SPEED;
extern float FEED_MOTOR_NORMAL_ACCELERATION;
extern float FEED_MOTOR_RETURN_SPEED;
extern float FEED_MOTOR_RETURN_ACCELERATION;
extern float FEED_MOTOR_HOMING_SPEED;

// Distances & Movement (defaults - can be modified via dashboard)
extern float FEED_MOTOR_RETURN_DISTANCE;

//* ************************************************************************
//* ************************ [🤖] TRANSFER ARM *****************************
//* ************************************************************************
extern unsigned long TA_SIGNAL_DURATION;
extern float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;

//* ************************************************************************
//* ************************ [⏱️] GENERAL TIMING ***************************
//* ************************************************************************
extern unsigned long SENSOR_STABILIZATION_DELAY_MS;

//* ************************************************************************
//* ************************ [📐] PRE-CALCULATED STEPS *********************
//* ************************************************************************
extern const long SUCTION_SENSOR_CHECK_DISTANCE_STEPS;

#endif // MOTOR_CONFIG_H
