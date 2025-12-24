#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION ***************************
//* ************************************************************************

//* ************************************************************************
//* ************************ [⚙️] ROTATION SERVO **************************
//* ************************************************************************
// (Servo parameters moved to Config.h)

//* ************************************************************************
//* ************************ [🔧] ROTATION CLAMP **************************
//* ************************************************************************
// (Rotation clamp parameters moved to Config.h)

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
// (Transfer arm parameters moved to Config.h)

//* ************************************************************************
//* ************************ [⏱️] GENERAL TIMING ***************************
//* ************************************************************************
// (General timing parameters moved to Config.h)

//* ************************************************************************
//* ************************ [📐] PRE-CALCULATED STEPS *********************
//* ************************************************************************

#endif // MOTOR_CONFIG_H
