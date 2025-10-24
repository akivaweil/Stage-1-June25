#include "Config/Motor_Config.h"

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION ***************************
//* ************************************************************************

//* ************************************************************************
//* ************************ [⚙️] ROTATION SERVO **************************
//* ************************************************************************
// (Servo parameters moved to config.cpp)

//* ************************************************************************
//* ************************ [🔧] ROTATION CLAMP **************************
//* ************************************************************************
// (Rotation clamp parameters moved to config.cpp)

//* ************************************************************************
//* ************************ [🔄] CUT MOTOR ********************************
//* ************************************************************************
float CUT_MOTOR_STEPS_PER_INCH = 500.0;
const int CUT_HOMING_DIRECTION = -1;

// Speeds
float CUT_MOTOR_NORMAL_SPEED = 640;
float CUT_MOTOR_NORMAL_ACCELERATION = 16000;
float CUT_MOTOR_NO_WOOD_SPEED = 512; // 80% of normal speed
float CUT_MOTOR_RETURN_SPEED = 20000;
float CUT_MOTOR_HOMING_SPEED = 1500;

// Distances & Movement
float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1;
float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4;
float SUCTION_SENSOR_CHECK_DISTANCE_INCHES = 0.5;

//* ************************************************************************
//* ************************ [⚡] FEED MOTOR *******************************
//* ************************************************************************
float FEED_MOTOR_STEPS_PER_INCH = 1000.0;
const int FEED_HOMING_DIRECTION = 1;
const float FEED_MOTOR_OFFSET_FROM_SENSOR = 0.5;

// Speeds
float FEED_MOTOR_NORMAL_SPEED = 22000;
float FEED_MOTOR_NORMAL_ACCELERATION = 22000;
float FEED_MOTOR_RETURN_SPEED = 40000;
float FEED_MOTOR_RETURN_ACCELERATION = 50000;
float FEED_MOTOR_HOMING_SPEED = 2000;

// Distances & Movement
float FEED_MOTOR_RETURN_DISTANCE = 0.0;

//* ************************************************************************
//* ************************ [🤖] TRANSFER ARM *****************************
//* ************************************************************************
// (Transfer arm parameters moved to config.cpp)

//* ************************************************************************
//* ************************ [⏱️] GENERAL TIMING ***************************
//* ************************************************************************
// (General timing parameters moved to config.cpp)

//* ************************************************************************
//* ************************ [📐] PRE-CALCULATED STEPS *********************
//* ************************************************************************
const long SUCTION_SENSOR_CHECK_DISTANCE_STEPS = SUCTION_SENSOR_CHECK_DISTANCE_INCHES * CUT_MOTOR_STEPS_PER_INCH;
