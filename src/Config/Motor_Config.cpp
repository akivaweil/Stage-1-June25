#include "Config/Motor_Config.h"

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION ***************************
//* ************************************************************************

//* ************************************************************************
//* ************************ [⚙️] ROTATION SERVO **************************
//* ************************************************************************
int ROTATION_SERVO_HOME_POSITION = 27;
int ROTATION_SERVO_ACTIVE_POSITION = 120;

unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2000;
unsigned long ROTATION_SERVO_RETURN_DELAY_MS = 150;
unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS = 500;

float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = 0.2;

//* ************************************************************************
//* ************************ [🔧] ROTATION CLAMP **************************
//* ************************************************************************
unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 2400;
float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 2.7;

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

// Timing
unsigned long CUT_HOME_TIMEOUT = 5000;
unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000;
unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20;

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
unsigned long TA_SIGNAL_DURATION = 1000;
float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = 0.01;

//* ************************************************************************
//* ************************ [⏱️] GENERAL TIMING ***************************
//* ************************************************************************
unsigned long SENSOR_STABILIZATION_DELAY_MS = 30;

//* ************************************************************************
//* ************************ [💡] LED BLINKING *****************************
//* ************************************************************************
unsigned long LED_BLINK_ON_DURATION_MS = 3000;  // LED on duration for no-wood indication
unsigned long LED_BLINK_OFF_DURATION_MS = 100; // LED off duration for no-wood indication

//* ************************************************************************
//* ************************ [📐] PRE-CALCULATED STEPS *********************
//* ************************************************************************
const long SUCTION_SENSOR_CHECK_DISTANCE_STEPS = SUCTION_SENSOR_CHECK_DISTANCE_INCHES * CUT_MOTOR_STEPS_PER_INCH;
