#pragma once

// Motor Configuration

// [⚙️] Rotation Servo
// (Servo parameters moved to Config.h)

// [🔧] Rotation Clamp
// (Rotation clamp parameters moved to Config.h)

// [🔄] Cut Motor
extern float CUT_MOTOR_STEPS_PER_INCH;
extern const int CUT_HOMING_DIRECTION;

// Speeds (in inches/sec and inches/sec² - defaults can be modified via dashboard)
extern float CUT_MOTOR_NORMAL_SPEED; // inches/sec
extern float CUT_MOTOR_NORMAL_ACCELERATION; // inches/sec²
extern float CUT_MOTOR_NO_WOOD_SPEED; // inches/sec
extern float CUT_MOTOR_RETURN_SPEED; // inches/sec
extern float CUT_MOTOR_HOMING_SPEED; // inches/sec

// Distances & Movement (defaults - can be modified via dashboard)
extern float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
extern float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;

// [⚡] Feed Motor
extern float FEED_MOTOR_STEPS_PER_INCH;
extern const int FEED_HOMING_DIRECTION;
extern float FEED_MOTOR_OFFSET_FROM_SENSOR;

// Speeds (defaults - can be modified via dashboard)
extern float FEED_MOTOR_NORMAL_SPEED;
extern float FEED_MOTOR_NORMAL_ACCELERATION;
extern float FEED_MOTOR_RETURN_SPEED;
extern float FEED_MOTOR_RETURN_ACCELERATION;
extern float FEED_MOTOR_HOMING_SPEED;

// Distances & Movement (defaults - can be modified via dashboard)
extern float FEED_MOTOR_RETURN_DISTANCE;
extern float FEED_TRAVEL_DISTANCE;

// [🤖] Transfer Arm
// (Transfer arm parameters moved to Config.h)

// [⏱️] General Timing
// (General timing parameters moved to Config.h)

// [📐] Pre-calculated Steps

