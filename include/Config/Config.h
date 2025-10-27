#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

#include "Config/Motor_Config.h"

//* ************************************************************************
//* ************************ [⚙️] ROTATION SERVO **************************
//* ************************************************************************
extern int ROTATION_SERVO_HOME_POSITION;
extern int ROTATION_SERVO_ACTIVE_POSITION;
extern unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
extern unsigned long ROTATION_SERVO_RETURN_DELAY_MS;
extern unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS;
extern unsigned long ROTATION_SERVO_SUCTION_HOLD_DURATION_MS;
extern float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;

//* ************************************************************************
//* ************************ [🔧] ROTATION CLAMP **************************
//* ************************************************************************
extern unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS;
extern float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;

//* ************************************************************************
//* ************************ [🔄] CUT MOTOR TIMING *************************
//* ************************************************************************
extern unsigned long CUT_HOME_TIMEOUT;
extern unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS;
extern unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS;

//* ************************************************************************
//* ************************ [🤖] TRANSFER ARM *****************************
//* ************************************************************************
extern unsigned long TA_SIGNAL_DURATION;
extern float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;

//* ************************************************************************
//* ************************ [⏱️] GENERAL TIMING ***************************
//* ************************************************************************
extern unsigned long SENSOR_STABILIZATION_DELAY_MS;

#endif // SYSTEM_CONFIG_H 