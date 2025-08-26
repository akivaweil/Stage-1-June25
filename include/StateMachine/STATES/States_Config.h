#ifndef STATES_CONFIG_H
#define STATES_CONFIG_H

//* ************************************************************************
//* ************************ STATES CONFIGURATION ************************
//* ************************************************************************
// This file now contains only general configuration constants that are shared
// across multiple states. State-specific constants have been moved to their
// respective state header files.

//* ************************************************************************
//* ************************ GENERAL MOTOR CONFIGURATION ******************
//* ************************************************************************
// These constants are now defined in Motor_Config.h and included via General_Functions.h

//* ************************************************************************
//* ************************ GENERAL SERVO CONFIGURATION ******************
//* ************************************************************************
// These constants are used across multiple states and functions
extern const int ROTATION_SERVO_HOME_POSITION;
extern const int ROTATION_SERVO_ACTIVE_POSITION;
extern const unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
extern const unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS;
extern const unsigned long TA_SIGNAL_DURATION;
extern const unsigned long ROTATION_SERVO_EXTENDED_WAIT_THRESHOLD_MS;
extern const unsigned long ROTATION_SERVO_SAFETY_DELAY_MS;

#endif // STATES_CONFIG_H 