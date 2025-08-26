#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor step calculations and travel distances - defined in Config.h
// Cut motor incremental move settings - defined in Config.h

// Position and movement constants
const long LARGE_POSITION_VALUE = 10000; // Large position value for homing moves
const float FEED_MOTOR_RETURN_DISTANCE = 0.0; // Distance for feed motor return moves (inches)
const float FEED_MOTOR_OFFSET_FROM_SENSOR = 0.5; // Offset from home sensor for working zero (inches)

// Motor operation timing
const unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000; // Timeout for cut motor recovery attempts
const unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20; // Delay for final cut motor position verification
const unsigned long SENSOR_STABILIZATION_DELAY_MS = 30; // Delay for sensor reading stabilization
const float SUCTION_SENSOR_CHECK_DISTANCE_INCHES = 1.8; // Distance cut motor must travel before checking suction sensor

#endif // MOTOR_CONFIG_H
