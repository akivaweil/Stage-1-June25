#include "Config/Config.h"

//* ************************************************************************
//* ************************ CONFIGURATION VALUES ************************
//* ************************************************************************

// Board identification
const char* BOARD_ID = "ESP32-S3-TRANSFER-ARM";
const char* BOARD_DESCRIPTION = "Transfer Arm Control Board";

// Constants
const float STEPS_PER_REV = 400.0;  // Steps per revolution for steppers (1.8° with 1/2 microstepping)
const float PULLEY_TEETH = 20.0;    // Number of teeth on the pulley
const float BELT_PITCH = 2.0;       // GT2 belt pitch in mm
const float STEPS_PER_MM = STEPS_PER_REV / (PULLEY_TEETH * BELT_PITCH);  // Steps per mm of linear movement
const float STEPS_PER_INCH = STEPS_PER_MM * 25.4;  // Convert to steps per inch

// Positions
const float X_HOME_POS = 0.0;  // X-axis home position (in steps)
const float Z_HOME_POS = 0.0;  // Z-axis home position (in steps)

// X-axis positions in inches from home
const float X_PICKUP_POS_INCHES = 1.0;      // X-axis pickup position (1 inch)
const float X_DROPOFF_POS_INCHES = 20.95;    // X-axis dropoff position (20 inches)
const float X_DROPOFF_OVERSHOOT_INCHES = X_DROPOFF_POS_INCHES + 1.75;  // 3 inches past dropoff for servo rotation
const float X_SERVO_ROTATE_INCHES = X_DROPOFF_POS_INCHES - 2.0;  // Start servo rotation 2 inches before dropoff
const float X_MIDPOINT_INCHES = (X_PICKUP_POS_INCHES + X_DROPOFF_POS_INCHES) / 2.0;  // Midpoint kept for reference

// Z-axis distances in inches
const float Z_PICKUP_LOWER_INCHES = 7.0;   // Lower Z-axis by 5 inches for pickup
const float Z_SUCTION_START_INCHES = 4.0;  // Start suction when Z is 4 inches down
const float Z_DROPOFF_LOWER_INCHES = 5.5;  // Lower Z-axis by 5.5 inches for dropoff

// Converted positions to steps
const float X_PICKUP_POS = X_PICKUP_POS_INCHES * STEPS_PER_INCH;
const float X_DROPOFF_POS = X_DROPOFF_POS_INCHES * STEPS_PER_INCH;
const float X_DROPOFF_OVERSHOOT_POS = X_DROPOFF_OVERSHOOT_INCHES * STEPS_PER_INCH;  // Overshoot position in steps
const float X_SERVO_ROTATE_POS = X_SERVO_ROTATE_INCHES * STEPS_PER_INCH;  // Position to start servo rotation for dropoff
const float X_MIDPOINT_POS = X_MIDPOINT_INCHES * STEPS_PER_INCH;  // Kept for reference

const float Z_UP_POS = 0.0;  // Z-axis fully up position
const float Z_PICKUP_POS = Z_PICKUP_LOWER_INCHES * STEPS_PER_INCH;  // Z-axis down position for pickup
const float Z_SUCTION_START_POS = Z_SUCTION_START_INCHES * STEPS_PER_INCH;  // Z position to start suction
const float Z_DROPOFF_POS = Z_DROPOFF_LOWER_INCHES * STEPS_PER_INCH;  // Z-axis down position for dropoff

// Servo angles
const float SERVO_HOME_POS = 90.0;    // Servo home position (in degrees)
const float SERVO_PICKUP_POS = 10.0;  // Servo pickup position (in degrees)
const float SERVO_TRAVEL_POS = 0.0;   // Servo position for travel after pickup (in degrees)
const float SERVO_DROPOFF_POS = 80.0; // Servo dropoff position (90 degrees from pickup)

// Timing constants
const unsigned long PICKUP_HOLD_TIME = 300;  // Hold time at pickup position (300ms)
const unsigned long DROPOFF_HOLD_TIME = 100; // Hold time at dropoff position (100ms)
const unsigned long SERVO_ROTATION_WAIT_TIME = 500;  // Wait time for servo to complete rotation at overshoot position (500ms)

// Stepper settings
const float X_MAX_SPEED = 7000.0;      // Maximum speed for X-axis in steps per second
const float X_ACCELERATION = 10000.0;   // Acceleration for X-axis in steps per second^2
const float Z_MAX_SPEED = 10000.0;      // Maximum speed for Z-axis in steps per second
const float Z_ACCELERATION = 10000.0;   // Acceleration for Z-axis in steps per second^2
const float Z_DROPOFF_MAX_SPEED = Z_MAX_SPEED / 1.0;  // Same speed for now for dropoff
const float Z_DROPOFF_ACCELERATION = Z_ACCELERATION / 1.0;  // Same acceleration for now for dropoff
const float X_HOME_SPEED = 1000.0;      // Homing speed for X-axis in steps per second
const float Z_HOME_SPEED = 1000.0;      // Homing speed for Z-axis in steps per second 