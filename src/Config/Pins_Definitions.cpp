#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
const int CUT_MOTOR_STEP_PIN = 12;         // Step pulse signal for cutting motor
const int CUT_MOTOR_DIR_PIN = 11;          // Direction control for cutting motor
const int FEED_MOTOR_STEP_PIN = 17;   // Step pulse signal for feed motor (pushes wood forward for angled cuts)
const int FEED_MOTOR_DIR_PIN = 18;    // Direction control for feed motor

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
const int ROTATION_SERVO_PIN = 14;

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
  // Homing switches and sensors
  const int CUT_MOTOR_HOME_SWITCH = 3;        // Active HIGH - input pulldown
  const int FEED_MOTOR_HOME_SENSOR = 13;      // Active LOW - input pullup

// Control switches (Active HIGH - input pulldown)
const int RELOAD_SWITCH = 6;
const int START_CYCLE_SWITCH = 5;
const int MANUAL_FEED_SWITCH = 41;         // Manual wood feed control

// Sensors (Active LOW - input pullup)
const int FIRST_CUT_OR_WOOD_FWD_ONE = 10;       // Decides state: LOW = wood_fwd_one, HIGH = first_cut
const int _2x4_PRESENT_SENSOR = 4;
const int WOOD_SUCTION_CONFIRM_SENSOR = 39;  // Confirms wood is secured by suction

//* ************************************************************************
//* ************************ CLAMP PINS ***********************************
//* ************************************************************************
// Pneumatic clamp control pins (HIGH = extend, LOW = retract)
const int FEED_CLAMP = 36;         // Clamps wood during feed positioning
const int _2x4_SECURE_CLAMP = 48;       // Secures 2x4 during cutting
const int ROTATION_CLAMP = 42;          // Clamps cut pieces for rotation

//* ************************************************************************
//* ************************ SIGNAL PINS **********************************
//* ************************************************************************
// Communication pins for external systems
const int TRANSFER_ARM_SIGNAL_PIN = 8;  // Signal to Transfer Arm system

//* ************************************************************************
//* ************************ LED PINS *************************************
//* ************************************************************************
// Status indication LEDs
const int STATUS_LED_RED = 47;      // Error/fault indication
const int STATUS_LED_YELLOW = 21;   // Warning/caution indication
const int STATUS_LED_GREEN = 37;    // Ready/operation OK indication
const int STATUS_LED_BLUE = 19;     // Process active indication 