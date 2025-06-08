#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
extern const int CUT_MOTOR_STEP_PIN;         // Step pulse signal for cutting motor
extern const int CUT_MOTOR_DIR_PIN;          // Direction control for cutting motor
extern const int FEED_MOTOR_STEP_PIN;   // Step pulse signal for feed motor (pushes wood forward for angled cuts)
extern const int FEED_MOTOR_DIR_PIN;    // Direction control for feed motor

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
extern const int ROTATION_SERVO_PIN;

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
  // Homing switches and sensors
  extern const int CUT_MOTOR_HOME_SWITCH;     // Active HIGH - input pulldown
  extern const int FEED_MOTOR_HOME_SENSOR;    // Active LOW - input pullup

// Control switches (Active HIGH - input pulldown)
extern const int RELOAD_SWITCH;
extern const int START_CYCLE_SWITCH;
extern const int MANUAL_FEED_SWITCH;         // Manual wood feed control

// Sensors (Active LOW - input pullup)
extern const int FIRST_CUT_OR_WOOD_FWD_ONE;      // Decides state: LOW = wood_fwd_one, HIGH = first_cut
extern const int _2x4_PRESENT_SENSOR;
extern const int WOOD_SUCTION_CONFIRM_SENSOR;  // Confirms wood is secured by suction

//* ************************************************************************
//* ************************ CLAMP PINS ***********************************
//* ************************************************************************
// Pneumatic clamp control pins (HIGH = extend, LOW = retract)
extern const int FEED_CLAMP;      // Clamps wood during feed positioning
extern const int _2x4_SECURE_CLAMP;    // Secures 2x4 during cutting
extern const int ROTATION_CLAMP;        // Clamps cut pieces for rotation

//* ************************************************************************
//* ************************ SIGNAL PINS **********************************
//* ************************************************************************
// Communication pins for external systems
extern const int TRANSFER_ARM_SIGNAL_PIN;  // Signal to Transfer Arm system

//* ************************************************************************
//* ************************ LED PINS *************************************
//* ************************************************************************
// Status indication LEDs
extern const int STATUS_LED_RED;      // Error/fault indication
extern const int STATUS_LED_YELLOW;   // Warning/caution indication
extern const int STATUS_LED_GREEN;    // Ready/operation OK indication
extern const int STATUS_LED_BLUE;     // Process active indication

#endif // PIN_DEFINITIONS_H 