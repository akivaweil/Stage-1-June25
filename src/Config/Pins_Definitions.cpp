#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ PIN DEFINITIONS ******************************
//* ************************************************************************

// Pin Definitions
// Inputs
const int START_BUTTON_PIN = 2;    // Start button input (active high)
const int STAGE1_SIGNAL_PIN = 23;  // Stage 1 machine signal input (active high)
const int X_HOME_SWITCH_PIN = 15;  // X-axis home limit switch (active high)
const int Z_HOME_SWITCH_PIN = 13;  // Z-axis home limit switch (active high)
const int STOP_SIGNAL_STAGE_2 = 22;   // Stage 2 safety signal (active high, wait for low)

// Outputs
const int X_STEP_PIN = 27;          // X-axis stepper motor step pin
const int X_DIR_PIN = 14;           // X-axis stepper motor direction pin
const int X_ENABLE_PIN = 4;        // X-axis stepper motor enable pin (active low)
const int Z_STEP_PIN = 19;          // Z-axis stepper motor step pin
const int Z_DIR_PIN = 18;           // Z-axis stepper motor direction pin
const int SERVO_PIN = 26;           // Servo control pin
const int SOLENOID_RELAY_PIN = 33;  // Solenoid relay control pin
const int STAGE2_SIGNAL_PIN = 25;   // Signal output to Stage 2 machine (active high) 