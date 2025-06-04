#ifndef PINS_DEFINITIONS_H
#define PINS_DEFINITIONS_H

// Pin Definitions
// Inputs
extern const int START_BUTTON_PIN;    // Start button input (active high)
extern const int STAGE1_SIGNAL_PIN;  // Stage 1 machine signal input (active high)
extern const int X_HOME_SWITCH_PIN;  // X-axis home limit switch (active high)
extern const int Z_HOME_SWITCH_PIN;  // Z-axis home limit switch (active high)
extern const int STOP_SIGNAL_STAGE_2;   // Stage 2 safety signal (active high, wait for low)

// Outputs
extern const int X_STEP_PIN;          // X-axis stepper motor step pin
extern const int X_DIR_PIN;           // X-axis stepper motor direction pin
extern const int X_ENABLE_PIN;       // X-axis stepper motor enable pin (active low)
extern const int Z_STEP_PIN;          // Z-axis stepper motor step pin
extern const int Z_DIR_PIN;           // Z-axis stepper motor direction pin
extern const int SERVO_PIN;           // Servo control pin
extern const int SOLENOID_RELAY_PIN;  // Solenoid relay control pin
extern const int STAGE2_SIGNAL_PIN;   // Signal output to Stage 2 machine (active high)

#endif  // PINS_DEFINITIONS_H 