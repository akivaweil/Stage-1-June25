#ifndef CONFIG_H
#define CONFIG_H

// Board identification
extern const char* BOARD_ID;
extern const char* BOARD_DESCRIPTION;

// Constants
extern const float STEPS_PER_REV;  // Steps per revolution for steppers (1.8° with 1/2 microstepping)
extern const float PULLEY_TEETH;    // Number of teeth on the pulley
extern const float BELT_PITCH;       // GT2 belt pitch in mm
extern const float STEPS_PER_MM;  // Steps per mm of linear movement
extern const float STEPS_PER_INCH;  // Convert to steps per inch

// Positions
extern const float X_HOME_POS;  // X-axis home position (in steps)
extern const float Z_HOME_POS;  // Z-axis home position (in steps)

// X-axis positions in inches from home
extern const float X_PICKUP_POS_INCHES;      // X-axis pickup position (1 inch)
extern const float X_DROPOFF_POS_INCHES;    // X-axis dropoff position (20 inches)
extern const float X_DROPOFF_OVERSHOOT_INCHES;  // 3 inches past dropoff for servo rotation
extern const float X_SERVO_ROTATE_INCHES;  // Start servo rotation 2 inches before dropoff
extern const float X_MIDPOINT_INCHES;  // Midpoint kept for reference

// Z-axis distances in inches
extern const float Z_PICKUP_LOWER_INCHES;   // Lower Z-axis by 5 inches for pickup
extern const float Z_SUCTION_START_INCHES;  // Start suction when Z is 4 inches down
extern const float Z_DROPOFF_LOWER_INCHES;  // Lower Z-axis by 5.5 inches for dropoff

// Converted positions to steps
extern const float X_PICKUP_POS;
extern const float X_DROPOFF_POS;
extern const float X_DROPOFF_OVERSHOOT_POS;  // Overshoot position in steps
extern const float X_SERVO_ROTATE_POS;  // Position to start servo rotation for dropoff
extern const float X_MIDPOINT_POS;  // Kept for reference

extern const float Z_UP_POS;  // Z-axis fully up position
extern const float Z_PICKUP_POS;  // Z-axis down position for pickup
extern const float Z_SUCTION_START_POS;  // Z position to start suction
extern const float Z_DROPOFF_POS;  // Z-axis down position for dropoff

// Servo angles
extern const float SERVO_HOME_POS;    // Servo home position (in degrees)
extern const float SERVO_PICKUP_POS;  // Servo pickup position (in degrees)
extern const float SERVO_TRAVEL_POS;   // Servo position for travel after pickup (in degrees)
extern const float SERVO_DROPOFF_POS; // Servo dropoff position (90 degrees from pickup)

// Timing constants
extern const unsigned long PICKUP_HOLD_TIME;  // Hold time at pickup position (300ms)
extern const unsigned long DROPOFF_HOLD_TIME; // Hold time at dropoff position (100ms)
extern const unsigned long SERVO_ROTATION_WAIT_TIME;  // Wait time for servo to complete rotation at overshoot position (500ms)

// Stepper settings
extern const float X_MAX_SPEED;      // Maximum speed for X-axis in steps per second
extern const float X_ACCELERATION;   // Acceleration for X-axis in steps per second^2
extern const float Z_MAX_SPEED;      // Maximum speed for Z-axis in steps per second
extern const float Z_ACCELERATION;   // Acceleration for Z-axis in steps per second^2
extern const float Z_DROPOFF_MAX_SPEED;  // Same speed for now for dropoff
extern const float Z_DROPOFF_ACCELERATION;  // Same acceleration for now for dropoff
extern const float X_HOME_SPEED;      // Homing speed for X-axis in steps per second
extern const float Z_HOME_SPEED;      // Homing speed for Z-axis in steps per second

// State enum for pick cycle
enum PickCycleState {
  IDLE,
  MOVE_TO_PICKUP,
  LOWER_Z_FOR_PICKUP,
  WAIT_AT_PICKUP,
  RAISE_Z_WITH_OBJECT,
  ROTATE_SERVO_AFTER_PICKUP,
  MOVE_TO_DROPOFF_OVERSHOOT,
  WAIT_FOR_SERVO_ROTATION,
  RETURN_TO_DROPOFF,
  LOWER_Z_FOR_DROPOFF,
  RELEASE_OBJECT,
  WAIT_AFTER_RELEASE,
  RAISE_Z_AFTER_DROPOFF,
  SIGNAL_STAGE2,
  RETURN_TO_PICKUP,
  HOME_X_AXIS,
  FINAL_MOVE_TO_PICKUP
};

// State enums for split pick cycle sequences
enum IdleState {
  IDLE_WAIT,
  TRIGGER_DETECTED
};

enum PickupSequenceState {
  PICKUP_MOVE_TO_PICKUP_POS,
  PICKUP_LOWER_Z_FOR_PICKUP,
  PICKUP_WAIT_AT_PICKUP_POS,
  PICKUP_RAISE_Z_WITH_OBJECT,
  PICKUP_COMPLETE
};

enum TransportSequenceState {
  TRANSPORT_ROTATE_SERVO_TO_TRAVEL,
  TRANSPORT_MOVE_TO_OVERSHOOT,
  TRANSPORT_WAIT_FOR_SERVO_ROTATION,
  TRANSPORT_RETURN_TO_DROPOFF_POS,
  TRANSPORT_COMPLETE
};

enum DropoffSequenceState {
  DROPOFF_LOWER_Z_FOR_DROPOFF,
  DROPOFF_RELEASE_OBJECT_STATE,
  DROPOFF_WAIT_AFTER_RELEASE,
  DROPOFF_RAISE_Z_AFTER_DROPOFF,
  DROPOFF_COMPLETE
};

enum CompletionSequenceState {
  COMPLETION_SIGNAL_STAGE2_STATE,
  COMPLETION_RETURN_TO_PICKUP_PRE_HOME,
  COMPLETION_HOME_X_AXIS_STATE,
  COMPLETION_FINAL_MOVE_TO_PICKUP_POS,
  COMPLETION_COMPLETE
};

#endif  // CONFIG_H 