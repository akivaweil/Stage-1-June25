#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// Forward declarations for functions defined in 03_TRANSPORT_SEQUENCE_FUNCTIONS.cpp
void initializeTransportSequence();
void updateTransportSequence();
bool moveToPosition(FastAccelStepper* stepper, float targetPosition);
bool Wait(unsigned long duration, unsigned long* timer);

//* ************************************************************************
//* ************************ TRANSPORT SEQUENCE ***************************
//* ************************************************************************
// This state handles transporting the object to the dropoff area including:
// - Rotating servo to travel position
// - Moving to dropoff overshoot position
// - Rotating servo to dropoff position
// - Moving back to dropoff position

// State variables
TransportSequenceState currentTransportState = TRANSPORT_ROTATE_SERVO_TO_TRAVEL;
unsigned long transportStateTimer = 0;

// Initialize the transport sequence
void initializeTransportSequence() {
  currentTransportState = TRANSPORT_ROTATE_SERVO_TO_TRAVEL;
  transportStateTimer = 0;
  smartLog("Transport sequence initialized");
}

// Update the transport sequence state machine
void updateTransportSequence() {
  switch (currentTransportState) {
    case TRANSPORT_ROTATE_SERVO_TO_TRAVEL:
      // Rotate servo to travel position after pickup
      transferArm.setServoPosition(SERVO_TRAVEL_POS);
      smartLog("Servo rotated to travel position, moving to dropoff overshoot");
      //! Step 1: Move to Dropoff Overshoot Position
      currentTransportState = TRANSPORT_MOVE_TO_OVERSHOOT;
      break;

    case TRANSPORT_MOVE_TO_OVERSHOOT:
      // Move X axis to overshoot position (4 inches past dropoff) - BLOCKING
      transferArm.getXStepper()->moveTo(X_DROPOFF_OVERSHOOT_POS);
      // Wait for movement to complete
      while (transferArm.getXStepper()->isRunning()) {
        yield();
      }
      transferArm.setServoPosition(SERVO_DROPOFF_POS);
      transportStateTimer = 0;
      //! Step 2: Wait for Servo Rotation
      currentTransportState = TRANSPORT_WAIT_FOR_SERVO_ROTATION;
      break;

    case TRANSPORT_WAIT_FOR_SERVO_ROTATION:
      // Wait for servo to complete rotation at overshoot position
      if (Wait(SERVO_ROTATION_WAIT_TIME, &transportStateTimer)) {
        smartLog("Servo rotation complete, returning to dropoff position");
        //! Step 3: Return to Dropoff Position
        currentTransportState = TRANSPORT_RETURN_TO_DROPOFF_POS;
      }
      break;

    case TRANSPORT_RETURN_TO_DROPOFF_POS:
      // Move X axis back to normal dropoff position - BLOCKING
      transferArm.getXStepper()->moveTo(X_DROPOFF_POS);
      // Wait for movement to complete
      while (transferArm.getXStepper()->isRunning()) {
        yield();
      }
      currentTransportState = TRANSPORT_COMPLETE;
      break;
      
    case TRANSPORT_COMPLETE:
      // Transport sequence finished, ready to transition to dropoff
      smartLog("Transport sequence complete, ready for dropoff");
      currentTransportState = TRANSPORT_ROTATE_SERVO_TO_TRAVEL;  // Reset for next cycle
      break;
  }
}

// Get current transport state
TransportSequenceState getCurrentTransportState() {
  return currentTransportState;
}

// Set transport state (for web control)
void setTransportState(TransportSequenceState newState) {
  currentTransportState = newState;
  transportStateTimer = 0;
} 