#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

//* ************************************************************************
//* ************************ COMPLETION SEQUENCE FUNCTIONS ***************************
//* ************************************************************************
// Functions for completion sequence operations

// Setup Stage 2 signal pin
void setupStage2Signal() {
  pinMode(STAGE2_SIGNAL_PIN, OUTPUT);
  digitalWrite(STAGE2_SIGNAL_PIN, LOW);  // Initialize as LOW
  smartLog("Stage 2 signal pin configured as output, initialized LOW");
}

// Reset servo to pickup position
void resetServoToPickupPosition() {
  transferArm.setServoPosition(SERVO_PICKUP_POS);
  smartLog("Servo reset to pickup position: " + String(SERVO_PICKUP_POS));
}

// Move to pickup position with logging - BLOCKING
bool moveToPickupPosition() {
  transferArm.getXStepper()->moveTo(X_PICKUP_POS);
  // Wait for movement to complete
  while (transferArm.getXStepper()->isRunning()) {
    yield();
  }
  return true;  // Movement is guaranteed complete
}

// Check if at pickup position
bool isAtPickupPosition() {
  float currentPos = transferArm.getXStepper()->getCurrentPosition();
  float tolerance = 1.0;  // Allow 1 unit tolerance
  return abs(currentPos - X_PICKUP_POS) <= tolerance;
}

// Get completion sequence status for debugging
String getCompletionStatusString() {
  String status = "Completion Status - ";
  status += "X Position: " + String(transferArm.getXStepper()->getCurrentPosition());
  status += ", Target: " + String(X_PICKUP_POS);
  status += ", Servo: " + String(transferArm.getServoPosition());
  return status;
} 