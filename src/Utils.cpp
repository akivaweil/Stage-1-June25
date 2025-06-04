#include "../include/Utils.h"
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include "../include/TransferArm.h"
#include "FastAccelStepper.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <ESP32Servo.h>

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ***************************
//* ************************************************************************
// This file contains utility functions for movement control, timing, servo control,
// axis control, status checking, and logging that are used throughout the Transfer Arm system.

//* ************************************************************************
//* ************************ MOVEMENT FUNCTIONS ***************************
//* ************************************************************************

// Move a stepper to a position and return true when motion is complete
bool moveToPosition(FastAccelStepper* stepper, float position) {
  static float lastTarget = -999999.0;   // Track the last target position
  static bool motionStarted = false;     // Track if motion has started

  // If this is a new target position or motion hasn't started yet
  if (position != lastTarget || !motionStarted) {
    smartLog("Setting new target position: " + String(position) +
             ", Current pos: " + String(stepper->getCurrentPosition()));

    // Set the target position
    stepper->moveTo(position);
    lastTarget = position;
    motionStarted = true;
    return false;  // Motion is not complete
  }

  // Return true if the motor has reached the target position
  if (!stepper->isRunning()) {
    motionStarted = false;  // Reset for next movement
    return true;            // Motion is complete
  }

  return false;  // Motion is still in progress
}

// Activate vacuum solenoid (cylinder extended)
void activateVacuum() {
  digitalWrite(SOLENOID_RELAY_PIN, HIGH);
  smartLog("Vacuum activated (cylinder extended)");
}

// Deactivate vacuum solenoid (cylinder retracted)
void deactivateVacuum() {
  digitalWrite(SOLENOID_RELAY_PIN, LOW);
  smartLog("Vacuum deactivated (cylinder retracted)");
}

//* ************************************************************************
//* ************************ TIMING FUNCTIONS ***************************
//* ************************************************************************

// Non-blocking delay function
bool Wait(unsigned long delayTime, unsigned long* startTimePtr) {
  // First time entering this function
  if (*startTimePtr == 0) {
    *startTimePtr = millis();
    return false;
  }

  // Check if the delay time has elapsed
  if (millis() - *startTimePtr >= delayTime) {
    *startTimePtr = 0;  // Reset for next use
    return true;
  }

  return false;
}

// Reset timing for state transitions
void resetStateTimer(unsigned long* timer) {
  *timer = 0;
}

//* ************************************************************************
//* ************************ SERVO FUNCTIONS ***************************
//* ************************************************************************

// Set servo to pickup position
void setServoPickup() {
  transferArm.setServoPosition(SERVO_PICKUP_POS);
  smartLog("Servo set to pickup position: " + String(SERVO_PICKUP_POS));
}

// Set servo to travel position
void setServoTravel() {
  transferArm.setServoPosition(SERVO_TRAVEL_POS);
  smartLog("Servo set to travel position: " + String(SERVO_TRAVEL_POS));
}

// Set servo to dropoff position
void setServoDropoff() {
  transferArm.setServoPosition(SERVO_DROPOFF_POS);
  smartLog("Servo set to dropoff position: " + String(SERVO_DROPOFF_POS));
}

// Set servo to home position
void setServoHome() {
  transferArm.setServoPosition(SERVO_HOME_POS);
  smartLog("Servo set to home position: " + String(SERVO_HOME_POS));
}

//* ************************************************************************
//* ************************ AXIS CONTROL FUNCTIONS ***************************
//* ************************************************************************

// Set Z-axis to normal operating speeds
void setZAxisNormalSpeed() {
  transferArm.getZStepper()->setSpeedInHz(Z_MAX_SPEED);
  transferArm.getZStepper()->setAcceleration(Z_ACCELERATION);
}

// Set Z-axis to slow dropoff speeds
void setZAxisDropoffSpeed() {
  transferArm.getZStepper()->setSpeedInHz(Z_DROPOFF_MAX_SPEED);
  transferArm.getZStepper()->setAcceleration(Z_DROPOFF_ACCELERATION);
}

// Move Z to pickup position
void moveZToPickup() {
  transferArm.getZStepper()->moveTo(Z_PICKUP_POS);
}

// Move Z to up position
void moveZToUp() {
  transferArm.getZStepper()->moveTo(Z_UP_POS);
}

// Move Z to dropoff position
void moveZToDropoff() {
  transferArm.getZStepper()->moveTo(Z_DROPOFF_POS);
}

// Move X to pickup position - BLOCKING
void moveXToPickup() {
  transferArm.getXStepper()->moveTo(X_PICKUP_POS);
  // Wait for movement to complete
  while (transferArm.getXStepper()->isRunning()) {
    yield();
  }
}

// Move X to dropoff position - BLOCKING
void moveXToDropoff() {
  transferArm.getXStepper()->moveTo(X_DROPOFF_POS);
  // Wait for movement to complete
  while (transferArm.getXStepper()->isRunning()) {
    yield();
  }
}

// Move X to dropoff overshoot position - BLOCKING
void moveXToDropoffOvershoot() {
  transferArm.getXStepper()->moveTo(X_DROPOFF_OVERSHOOT_POS);
  // Wait for movement to complete
  while (transferArm.getXStepper()->isRunning()) {
    yield();
  }
}

//* ************************************************************************
//* ************************ STATUS FUNCTIONS ***************************
//* ************************************************************************

// Check if Z axis is at target position
bool isZAxisAtTarget() {
  return (!transferArm.getZStepper()->isRunning());
}

// Check if X axis is at target position
bool isXAxisAtTarget() {
  return (!transferArm.getXStepper()->isRunning());
}

// Check if Z axis has reached suction start position
bool isZAtSuctionStart() {
  return (transferArm.getZStepper()->getCurrentPosition() >= Z_SUCTION_START_POS);
}

//* ************************************************************************
//* ************************ STATE FUNCTIONS ***************************
//* ************************************************************************

// Get state as string for debugging and status reporting
const char* getStateString(PickCycleState state) {
  switch (state) {
    case IDLE:
      return "IDLE";
    case MOVE_TO_PICKUP:
      return "MOVE_TO_PICKUP";
    case LOWER_Z_FOR_PICKUP:
      return "LOWER_Z_FOR_PICKUP";
    case WAIT_AT_PICKUP:
      return "WAIT_AT_PICKUP";
    case RAISE_Z_WITH_OBJECT:
      return "RAISE_Z_WITH_OBJECT";
    case ROTATE_SERVO_AFTER_PICKUP:
      return "ROTATE_SERVO_AFTER_PICKUP";
    case MOVE_TO_DROPOFF_OVERSHOOT:
      return "MOVE_TO_DROPOFF_OVERSHOOT";
    case WAIT_FOR_SERVO_ROTATION:
      return "WAIT_FOR_SERVO_ROTATION";
    case RETURN_TO_DROPOFF:
      return "RETURN_TO_DROPOFF";
    case LOWER_Z_FOR_DROPOFF:
      return "LOWER_Z_FOR_DROPOFF";
    case RELEASE_OBJECT:
      return "RELEASE_OBJECT";
    case WAIT_AFTER_RELEASE:
      return "WAIT_AFTER_RELEASE";
    case RAISE_Z_AFTER_DROPOFF:
      return "RAISE_Z_AFTER_DROPOFF";
    case SIGNAL_STAGE2:
      return "SIGNAL_STAGE2";
    case RETURN_TO_PICKUP:
      return "RETURN_TO_PICKUP";
    case HOME_X_AXIS:
      return "HOME_X_AXIS";
    case FINAL_MOVE_TO_PICKUP:
      return "FINAL_MOVE_TO_PICKUP";
    default:
      return "UNKNOWN";
  }
}

//* ************************************************************************
//* ************************ SIGNAL FUNCTIONS ***************************
//* ************************************************************************

// Send signal pulse to Stage 2
void signalStage2() {
  digitalWrite(STAGE2_SIGNAL_PIN, HIGH);
  delay(100);  // Brief pulse
  digitalWrite(STAGE2_SIGNAL_PIN, LOW);
  smartLog("Stage 2 signal sent");
}

//* ************************************************************************
//* ************************ LOGGING FUNCTIONS ***************************
//* ************************************************************************

// Smart logging function - outputs to Serial only
void smartLog(const String& message) {
  Serial.println(message);
}