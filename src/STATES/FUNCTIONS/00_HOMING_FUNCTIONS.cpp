#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"
#include "FastAccelStepper.h"
#include <Arduino.h>
#include <Bounce2.h>

//* ************************************************************************
//* ************************ HOMING FUNCTIONS ***************************
//* ************************************************************************
// This file contains all the individual functions needed for the homing sequence.
// Each axis has its own dedicated homing function with proper limit switch handling.

// Home the Z axis
void homeZAxis() {


  // Move towards home switch at constant speed
  transferArm.getZStepper()->setSpeedInHz(Z_HOME_SPEED);
  transferArm.getZStepper()->runBackward();

  // Keep stepping until home switch is triggered (active HIGH)
  while (transferArm.getZHomeSwitch().read() == LOW) {
    transferArm.getZHomeSwitch().update();
    yield();  // Allow ESP32 to handle background tasks
  }

  // Stop the motor immediately
  transferArm.getZStepper()->forceStop();

  // Set current position as home
  transferArm.getZStepper()->setCurrentPosition(Z_HOME_POS);

  // Restore Z-axis to normal operating speed after homing
  transferArm.getZStepper()->setSpeedInHz(Z_MAX_SPEED);
  transferArm.getZStepper()->setAcceleration(Z_ACCELERATION);

}

// Home the X axis  
void homeXAxis() {
  // Enable X motor before homing
  transferArm.enableXMotor();
  
  // Check if X home switch is already activated
  transferArm.getXHomeSwitch().update();
  if (transferArm.getXHomeSwitch().read() == HIGH) {
    transferArm.getXStepper()->forceStop();
    transferArm.getXStepper()->setCurrentPosition(X_HOME_POS);

    // Move away from the switch using a controlled move to 500 steps (away from home)
    transferArm.getXStepper()->setSpeedInHz(X_HOME_SPEED);
    transferArm.getXStepper()->setAcceleration(X_HOME_SPEED * 2.0);
    transferArm.getXStepper()->moveTo(500.0);  // Move to positive position (away from home)
    
    // Wait for movement to complete
    while (transferArm.getXStepper()->isRunning()) {
      yield();
    }

    // Restore X-axis to normal operating speed after homing
    transferArm.getXStepper()->setSpeedInHz(X_MAX_SPEED);
    transferArm.getXStepper()->setAcceleration(X_ACCELERATION);
    return;
  }

  // Move towards home switch
  transferArm.getXStepper()->setSpeedInHz(X_HOME_SPEED);
  transferArm.getXStepper()->runBackward();  // Negative direction towards home

  // Keep stepping until home switch is triggered (active HIGH)
  while (transferArm.getXHomeSwitch().read() == LOW) {
    transferArm.getXHomeSwitch().update();
    yield();  // Allow ESP32 to handle background tasks
  }

  // Stop the motor immediately
  transferArm.getXStepper()->forceStop();

  // Set current position as home
  transferArm.getXStepper()->setCurrentPosition(X_HOME_POS);

  // Move away from the switch using a controlled move to 500 steps (away from home)
  transferArm.getXStepper()->setSpeedInHz(X_HOME_SPEED);
  transferArm.getXStepper()->setAcceleration(X_HOME_SPEED * 2.0);
  transferArm.getXStepper()->moveTo(500.0);  // Move to positive position (away from home)
  
  // Wait for movement to complete
  while (transferArm.getXStepper()->isRunning()) {
    yield();
  }

  // Restore X-axis to normal operating speed after homing
  transferArm.getXStepper()->setSpeedInHz(X_MAX_SPEED);
  transferArm.getXStepper()->setAcceleration(X_ACCELERATION);
}