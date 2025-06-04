#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// Forward declarations for functions defined in 00_HOMING_FUNCTIONS.cpp
void homeZAxis();
void homeXAxis();

//* ************************************************************************
//* ************************ HOMING ***************************
//* ************************************************************************
// This file contains the main homing state machine logic for the Transfer Arm.
// The homing sequence coordinates X and Z axes to establish reference positions
// using limit switches.

// Main homing sequence that coordinates all axes according to the specified sequence
void homeSystem() {
  smartLog("Starting homing sequence...");

  //! Step 1: Home Z axis first
  homeZAxis();

  //! Step 2: Move Z axis up 5 inches
  transferArm.getZStepper()->moveTo(Z_UP_POS);

  // Wait for Z movement to complete
  while (transferArm.getZStepper()->isRunning()) {
    yield();
  }

  //! Step 3: Home X axis
  homeXAxis();

  //! Step 4: Move X axis to pickup position - BLOCKING
  transferArm.getXStepper()->moveTo(X_PICKUP_POS);
  // Wait for movement to complete
  while (transferArm.getXStepper()->isRunning()) {
    yield();
  }

  smartLog("Homing sequence completed");
} 