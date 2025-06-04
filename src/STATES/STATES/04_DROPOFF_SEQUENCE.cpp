#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// Forward declarations for functions defined in 04_DROPOFF_SEQUENCE_FUNCTIONS.cpp
void initializeDropoffSequence();
void updateDropoffSequence();
bool Wait(unsigned long duration, unsigned long* timer);
void setupZAxisForDropoff();
void releaseObject();
void restoreZAxisToNormalSpeed();

//* ************************************************************************
//* ************************ DROPOFF SEQUENCE ***************************
//* ************************************************************************
// This state handles the dropoff sequence including:
// - Lowering Z-axis for dropoff at slower speed
// - Releasing the object (turning off vacuum)
// - Waiting briefly after release
// - Raising Z-axis back up

// State variables
DropoffSequenceState currentDropoffState = DROPOFF_LOWER_Z_FOR_DROPOFF;
unsigned long dropoffStateTimer = 0;

// Initialize the dropoff sequence
void initializeDropoffSequence() {
  currentDropoffState = DROPOFF_LOWER_Z_FOR_DROPOFF;
  dropoffStateTimer = 0;
  setupZAxisForDropoff();
  smartLog("Dropoff sequence initialized");
}

// Update the dropoff sequence state machine
void updateDropoffSequence() {
  switch (currentDropoffState) {
    case DROPOFF_LOWER_Z_FOR_DROPOFF:
      // Check Stage 2 safety signal before lowering Z axis
      if (!transferArm.isStage2SafeForZLowering()) {
        // Wait for Stage 2 to signal it's safe (pin goes low)
        return;  // Stay in this state until safe
      }
      
      // Lower Z axis for dropoff at slower speed
      transferArm.getZStepper()->moveTo(Z_DROPOFF_POS);
      if (!transferArm.getZStepper()->isRunning()) {
        smartLog("Z-axis lowered for dropoff, releasing object");
        //! Step 1: Release Object
        currentDropoffState = DROPOFF_RELEASE_OBJECT_STATE;
      }
      break;

    case DROPOFF_RELEASE_OBJECT_STATE:
      // Turn off the vacuum solenoid
      releaseObject();
      smartLog("Object released, waiting briefly");
      dropoffStateTimer = 0;
      //! Step 2: Wait After Release
      currentDropoffState = DROPOFF_WAIT_AFTER_RELEASE;
      break;

    case DROPOFF_WAIT_AFTER_RELEASE:
      // Wait briefly after release
      if (Wait(DROPOFF_HOLD_TIME, &dropoffStateTimer)) {
        smartLog("Wait complete, raising Z-axis");
        restoreZAxisToNormalSpeed();
        //! Step 3: Raise Z-axis After Dropoff
        currentDropoffState = DROPOFF_RAISE_Z_AFTER_DROPOFF;
      }
      break;

    case DROPOFF_RAISE_Z_AFTER_DROPOFF:
      // Raise Z axis after dropoff
      transferArm.getZStepper()->moveTo(Z_UP_POS);
      if (!transferArm.getZStepper()->isRunning()) {
        smartLog("Z-axis raised, dropoff sequence complete");
        currentDropoffState = DROPOFF_COMPLETE;
      }
      break;
      
    case DROPOFF_COMPLETE:
      // Dropoff sequence finished, ready to transition to completion
      smartLog("Dropoff sequence complete, ready for completion sequence");
      currentDropoffState = DROPOFF_LOWER_Z_FOR_DROPOFF;  // Reset for next cycle
      break;
  }
}

// Get current dropoff state
DropoffSequenceState getCurrentDropoffState() {
  return currentDropoffState;
}

// Set dropoff state (for web control)
void setDropoffState(DropoffSequenceState newState) {
  currentDropoffState = newState;
  dropoffStateTimer = 0;
} 