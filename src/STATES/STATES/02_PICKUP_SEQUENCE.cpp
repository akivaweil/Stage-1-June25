#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// Forward declarations for functions defined in 02_PICKUP_SEQUENCE_FUNCTIONS.cpp
void initializePickupSequence();
void updatePickupSequence();
bool moveToPosition(FastAccelStepper* stepper, float targetPosition);
bool Wait(unsigned long duration, unsigned long* timer);
void setupZAxisForPickup();
void activateVacuumDuringDescent();

//* ************************************************************************
//* ************************ PICKUP SEQUENCE ***************************
//* ************************************************************************
// This state handles the complete pickup sequence including:
// - Moving X-axis to pickup position
// - Lowering Z-axis for pickup
// - Activating vacuum during descent
// - Waiting at pickup position
// - Raising Z-axis with object

// State variables
PickupSequenceState currentPickupState = PICKUP_MOVE_TO_PICKUP_POS;
unsigned long pickupStateTimer = 0;
bool vacuumActivatedDuringDescent = false;

// Initialize the pickup sequence
void initializePickupSequence() {
  currentPickupState = PICKUP_MOVE_TO_PICKUP_POS;
  pickupStateTimer = 0;
  vacuumActivatedDuringDescent = false;
  setupZAxisForPickup();
  smartLog("Pickup sequence initialized");
}

// Update the pickup sequence state machine
void updatePickupSequence() {
  switch (currentPickupState) {
    case PICKUP_MOVE_TO_PICKUP_POS:
      // Move X-axis to pickup position - BLOCKING
      transferArm.getXStepper()->moveTo(X_PICKUP_POS);
      // Wait for movement to complete
      while (transferArm.getXStepper()->isRunning()) {
        yield();
      }
      
      // Trigger photo capture on Raspberry Pi
      transferArm.sendBurstRequest();
      
      smartLog("Photo capture triggered. Lowering Z to pickup.");
      smartLog("Target Z: " + String(Z_PICKUP_POS) +
               ", Suction Start Z: " + String(Z_SUCTION_START_POS));
      transferArm.setServoPosition(SERVO_PICKUP_POS);
      vacuumActivatedDuringDescent = false;
      transferArm.getZStepper()->moveTo(Z_PICKUP_POS);
      //! Step 1: Lower Z-axis for Pickup
      currentPickupState = PICKUP_LOWER_Z_FOR_PICKUP;
      break;

    case PICKUP_LOWER_Z_FOR_PICKUP:
      // Check Stage 2 safety signal before lowering Z axis
      if (!transferArm.isStage2SafeForZLowering()) {
        // Wait for Stage 2 to signal it's safe (pin goes low)
        return;  // Stay in this state until safe
      }
      
      // Lower Z axis and activate vacuum mid-way
      activateVacuumDuringDescent();
      
      if (!transferArm.getZStepper()->isRunning()) {
        smartLog("Z fully lowered for pickup, waiting");
        pickupStateTimer = 0;
        //! Step 2: Wait at Pickup Position
        currentPickupState = PICKUP_WAIT_AT_PICKUP_POS;
      }
      break;

    case PICKUP_WAIT_AT_PICKUP_POS:
      // Wait for hold time at pickup position
      if (Wait(PICKUP_HOLD_TIME, &pickupStateTimer)) {
        smartLog("Pickup wait complete, raising Z-axis with object");
        //! Step 3: Raise Z-axis with Object
        currentPickupState = PICKUP_RAISE_Z_WITH_OBJECT;
      }
      break;

    case PICKUP_RAISE_Z_WITH_OBJECT:
      // Raise Z axis with object
      transferArm.getZStepper()->moveTo(Z_UP_POS);
      if (!transferArm.getZStepper()->isRunning()) {
        smartLog("Z-axis raised with object, pickup sequence complete");
        currentPickupState = PICKUP_COMPLETE;
      }
      break;
      
    case PICKUP_COMPLETE:
      // Pickup sequence finished, ready to transition to transport
      smartLog("Pickup sequence complete, ready for transport");
      currentPickupState = PICKUP_MOVE_TO_PICKUP_POS;  // Reset for next cycle
      break;
  }
}

// Get current pickup state
PickupSequenceState getCurrentPickupState() {
  return currentPickupState;
}

// Set pickup state (for web control)
void setPickupState(PickupSequenceState newState) {
  currentPickupState = newState;
  pickupStateTimer = 0;
  vacuumActivatedDuringDescent = false;
} 