#include "../include/PickCycle.h"
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include "../include/TransferArm.h"
#include "../include/Utils.h"

//* ************************************************************************
//* ************************ PICK CYCLE COORDINATOR ***************************
//* ************************************************************************
// This file coordinates the individual state sequences to create the complete
// pick cycle functionality. It acts as a bridge between the old PickCycle
// interface and the new modular state system.

// Forward declarations for state functions
extern void initializeIdleState();
extern void updateIdleState();
extern IdleState getCurrentIdleState();
extern void setIdleState(IdleState newState);

extern void initializePickupSequence();
extern void updatePickupSequence();
extern PickupSequenceState getCurrentPickupState();

extern void initializeTransportSequence();
extern void updateTransportSequence();
extern TransportSequenceState getCurrentTransportState();

extern void initializeDropoffSequence();
extern void updateDropoffSequence();
extern DropoffSequenceState getCurrentDropoffState();

extern void initializeCompletionSequence();
extern void updateCompletionSequence();
extern CompletionSequenceState getCurrentCompletionState();

// Current main state
enum MainState {
  MAIN_IDLE,
  MAIN_PICKUP_SEQUENCE,
  MAIN_TRANSPORT_SEQUENCE,
  MAIN_DROPOFF_SEQUENCE,
  MAIN_COMPLETION_SEQUENCE
};

MainState currentMainState = MAIN_IDLE;

// Initialize the pick cycle system
void initializePickCycle() {
  currentMainState = MAIN_IDLE;
  initializeIdleState();
  transferArm.disableXMotor();  // Ensure X motor starts disabled when idle
  smartLog("Pick cycle system initialized");
}

// Update the pick cycle system
void updatePickCycle() {
  switch (currentMainState) {
    case MAIN_IDLE:
      updateIdleState();
      // Check if idle state triggered a pick cycle
      if (getCurrentIdleState() == TRIGGER_DETECTED) {
        smartLog("Transitioning from idle to pickup sequence");
        transferArm.enableXMotor();  // Enable X motor for pick cycle
        currentMainState = MAIN_PICKUP_SEQUENCE;
        initializePickupSequence();
      }
      break;
      
    case MAIN_PICKUP_SEQUENCE:
      updatePickupSequence();
      // Check if pickup sequence is complete
      if (getCurrentPickupState() == PICKUP_COMPLETE) {
        smartLog("Transitioning from pickup to transport sequence");
        currentMainState = MAIN_TRANSPORT_SEQUENCE;
        initializeTransportSequence();
      }
      break;
      
    case MAIN_TRANSPORT_SEQUENCE:
      updateTransportSequence();
      // Check if transport sequence is complete
      if (getCurrentTransportState() == TRANSPORT_COMPLETE) {
        smartLog("Transitioning from transport to dropoff sequence");
        currentMainState = MAIN_DROPOFF_SEQUENCE;
        initializeDropoffSequence();
      }
      break;
      
    case MAIN_DROPOFF_SEQUENCE:
      updateDropoffSequence();
      // Check if dropoff sequence is complete
      if (getCurrentDropoffState() == DROPOFF_COMPLETE) {
        smartLog("Transitioning from dropoff to completion sequence");
        currentMainState = MAIN_COMPLETION_SEQUENCE;
        initializeCompletionSequence();
      }
      break;
      
    case MAIN_COMPLETION_SEQUENCE:
      updateCompletionSequence();
      // Check if completion sequence is complete
      if (getCurrentCompletionState() == COMPLETION_COMPLETE) {
        smartLog("Transitioning from completion back to idle");
        transferArm.disableXMotor();  // Disable X motor when returning to idle
        currentMainState = MAIN_IDLE;
        initializeIdleState();
      }
      break;
  }
}

// Get current state (for compatibility with old interface)
PickCycleState getCurrentState() {
  switch (currentMainState) {
    case MAIN_IDLE:
      return IDLE;
    case MAIN_PICKUP_SEQUENCE:
      return MOVE_TO_PICKUP;  // Simplified representation
    case MAIN_TRANSPORT_SEQUENCE:
      return MOVE_TO_DROPOFF_OVERSHOOT;  // Simplified representation
    case MAIN_DROPOFF_SEQUENCE:
      return LOWER_Z_FOR_DROPOFF;  // Simplified representation
    case MAIN_COMPLETION_SEQUENCE:
      return SIGNAL_STAGE2;  // Simplified representation
    default:
      return IDLE;
  }
}

// Set current state (for web control)
void setCurrentState(PickCycleState newState) {
  // This is a simplified implementation for web control
  // In practice, you might want more granular control
  switch (newState) {
    case IDLE:
      currentMainState = MAIN_IDLE;
      initializeIdleState();
      break;
    default:
      smartLog("State change not implemented for: " + String(getStateString(newState)));
      break;
  }
}

// Trigger pick cycle from web interface
void triggerPickCycleFromWeb() {
  if (currentMainState == MAIN_IDLE) {
    smartLog("Pick cycle triggered from web interface");
    setIdleState(TRIGGER_DETECTED);
  } else {
    smartLog("Pick cycle already in progress, ignoring web trigger");
  }
} 