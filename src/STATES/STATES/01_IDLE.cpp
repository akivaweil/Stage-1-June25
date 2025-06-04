#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// Forward declarations for functions defined in 01_IDLE_FUNCTIONS.cpp
void initializeIdleState();
void updateIdleState();
bool checkPickCycleTrigger();

//* ************************************************************************
//* ************************ IDLE STATE ***************************
//* ************************************************************************
// This state waits for triggers to start the pick cycle sequence.
// It monitors the start button and Stage 1 signal for activation.

// State variables
IdleState currentIdleState = IDLE_WAIT;
unsigned long idleStateTimer = 0;

// Initialize the idle state
void initializeIdleState() {
  currentIdleState = IDLE_WAIT;
  idleStateTimer = 0;
  smartLog("Idle state initialized - ready for pick cycle trigger");
}

// Update the idle state machine
void updateIdleState() {
  switch (currentIdleState) {
    case IDLE_WAIT:
      // Check for pick cycle trigger
      if (checkPickCycleTrigger()) {
        smartLog("Pick Cycle Triggered from idle state");
        currentIdleState = TRIGGER_DETECTED;
        idleStateTimer = 0;
      }
      break;
      
    case TRIGGER_DETECTED:
      // Transition to pickup sequence will be handled by main state machine
      smartLog("Transitioning to pickup sequence");
      currentIdleState = IDLE_WAIT;  // Reset for next cycle
      break;
  }
}

// Get current idle state
IdleState getCurrentIdleState() {
  return currentIdleState;
}

// Set idle state (for web control)
void setIdleState(IdleState newState) {
  currentIdleState = newState;
  idleStateTimer = 0;
} 