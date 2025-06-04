#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// Forward declarations for functions defined in 05_COMPLETION_SEQUENCE_FUNCTIONS.cpp
void initializeCompletionSequence();
void updateCompletionSequence();
bool moveToPosition(FastAccelStepper* stepper, float targetPosition);
void homeXAxis();
void signalStage2();
void setupStage2Signal();

//* ************************************************************************
//* ************************ COMPLETION SEQUENCE ***************************
//* ************************************************************************
// This state handles the final completion sequence including:
// - Signaling Stage 2 machine
// - Returning to pickup position
// - Homing X-axis
// - Final move to pickup position

// State variables
CompletionSequenceState currentCompletionState = COMPLETION_SIGNAL_STAGE2_STATE;
unsigned long completionStateTimer = 0;

// Initialize the completion sequence
void initializeCompletionSequence() {
  currentCompletionState = COMPLETION_SIGNAL_STAGE2_STATE;
  completionStateTimer = 0;
  setupStage2Signal();
  smartLog("Completion sequence initialized");
}

// Update the completion sequence state machine
void updateCompletionSequence() {
  switch (currentCompletionState) {
    case COMPLETION_SIGNAL_STAGE2_STATE:
      // Send signal to Stage 2 machine
      signalStage2();

      //! Step 1: Return to Pickup Position (pre-homing)
      currentCompletionState = COMPLETION_RETURN_TO_PICKUP_PRE_HOME;
      break;

    case COMPLETION_RETURN_TO_PICKUP_PRE_HOME:
      // Return to pickup position to prepare for homing - BLOCKING
      transferArm.getXStepper()->moveTo(X_PICKUP_POS);
      // Wait for movement to complete
      while (transferArm.getXStepper()->isRunning()) {
        yield();
      }
      //! Step 2: Home X-axis
      currentCompletionState = COMPLETION_HOME_X_AXIS_STATE;
      break;

    case COMPLETION_HOME_X_AXIS_STATE:
      // Home the X-axis
      homeXAxis();  // This is a blocking call

      //! Step 3: Final Move to Pickup Position (post-homing)
      currentCompletionState = COMPLETION_FINAL_MOVE_TO_PICKUP_POS;
      break;

    case COMPLETION_FINAL_MOVE_TO_PICKUP_POS:
      // Move to pickup position after homing - BLOCKING
      transferArm.getXStepper()->moveTo(X_PICKUP_POS);
      // Wait for movement to complete
      while (transferArm.getXStepper()->isRunning()) {
        yield();
      }
      //! Cycle Complete: Ready for next cycle
      currentCompletionState = COMPLETION_COMPLETE;
      break;
      
    case COMPLETION_COMPLETE:
      // Completion sequence finished, ready to return to idle
      smartLog("Completion sequence complete, returning to idle state");
      currentCompletionState = COMPLETION_SIGNAL_STAGE2_STATE;  // Reset for next cycle
      break;
  }
}

// Get current completion state
CompletionSequenceState getCurrentCompletionState() {
  return currentCompletionState;
}

// Set completion state (for web control)
void setCompletionState(CompletionSequenceState newState) {
  currentCompletionState = newState;
  completionStateTimer = 0;
} 