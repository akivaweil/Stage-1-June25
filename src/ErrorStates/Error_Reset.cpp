#include "ErrorStates/Error_Reset.h"
#include "StateMachine/StateManager.h"

// External references to functions from main.cpp (LED functions only)
extern void turnRedLedOff();
extern void turnYellowLedOff();

//* ************************************************************************
//* ************************** ERROR_RESET *********************************
//* ************************************************************************
// Handles the reset sequence after an error has been acknowledged.
// Step 1: Turn off red and yellow error LEDs.
// Step 2: Reset errorAcknowledged and woodSuctionError flags using StateManager.
// Step 3: Transition to STARTUP state to re-initialize the system (which will lead to HOMING).
void handleErrorResetState() {
    //serial.println("Entering error reset state.");
    
    // Turn off error LEDs
    turnRedLedOff();
    turnYellowLedOff();
    
    // Reset flags using StateManager
    stateManager.setErrorAcknowledged(false);
    stateManager.setWoodSuctionError(false);
    
    // Return to homing state to re-initialize using StateManager
    stateManager.changeState(STARTUP);
    //serial.println("Error reset complete, restarting system. Transitioning to STARTUP.");
} 