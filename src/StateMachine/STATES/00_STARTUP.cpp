#include "StateMachine/00_STARTUP.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** STARTUP STATE *******************************
//* ************************************************************************
// Function-based startup state handling.

//! ************************************************************************
//! STEP 1: TURN ON BLUE LED TO INDICATE STARTUP/HOMING
//! ************************************************************************

//! ************************************************************************
//! STEP 2: TRANSITION TO HOMING STATE
//! ************************************************************************

void executeStartupState() {
    turnBlueLedOn();  // Blue LED on during startup/homing
    changeState(HOMING);
}

void onEnterStartupState() {
    // No specific entry actions for startup state
}

void onExitStartupState() {
    // No specific exit actions for startup state
} 