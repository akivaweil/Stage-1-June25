#include "StateMachine/00_STARTUP.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include <WiFi.h>

//* ************************************************************************
//* ************************** STARTUP STATE *******************************
//* ************************************************************************
// Function-based startup state handling.

//! ************************************************************************
//! STEP 1: TURN ON BLUE LED TO INDICATE STARTUP/HOMING
//! ************************************************************************

//! ************************************************************************
//! STEP 2: DISPLAY IP ADDRESS ON SERIAL MONITOR
//! ************************************************************************

//! ************************************************************************
//! STEP 3: TRANSITION TO HOMING STATE
//! ************************************************************************

void executeStartupState() {
    // Wait for WiFi to be ready
    if (WiFi.status() == WL_CONNECTED) {
        // WiFi is connected, transition to HOMING state
        changeState(HOMING);
    }
}

void onEnterStartupState() {
    // Startup state entered
    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
    }
    
    // WiFi connected, transition to HOMING
    changeState(HOMING);
}

void onExitStartupState() {
    // Startup state exited
} 