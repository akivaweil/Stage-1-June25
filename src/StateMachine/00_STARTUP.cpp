#include "StateMachine/00_STARTUP.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include <WiFi.h>

// STARTUP STATE
// Function-based startup state handling.

// STEP 1: TURN ON BLUE LED TO INDICATE STARTUP/HOMING

// STEP 2: TRANSITION TO HOMING STATE

void handleStartupState() {
    showBlueLed();  // Blue LED on during startup/homing

    // Small startup settle delay
    delay(1000);

    changeState(STATE_HOMING);
}

void onEnterStartupState() {
    // No specific entry actions for startup state
}

void onExitStartupState() {
    // No specific exit actions for startup state
} 