#include "StateMachine/02_IDLE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** IDLE STATE **********************************
//* ************************************************************************
// Handles the idle state, awaiting user input or automatic cycle start.
// Maintains secure wood clamp extended and feed clamp retracted.
// Checks for pushwood forward switch press to transition to FeedFirstCut state.

//! ************************************************************************
//! STEP 1: TURN ON GREEN LED TO INDICATE SYSTEM IS IDLE
//! ************************************************************************

//! ************************************************************************
//! STEP 2: CHECK FOR PUSHWOOD FORWARD SWITCH PRESS AND SENSOR STATE
//! ************************************************************************
// AND Reload mode is not active
// AND Start cycle switch safety is not active  
// AND Wood suction error is not present

//! ************************************************************************
//! STEP 3: CHECK MANUAL FEED BUTTON PRESS AND FIRST_CUT_OR_WOOD_FWD_ONE SENSOR
//! ************************************************************************
// If HIGH, transition to FeedFirstCut state
// If LOW, transition to FeedWoodFwdOne state

//! ************************************************************************
//! STEP 4: CHECK FOR START CYCLE CONDITIONS
//! ************************************************************************
// Start switch just flipped ON (rising edge)
// OR Continuous mode active AND not already in a cutting cycle
// AND Wood suction error is not present
// AND Start switch is safe to use

//! ************************************************************************
//! STEP 5: IF START CONDITIONS MET - TRANSITION TO CUTTING
//! ************************************************************************
// Turn off green LED, turn on yellow LED
// Set cuttingCycleInProgress flag to true
// Transition to CUTTING state
// Configure cut motor for cutting speed
// Ensure position and wood secure clamps are engaged
// If no wood detected, turn on blue LED for NO_WOOD mode

void executeIdleState() {
    // Handle reload mode logic first
    handleReloadModeLogic();
    
    // Check for FeedFirstCut conditions if not in reload mode
    if (!getIsReloadMode()) {
        checkFirstCutConditions();
        checkStartConditions();
    }
}

void onEnterIdleState() {
    // Maintain clamp states: secure wood clamp extended, feed clamp retracted, rotation clamp retracted
    extend2x4SecureClamp();
    retractFeedClamp();
    retractRotationClamp(); // Ensure rotation clamp is retracted in IDLE state
    
}

void onExitIdleState() {
    // No specific cleanup needed when exiting IDLE state
}

void handleReloadModeLogic() {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool reloadSwitchOn = getReloadSwitch()->read() == HIGH;
    bool isReloadMode = getIsReloadMode();
    
    if (reloadSwitchOn && !isReloadMode) {
        // Enter reload mode
        setIsReloadMode(true);
        retractFeedClamp(); // Retract feed clamp
        retract2x4SecureClamp(); // Retract 2x4 secure clamp
        turnBlueLedOn();     // Turn on blue LED for reload mode
    } else if (!reloadSwitchOn && isReloadMode) {
        // Exit reload mode
        setIsReloadMode(false);
        extend2x4SecureClamp(); // Re-extend 2x4 secure clamp
        retractFeedClamp();   // Keep feed clamp retracted (idle state default)
        turnBlueLedOff();       // Turn off blue LED
    }
}

void checkFirstCutConditions() {
    // Check for pushwood forward switch press and FIRST_CUT_OR_WOOD_FWD_ONE sensor state
    extern Bounce pushwoodForwardSwitch;
    extern const int FIRST_CUT_OR_WOOD_FWD_ONE;
    bool pushwoodPressed = pushwoodForwardSwitch.rose();
    bool firstCutSensorHigh = (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == HIGH);
    bool firstCutSensorLow = (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == LOW);
    
    if (pushwoodPressed && firstCutSensorHigh) {
        
        changeState(FEED_FIRST_CUT);
    }
    else if (pushwoodPressed && firstCutSensorLow) {
        
        changeState(FEED_WOOD_FWD_ONE);
    }
}

void checkStartConditions() {
    turnGreenLedOn();
    
    bool startCycleRose = getStartCycleSwitch()->rose();
    bool continuousModeActive = getContinuousModeActive();
    bool cuttingCycleInProgress = getCuttingCycleInProgress();
    bool woodSuctionError = getWoodSuctionError();
    bool startSwitchSafe = getStartSwitchSafe();
    bool _2x4Present = get2x4Present();
    
    if (((startCycleRose || (continuousModeActive && !cuttingCycleInProgress)) 
        && !woodSuctionError) && startSwitchSafe) {
        turnGreenLedOff();
        turnYellowLedOn();
        turnBlueLedOff();
        
        setCuttingCycleInProgress(true);
        changeState(CUTTING);
        configureCutMotorForCutting();
        
        extendFeedClamp();
        extend2x4SecureClamp();
        
        if (!_2x4Present) {
            turnBlueLedOn();
        }
    }
} 