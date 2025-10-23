#include "StateMachine/02_IDLE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//* ************************************************************************
//* ************************** IDLE STATE **********************************
//* ************************************************************************
// Handles the idle state, awaiting user input or automatic cycle start.
// CRITICAL: Maintains secure wood clamp extended in normal idle - ONLY retracts in reload mode.
// Maintains feed clamp retracted.
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

// LED blinking variables for gentle blue blink pattern when no wood detected
static unsigned long idleLastLedChangeTime = 0;
static bool idleLedState = true; // Start with LED on

void executeIdleState() {
    //! Handle gentle blue LED blinking if coming from no-wood situation
    if (getComingFromNoWoodWithSensorsClear()) {
        unsigned long currentTime = millis();
        unsigned long timeSinceLastChange = currentTime - idleLastLedChangeTime;
        
        if (idleLedState && timeSinceLastChange >= LED_BLINK_ON_DURATION_MS) {
            // Been on for configured duration, turn off
            turnBlueLedOff();
            idleLedState = false;
            idleLastLedChangeTime = currentTime;
        } else if (!idleLedState && timeSinceLastChange >= LED_BLINK_OFF_DURATION_MS) {
            // Been off for configured duration, turn on
            turnOnlyBlueLedOn();
            idleLedState = true;
            idleLastLedChangeTime = currentTime;
        } else if (idleLedState && idleLastLedChangeTime == 0) {
            // Initialize on first call
            turnOnlyBlueLedOn();
            idleLastLedChangeTime = currentTime;
        }
    }
    
    // Check if reload switch is activated - if so, transition to reload state
    bool reloadSwitchOn = getReloadSwitch()->read() == HIGH;
    if (reloadSwitchOn && !getIsReloadMode()) {
        changeState(RELOAD);
        return;
    }

    // Check for FeedFirstCut conditions if not in reload mode
    if (!getIsReloadMode()) {
        checkFirstCutConditions();
        checkStartConditions();
    }
}

void onEnterIdleState() {
    // Cycle counter is now incremented in RETURNING states to handle continuous mode properly

    // Check if coming from no2x4 with no wood detected - if so, keep secure clamp extended
    bool comingFromNoWood = getComingFromNoWoodWithSensorsClear();
    bool isReloadMode = getIsReloadMode();

    // Handle 2x4 secure clamp based on reload mode and no-wood status
    if (isReloadMode) {
        // In reload mode - retract 2x4 secure clamp for safety
        retract2x4SecureClamp();
    } else if (comingFromNoWood) {
        // Coming from no2x4 with no wood - keep secure clamp extended
        // Don't retract the secure clamp, it should stay extended
        // Initialize LED blinking pattern
        turnOnlyBlueLedOn();
        idleLastLedChangeTime = millis();
        idleLedState = true;
    } else {
        // Normal case - extend secure clamp (only retracted in reload mode)
        extend2x4SecureClamp();
    }

    // Always retract other clamps (but NOT the secure 2x4 clamp in normal idle)
    retractFeedClamp();
    retractRotationClamp();

    // NOTE: Do NOT reset the no-wood flag here - keep it set so blinking continues
    // It will be reset when starting a new cycle or pressing feed button

    //serial.println("Idle: All clamps retracted");
}

void onExitIdleState() {
    // No specific cleanup needed when exiting IDLE state
}


void checkFirstCutConditions() {
    // Check for pushwood forward switch press and FIRST_CUT_OR_WOOD_FWD_ONE sensor state
    extern Bounce pushwoodForwardSwitch;
    extern const int FIRST_CUT_OR_WOOD_FWD_ONE;
    bool pushwoodPressed = pushwoodForwardSwitch.rose();
    bool firstCutSensorHigh = (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == HIGH);
    bool firstCutSensorLow = (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == LOW);
    
    if (pushwoodPressed && firstCutSensorHigh) {
        //serial.println("Idle: Manual feed switch pressed with FIRST_CUT_OR_WOOD_FWD_ONE sensor HIGH - transitioning to FEED_FIRST_CUT");
        // Reset the no-wood flag when feed button is pressed
        setComingFromNoWoodWithSensorsClear(false);
        changeState(FEED_FIRST_CUT);
    }
    else if (pushwoodPressed && firstCutSensorLow) {
        //serial.println("Idle: Manual feed switch pressed with FIRST_CUT_OR_WOOD_FWD_ONE sensor LOW - transitioning to FEED_WOOD_FWD_ONE");
        // Reset the no-wood flag when feed button is pressed
        setComingFromNoWoodWithSensorsClear(false);
        changeState(FEED_WOOD_FWD_ONE);
    }
}

void checkStartConditions() {
    // Only turn on green LED if not coming from no-wood situation
    if (!getComingFromNoWoodWithSensorsClear()) {
        turnOnlyGreenLedOn();
    }
    
    // Sync continuous mode flag with actual switch state to prevent race conditions
    bool startSwitchOn = getStartCycleSwitch()->read() == HIGH;
    bool startSwitchSafe = getStartSwitchSafe();
    if (startSwitchOn != getContinuousModeActive() && startSwitchSafe) {
        setContinuousModeActive(startSwitchOn);
    }
    
    bool startCycleRose = getStartCycleSwitch()->rose();
    bool continuousModeActive = getContinuousModeActive();
    bool cuttingCycleInProgress = getCuttingCycleInProgress();
    bool woodSuctionError = getWoodSuctionError();
    bool _2x4Present = get2x4Present();
    
    // For continuous mode, also verify switch is still HIGH to prevent false triggers
    bool shouldStart = false;
    if (startCycleRose) {
        shouldStart = true; // Manual press
    } else if (continuousModeActive && !cuttingCycleInProgress && startSwitchOn) {
        shouldStart = true; // Continuous mode with switch still ON
    }
    
    if (shouldStart && !woodSuctionError && startSwitchSafe) {
        
        // Don't start new cycle if coming from no-wood situation - require manual reset
        if (getComingFromNoWoodWithSensorsClear()) {
            return; // Exit without starting cycle
        }
        
        turnGreenLedOff();
        turnOnlyYellowLedOn();
        turnBlueLedOff();
        
        // Reset the no-wood flag when starting a new cutting cycle
        setComingFromNoWoodWithSensorsClear(false);
        
        setCuttingCycleInProgress(true);
        changeState(CUTTING);
        configureCutMotorForCutting();
        
        extendFeedClamp();
        extend2x4SecureClamp();
        
        // LED status will be handled by cutting state based on wood sensor
    }
} 