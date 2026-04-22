#include "StateMachine/02_IDLE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "WebSocketDashboard/websocket_dashboard.h"

const unsigned long IDLE_WOOD_PRESENT_ACTIVE_DELAY_MS = 1500;
static bool idleWoodPresentPreviousState = LOW;
static bool idleWoodPresentDelayActive = false;
static unsigned long idleWoodPresentDelayStartMs = 0;

static bool checkWoodPresentActiveAutoFeedFirstCut() {
    bool woodPresentNow = getWoodPresentSensorBounce()->read();
    bool woodPresentBecameActive = (idleWoodPresentPreviousState == HIGH && woodPresentNow == LOW);
    extern const int FIRST_CUT_OR_WOOD_FWD_ONE;
    bool firstCutOrWoodFwdOneActive = (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == LOW);

    if (woodPresentBecameActive) {
        idleWoodPresentDelayActive = true;
        idleWoodPresentDelayStartMs = millis();
    }

    if (idleWoodPresentDelayActive) {
        // Cancel pending auto-feed if wood-present becomes inactive during the delay.
        if (woodPresentNow == HIGH) {
            idleWoodPresentDelayActive = false;
        } else if (firstCutOrWoodFwdOneActive) {
            idleWoodPresentDelayActive = false;
        } else if ((millis() - idleWoodPresentDelayStartMs) >= IDLE_WOOD_PRESENT_ACTIVE_DELAY_MS) {
            idleWoodPresentDelayActive = false;
            setComingFromNoWoodWithSensorsClear(false);
            changeState(FEED_FIRST_CUT);
            idleWoodPresentPreviousState = woodPresentNow;
            return true;
        }
    }

    idleWoodPresentPreviousState = woodPresentNow;
    return false;
}

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 😴 IDLE STATE                                                       ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Handles the idle state, awaiting user input or automatic cycle start.
// CRITICAL: Maintains secure wood clamp extended in normal idle.
// Maintains feed clamp retracted.
// Checks for pushwood forward switch press to transition to FeedFirstCut state.

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 1: TURN ON GREEN LED TO INDICATE SYSTEM IS IDLE               ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 2: CHECK FOR PUSHWOOD FORWARD SWITCH PRESS AND SENSOR STATE    ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// AND Start cycle switch safety is not active  
// AND Wood suction error is not present

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 3: CHECK MANUAL FEED BUTTON PRESS AND FIRST_CUT_OR_WOOD_FWD_ONE ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// If HIGH, transition to FeedFirstCut state
// If LOW, transition to FeedWoodFwdOne state

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 4: CHECK FOR START CYCLE CONDITIONS                           ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Start switch just flipped ON (rising edge)
// OR Continuous mode active AND not already in a cutting cycle
// AND Wood suction error is not present
// AND Start switch is safe to use

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 5: IF START CONDITIONS MET - TRANSITION TO CUTTING            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Turn off green LED, turn on yellow LED
// Set cuttingCycleInProgress flag to true
// Transition to CUTTING state
// Configure cut motor for cutting speed
// Ensure position and wood secure clamps are engaged
// If no wood detected, turn on blue LED for NO_WOOD mode

void executeIdleState() {
    // Check if reload switch is activated - if so, transition to reload state
    if (getReloadSwitch()->read() == HIGH) {
        changeState(RELOAD);
        return;
    }

    checkFirstCutConditions();
    if (checkWoodPresentActiveAutoFeedFirstCut()) {
        return;
    }
    checkStartConditions();
}

void onEnterIdleState() {
    idleWoodPresentPreviousState = getWoodPresentSensorBounce()->read();
    idleWoodPresentDelayActive = false;
    idleWoodPresentDelayStartMs = 0;

    // Cycle counter is now incremented in RETURNING states to handle continuous mode properly

    // Check if coming from no2x4 with no wood detected - if so, keep secure clamp extended
    bool comingFromNoWood = getComingFromNoWoodWithSensorsClear();

    // Handle 2x4 secure clamp logic
    if (comingFromNoWood) {
        // Coming from no2x4 with no wood - keep secure clamp extended
        // Don't retract the secure clamp, it should stay extended
    } else {
        // Normal case - extend secure clamp
        extend2x4SecureClamp();
    }

    // Always retract other clamps
    retractFeedClamp();
    retractRotationClamp();

    // Reset the no-wood flag if it was set
    if (comingFromNoWood) {
        setComingFromNoWoodWithSensorsClear(false);
    }

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
    showGreenLed();
    
    bool startCycleRose = getStartCycleSwitch()->rose();
    
    // Check if start condition is met (either physical switch or dashboard trigger)
    bool startTriggered = startCycleRose || (dashboardStartCycleTrigger && !cuttingCycleInProgress);
    
    bool continuousModeActive = getContinuousModeActive();
    bool woodSuctionError = getWoodSuctionError();
    bool startSwitchSafe = getStartSwitchSafe();
    bool _2x4Present = get2x4Present();
    
    if (((startTriggered || (continuousModeActive && !cuttingCycleInProgress)) 
        && !woodSuctionError) && startSwitchSafe) {
        
        // Reset dashboard trigger once consumed
        dashboardStartCycleTrigger = false;
        
        // Don't start new cycle if coming from no-wood situation - require manual reset
        if (getComingFromNoWoodWithSensorsClear()) {
            return; // Exit without starting cycle
        }
        
        turnGreenLedOff();
        showYellowLed();
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