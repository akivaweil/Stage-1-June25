#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pin_Def.h"
#include "Config/config.h"
#include "WebSocketDashboard/websocket_dashboard.h"

// State-specific constants
const float FEED_MOTOR_SPEED_MULTIPLIER = 0.6; // Speed reduction for NO_2x4 returning sequence
const float FEED_MOTOR_2ND_POSITION = -1.2; // Position for 2nd position movement
const float FEED_MOTOR_HOME_POSITION = 0.8; // Home position
const float FEED_MOTOR_FINAL_POSITION = -1.2; // Final position

// Step enumeration for better readability
enum ReturningNo2x4Step {
    STEP_INITIALIZE = 0,
    STEP_WAIT_CUT_MOTOR_EXTEND_FEED_CLAMP = 1,
    STEP_MOVE_FEED_MOTOR_TO_2_INCHES = 2,
    STEP_WAIT_FEED_MOTOR_AT_2_INCHES_EXTEND_CLAMP = 3,
    STEP_MOVE_FEED_MOTOR_TO_HOME = 4,
    STEP_WAIT_FEED_MOTOR_HOME_RETRACT_CLAMP = 5,
    STEP_MOVE_FEED_MOTOR_TO_FINAL_POSITION = 6,
    STEP_WAIT_FEED_MOTOR_FINAL_EXTEND_CLAMP = 7,
    STEP_FINAL_COMPLETION = 8
};

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ❌ RETURNING NO 2X4 STATE                                           ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.
// 
// REFACTORED: 2025-01-XX - Major refactoring to reduce if-else complexity:
// - Added step enumeration for better readability and maintainability
// - Extracted common patterns into reusable helper functions
// - Consolidated error handling into dedicated functions
// - Improved code organization and reduced duplication

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 1: INITIALIZE SEQUENCE - MOVE CUT MOTOR HOME AND RETRACT 2X4 CLAMP ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 2: WAIT FOR CUT MOTOR HOME AND EXTEND FEED CLAMP               ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 3: MOVE FEED MOTOR TO -1 (NEGATIVE DIRECTION - EXTEND CLAMP)   ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 4: WAIT FOR FEED MOTOR AT -1 AND EXTEND FEED CLAMP            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 5: MOVE FEED MOTOR TO 3.4 (POSITIVE DIRECTION - RETRACT CLAMP) ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 6: WAIT FOR FEED MOTOR AT 3.4 AND RETRACT FEED CLAMP          ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 7: MOVE FEED MOTOR TO -1 AGAIN (NEGATIVE DIRECTION - EXTEND CLAMP) ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 8: WAIT FOR FEED MOTOR AT -1 AND EXTEND FEED CLAMP            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

// Static variables for returning no 2x4 state tracking
static int returningNo2x4Step = 0;
static unsigned long cylinderActionTime = 0;
static bool waitingForCylinder = false;

// Static variables for LED wave pattern
static unsigned long ledOnTime[4] = {0, 0, 0, 0}; // When each LED turned on (0 = off)
static unsigned long ledTurnOnTime[4] = {0, 0, 0, 0}; // When each LED should next turn on
const unsigned long LED_WAVE_INTERVAL = 170; // 170ms between each LED turning on (15% faster)
const unsigned long LED_ON_DURATION = 510; // Each LED stays on for 510ms (15% faster)


void executeReturningNo2x4State() {
    handleReturningNo2x4Sequence(); 
}

void onEnterReturningNo2x4State() {
    //! ************************************************************************
    //! STEP 1: INITIALIZE RETURNING NO 2X4 SEQUENCE
    //! ************************************************************************
    
    // Reset consecutive yeswood counter when nowood state occurs
    resetConsecutiveYeswoodCount();
    
    // Initialize RETURNING_NO_2x4 sequence from CUTTING_state logic
    configureCutMotorForReturn();
    // Cut motor already started in CUTTING state
    configureFeedMotorForNormalOperation();

    // Turn off all LEDs initially
    digitalWrite(STATUS_LED_RED, LOW);
    digitalWrite(STATUS_LED_YELLOW, LOW);
    digitalWrite(STATUS_LED_GREEN, LOW);
    digitalWrite(STATUS_LED_BLUE, LOW);
    
    // Initialize step tracking
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
    
    // Initialize LED wave timestamps (start each LED progressively)
    unsigned long now = millis();
    ledTurnOnTime[0] = now; // Red starts immediately
    ledTurnOnTime[1] = now + LED_WAVE_INTERVAL; // Yellow starts after red
    ledTurnOnTime[2] = now + (LED_WAVE_INTERVAL * 2); // Green starts after yellow
    ledTurnOnTime[3] = now + (LED_WAVE_INTERVAL * 3); // Blue starts after green
    ledOnTime[0] = 0;
    ledOnTime[1] = 0;
    ledOnTime[2] = 0;
    ledOnTime[3] = 0;
}

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

void handleReturningNo2x4Sequence() {
    // RETURNING_NO_2x4 sequence logic
    FastAccelStepper* feedMotor = getFeedMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    // Handle LED wave pattern: overlapping red -> yellow -> green -> blue
    unsigned long now = millis();
    for (int i = 0; i < 4; i++) {
        // Check if LED should turn on
        if (now >= ledTurnOnTime[i] && ledOnTime[i] == 0) {
            // Turn on this LED
            switch (i) {
                case 0: digitalWrite(STATUS_LED_RED, HIGH); break;
                case 1: digitalWrite(STATUS_LED_YELLOW, HIGH); break;
                case 2: digitalWrite(STATUS_LED_GREEN, HIGH); break;
                case 3: digitalWrite(STATUS_LED_BLUE, HIGH); break;
            }
            // Record when this LED turned on
            ledOnTime[i] = now;
            // Schedule next turn on (continuous wave)
            ledTurnOnTime[i] = now + LED_ON_DURATION + (LED_WAVE_INTERVAL * 3);
        }
        
        // Check if LED should turn off (after being on for LED_ON_DURATION)
        if (ledOnTime[i] > 0 && now - ledOnTime[i] >= LED_ON_DURATION) {
            // Turn off this LED
            switch (i) {
                case 0: digitalWrite(STATUS_LED_RED, LOW); break;
                case 1: digitalWrite(STATUS_LED_YELLOW, LOW); break;
                case 2: digitalWrite(STATUS_LED_GREEN, LOW); break;
                case 3: digitalWrite(STATUS_LED_BLUE, LOW); break;
            }
            // Reset on time (waiting for next turn on)
            ledOnTime[i] = 0;
        }
    }
    
    if (returningNo2x4Step == STEP_INITIALIZE) { // First time entering this specific RETURNING_NO_2x4 logic path
        retract2x4SecureClamp();
        returningNo2x4Step = STEP_WAIT_CUT_MOTOR_EXTEND_FEED_CLAMP;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        returningNo2x4Step++; 
    }
    
    if (!waitingForCylinder) {
        handleReturningNo2x4Step(returningNo2x4Step);
    }
}

void handleReturningNo2x4Step(int step) {
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case STEP_WAIT_CUT_MOTOR_EXTEND_FEED_CLAMP: // Wait for cut motor, then extend feed clamp
            handleWaitForMotorAndCylinderAction(cutMotor, true); // true = extend
            break;
            
        case STEP_MOVE_FEED_MOTOR_TO_2_INCHES: // Move feed motor to -1 (negative direction - extend clamp)
            configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER); // Use slow config for large position changes
            extendFeedClamp(); // Extend clamp for negative direction movement
            delay(5);
            moveFeedMotorToPosition(FEED_MOTOR_2ND_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_MOTOR_AT_2_INCHES_EXTEND_CLAMP; // Directly advance step here as it's a command
            break;
            
        case STEP_WAIT_FEED_MOTOR_AT_2_INCHES_EXTEND_CLAMP: // Wait for feed motor at -1, ensure clamp extended
            handleWaitForFeedMotorAndExtendClamp();
            break;
            
        case STEP_MOVE_FEED_MOTOR_TO_HOME: // Move feed motor to 3.4 (positive direction - retract clamp)
            configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER);
            retractFeedClamp(); // Retract clamp for positive direction movement
            delay(5);
            moveFeedMotorToPosition(FEED_MOTOR_HOME_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_MOTOR_HOME_RETRACT_CLAMP; // Directly advance step
            break;
            
        case STEP_WAIT_FEED_MOTOR_HOME_RETRACT_CLAMP: // Wait for feed motor at 3.4, ensure clamp retracted
            handleWaitForMotorAndCylinderAction(feedMotor, false); // false = retract
            break;
            
        case STEP_MOVE_FEED_MOTOR_TO_FINAL_POSITION: // Move feed motor to 0 again (negative direction - extend clamp)
            configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER);
            extendFeedClamp(); // Extend clamp for negative direction movement
            delay(5);
            moveFeedMotorToPosition(FEED_MOTOR_FINAL_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_MOTOR_FINAL_EXTEND_CLAMP; // Directly advance to wait step
            break;
            
        case STEP_WAIT_FEED_MOTOR_FINAL_EXTEND_CLAMP: // Wait for feed motor at -1, ensure clamp extended
            handleWaitForMotorAndCylinderAction(feedMotor, true); // true = extend
            startReloadTimer(); // Start reload time tracking when reaching final step
            break;
            
        case STEP_FINAL_COMPLETION: // Final step: wait for sensor to clear, then extend secure clamp
            if (feedMotor && !feedMotor->isRunning()) {
                // Retract feed clamp before checking sensor
                retractFeedClamp();
                delay(5);

                // Wait for 2x4 present sensor to be not active (HIGH) before extending clamp
                extern const int _2x4_PRESENT_SENSOR;
                if (getWoodPresentSensorBounce()->read() == HIGH) {
                    // Sensor is clear (not active) - safe to extend secure clamp
                    delay(1000);
                    extend2x4SecureClamp();
                    // Set flag to prevent IDLE from retracting the clamp
                    setComingFromNoWoodWithSensorsClear(true);
                    
                    // Complete sequence and transition to IDLE
                    resetReturningNo2x4Steps();
                    incrementCuttingCycleCounter();
                    setCuttingCycleInProgress(false);
                    
                    
                    // When no wood is detected, require manual reset of cycle switch
                    // This prevents automatic restart when no wood is present
                    if (getStartCycleSwitch()->read() == HIGH) {
                        setStartSwitchSafe(false);
                    }
                    
                    changeState(IDLE);
                }
                // If sensor is still active (LOW), wait here (non-blocking)
            }
            break;
    }
}



//* ************************************************************************
//* ****************** HELPER FUNCTIONS FOR STEP HANDLING ******************
//* ************************************************************************


// Generic function to wait for a motor to stop and then perform a cylinder action
void handleWaitForMotorAndCylinderAction(FastAccelStepper* motor, bool extendClamp) {
    if (motor && !motor->isRunning()) {
        if (extendClamp) {
            extendFeedClamp();
        } else {
            retractFeedClamp();
        }
        cylinderActionTime = millis();
        waitingForCylinder = true; // Will cause step increment after delay
    }
}

// Specific function for waiting for feed motor and extending clamp at -1
void handleWaitForFeedMotorAndExtendClamp() {
    FastAccelStepper* feedMotor = getFeedMotor();
    if (feedMotor && !feedMotor->isRunning()) {
        extendFeedClamp();
        //serial.println("ReturningNo2x4: Feed clamp extended at -1");
        returningNo2x4Step = STEP_MOVE_FEED_MOTOR_TO_HOME; // Move to next step
    }
}

void resetReturningNo2x4Steps() {
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 