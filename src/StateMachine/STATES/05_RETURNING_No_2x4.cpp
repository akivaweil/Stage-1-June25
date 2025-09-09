#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

// Timing constants for this state
const unsigned long ATTENTION_SEQUENCE_DELAY_MS = 50; // Delay between feed clamp movements in attention sequence
const int ATTENTION_SEQUENCE_MOVEMENTS = 9; // Total number of movements in attention sequence

// Feed motor position constants
const float FEED_MOTOR_2ND_POSITION = -1.2; // Position for 2nd position movement
const float FEED_MOTOR_HOME_POSITION = 3.4; // Home position (3.4)
const float FEED_MOTOR_FINAL_POSITION = -1.2; // Final position

// Step enumeration for better readability
enum ReturningNo2x4Step {
    STEP_INITIALIZE = 0,
    STEP_WAIT_CUT_MOTOR_EXTEND_FEED_CLAMP = 1,
    STEP_MOVE_FEED_MOTOR_TO_2_INCHES = 2,
    STEP_WAIT_FEED_MOTOR_AT_2_INCHES_EXTEND_CLAMP = 3,
    STEP_ATTENTION_SEQUENCE = 4,
    STEP_MOVE_FEED_MOTOR_TO_HOME = 5,
    STEP_WAIT_FEED_MOTOR_HOME_RETRACT_CLAMP = 6,
    STEP_MOVE_FEED_MOTOR_TO_FINAL_POSITION = 7,
    STEP_WAIT_FEED_MOTOR_FINAL_EXTEND_CLAMP = 8,
    STEP_FINAL_COMPLETION = 9
};

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.
// 
// REFACTORED: 2025-01-XX - Major refactoring to reduce if-else complexity:
// - Replaced 9 consecutive if-else statements in attention sequence with a loop
// - Added step enumeration for better readability and maintainability
// - Extracted common patterns into reusable helper functions
// - Consolidated error handling into dedicated functions
// - Improved code organization and reduced duplication

//! ************************************************************************
//! STEP 1: INITIALIZE SEQUENCE - MOVE CUT MOTOR HOME AND RETRACT 2X4 CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 2: WAIT FOR CUT MOTOR HOME AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 3: MOVE FEED MOTOR TO -1 (NEGATIVE DIRECTION - EXTEND CLAMP)
//! ************************************************************************

//! ************************************************************************
//! STEP 4: WAIT FOR FEED MOTOR AT -1 AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 5: ATTENTION GETTING SEQUENCE - INTENSE FEED CLAMP EXTENSION/RETRACTION (9 MOVEMENTS)
//! ************************************************************************

//! ************************************************************************
//! STEP 6: MOVE FEED MOTOR TO 3.4 (POSITIVE DIRECTION - RETRACT CLAMP)
//! ************************************************************************

//! ************************************************************************
//! STEP 7: WAIT FOR FEED MOTOR AT 3.4 AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 8: MOVE FEED MOTOR TO -1 AGAIN (NEGATIVE DIRECTION - EXTEND CLAMP)
//! ************************************************************************

//! ************************************************************************
//! STEP 9: WAIT FOR FEED MOTOR AT -1 AND EXTEND FEED CLAMP
//! ************************************************************************

// Static variables for returning no 2x4 state tracking
static int returningNo2x4Step = 0;
static unsigned long cylinderActionTime = 0;
static bool waitingForCylinder = false;


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
    moveCutMotorToHome();
    configureFeedMotorForNormalOperation();

    turnBlueLedOn();
    turnYellowLedOff();
    
    // Initialize step tracking
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
}

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

void handleReturningNo2x4Sequence() {
    // RETURNING_NO_2x4 sequence logic
    FastAccelStepper* feedMotor = getFeedMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
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
            configureFeedMotorForNormalOperation(); // Ensure correct config
            extendFeedClamp(); // Extend clamp for negative direction movement
            delay(50); // 50ms delay between cylinder extension and feed motor movement
            moveFeedMotorToPosition(FEED_MOTOR_2ND_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_MOTOR_AT_2_INCHES_EXTEND_CLAMP; // Directly advance step here as it's a command
            break;
            
        case STEP_WAIT_FEED_MOTOR_AT_2_INCHES_EXTEND_CLAMP: // Wait for feed motor at -1, ensure clamp extended
            handleWaitForFeedMotorAndExtendClamp();
            break;
            
        case STEP_ATTENTION_SEQUENCE: // Attention-getting sequence: 9 movements total
            handleAttentionSequence();
            break;
            
        case STEP_MOVE_FEED_MOTOR_TO_HOME: // Move feed motor to 3.4 (positive direction - retract clamp)
            configureFeedMotorForNormalOperation();
            retractFeedClamp(); // Retract clamp for positive direction movement
            delay(50); // 50ms delay between cylinder retraction and feed motor movement
            moveFeedMotorToPosition(FEED_MOTOR_HOME_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_MOTOR_HOME_RETRACT_CLAMP; // Directly advance step
            break;
            
        case STEP_WAIT_FEED_MOTOR_HOME_RETRACT_CLAMP: // Wait for feed motor at 3.4, ensure clamp retracted
            handleWaitForMotorAndCylinderAction(feedMotor, false); // false = retract
            break;
            
        case STEP_MOVE_FEED_MOTOR_TO_FINAL_POSITION: // Move feed motor to 0 again (negative direction - extend clamp)
            configureFeedMotorForNormalOperation();
            extendFeedClamp(); // Extend clamp for negative direction movement
            delay(50); // 50ms delay between cylinder extension and feed motor movement
            moveFeedMotorToPosition(FEED_MOTOR_FINAL_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_MOTOR_FINAL_EXTEND_CLAMP; // Directly advance to wait step
            break;
            
        case STEP_WAIT_FEED_MOTOR_FINAL_EXTEND_CLAMP: // Wait for feed motor at -1, ensure clamp extended
            handleWaitForMotorAndCylinderAction(feedMotor, true); // true = extend
            break;
            
        case STEP_FINAL_COMPLETION: // Final step: check wood present sensor and extend secure clamp if not active
            if (feedMotor && !feedMotor->isRunning()) {
                // Check if wood present sensor is not active
                if (digitalRead(_2x4_PRESENT_SENSOR) == HIGH) {
                    // Wood present sensor not active - extend secure wood clamp
                    extend2x4SecureClamp();
                    
                    // Complete sequence and transition to IDLE
                    resetReturningNo2x4Steps();
                    setCuttingCycleInProgress(false);
                    changeState(IDLE);
                }
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
        returningNo2x4Step = STEP_ATTENTION_SEQUENCE; // Move to attention sequence
    }
}

//* ************************************************************************
//* ****************** ATTENTION SEQUENCE HANDLER **************************
//* ************************************************************************
// Handles the attention-getting sequence with 9 movements

void handleAttentionSequence() {
    static int attentionStep = 0;
    static unsigned long attentionStartTime = 0;
    
    // Check if we need to wait for the delay
    if (attentionStep > 0 && millis() - attentionStartTime < ATTENTION_SEQUENCE_DELAY_MS) {
        return; // Still waiting
    }
    
    // Execute the current movement
    if (attentionStep < ATTENTION_SEQUENCE_MOVEMENTS) {
        // Alternate between retract and extend, starting with retract
        if (attentionStep % 2 == 0) {
            retractFeedClamp();
        } else {
            extendFeedClamp();
        }
        
        //serial.println("ReturningNo2x4: Attention sequence - " + 
        //    (attentionStep % 2 == 0 ? "retracting" : "extending") + 
        //    " feed clamp (" + String(attentionStep + 1) + "/" + String(ATTENTION_SEQUENCE_MOVEMENTS) + ")");
        
        attentionStep++;
        attentionStartTime = millis();
    } else {
        // Sequence complete - ensure clamp is extended and move to next step
        extendFeedClamp();
        //serial.println("ReturningNo2x4: Attention sequence - final extension (" + String(ATTENTION_SEQUENCE_MOVEMENTS) + "/" + String(ATTENTION_SEQUENCE_MOVEMENTS) + ")");
        
        attentionStep = 0; // Reset for next time
        returningNo2x4Step = STEP_MOVE_FEED_MOTOR_TO_HOME;
    }
}

void resetReturningNo2x4Steps() {
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 