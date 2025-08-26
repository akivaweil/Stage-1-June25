#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include <math.h>

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE *****************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.
// 
// This state feeds wood forward in preparation for the first cut, using a 
// multi-pass approach due to mechanical constraints.
//
// Step-by-Step Process:
// 1. Initialization Phase
//    - Sets up variables and calculates required passes
//    - Target: Move wood forward by 5.0 inches total
//    - Constraint: Ball screw can only travel 4.0 inches max per pass
//    - Result: 2 passes needed with 0.1 inch overlap
//
// 2. Pass Execution Sequence (repeats for each pass)
//    Pass 1: Move wood from 0" to 3.9"
//    - Step 1: Retract feed clamp only
//    - Step 2: Move ball screw away from home (to 3.9" position)
//    - Step 3: Extend feed clamp, retract 2x4 secure clamp
//    - Step 4: Wait 300ms for clamp operations to complete
//    - Step 5: Move ball screw back to push wood forward
//
//    Pass 2: Move wood from 3.8" to 5.0"
//    - Same 5-step sequence but with different coordinates
//    - Wood continues from where Pass 1 left off
//
// 3. State Completion
//    - After all passes complete, checks the start cycle switch
//    - If HIGH: Transitions to CUTTING state and starts cutting cycle
//    - If LOW: Transitions to IDLE state

//* ************************************************************************
//* ******************** FEED STATE CONFIGURATION **************************
//* ************************************************************************
// Configuration variables for easy adjustment
const float TOTAL_WOOD_MOVEMENT = 5.0;        // Total distance wood moves forward (inches)
const float BALL_SCREW_TRAVEL = 4.0;         // Maximum ball screw travel per pass (inches)
const float PASS_OVERLAP = 0.1;              // Overlap between passes (inches)
const unsigned long CLAMP_DELAY_MS = 300;     // Delay after clamp operations (ms)

// Number of passes needed (calculated: (5.0 - 0.1) / (4.0 - 0.1) = 4.9 / 3.9 = 1.26, so 2 passes)
const int TOTAL_PASSES = 2;

//* ************************************************************************
//* ************************ STATE ENUMERATIONS ****************************
//* ************************************************************************

// Main state steps
enum FeedFirstCutStep {
    INITIALIZATION,
    EXECUTE_PASSES,
    COMPLETION_CHECK
};

// Individual pass steps
enum PassStep {
    RETRACT_FEED_CLAMP,
    MOVE_BALL_SCREW_AWAY,
    EXTEND_FEED_RETRACT_SECURE,
    WAIT_FOR_CLAMPS,
    MOVE_BALL_SCREW_BACK
};

//* ************************************************************************
//* ************************ STATIC VARIABLES ******************************
//* ************************************************************************

// Main state tracking
static FeedFirstCutStep currentStep = INITIALIZATION;
static unsigned long stepStartTime = 0;

// Pass execution tracking
static int currentPass = 0;
static PassStep currentPassStep = RETRACT_FEED_CLAMP;
static unsigned long passStepStartTime = 0;

// Position tracking for each pass
static float passStartPositions[TOTAL_PASSES];
static float passEndPositions[TOTAL_PASSES];

//* ************************************************************************
//* ************************ HELPER FUNCTION DECLARATIONS ******************
//* ************************************************************************

// Forward declarations for helper functions
void calculatePassPositions();
void handleInitializationStep();
void handleCompletionCheck();
void advanceToNextPassStep();

//* ************************************************************************
//* ************************ STATE FUNCTIONS *******************************
//* ************************************************************************

void executeFeedFirstCutState() {
    executeFeedFirstCutStep();
}

void onEnterFeedFirstCutState() {
    // Reset all state variables
    currentStep = INITIALIZATION;
    currentPass = 0;
    currentPassStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    passStepStartTime = 0;
    
    // Calculate pass positions
    calculatePassPositions();
}

void onExitFeedFirstCutState() {
    // Clean up state variables
    currentStep = INITIALIZATION;
    currentPass = 0;
    currentPassStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    passStepStartTime = 0;
}

//* ************************************************************************
//* ********************** MAIN STATE EXECUTION ****************************
//* ************************************************************************

void executeFeedFirstCutStep() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    switch (currentStep) {
        case INITIALIZATION:
            handleInitializationStep();
            break;
            
        case EXECUTE_PASSES:
            if (currentPass < TOTAL_PASSES) {
                executeFeedPass();
            } else {
                // All passes complete, move to completion check
                currentStep = COMPLETION_CHECK;
            }
            break;
            
        case COMPLETION_CHECK:
            handleCompletionCheck();
            break;
    }
}

//* ************************************************************************
//* ********************** INITIALIZATION STEP ****************************
//* ************************************************************************

void handleInitializationStep() {
    // Ensure feed motor is not running before starting
    FastAccelStepper* feedMotor = getFeedMotor();
    if (feedMotor && !feedMotor->isRunning()) {
        // Move feed motor to the starting position (0.0) for the first pass
        // This ensures we start from the correct position before beginning the multi-pass sequence
        moveFeedMotorToPosition(0.0);
        stepStartTime = millis();
    }
    
    // Wait for motor to reach starting position before proceeding
    if (feedMotor && !feedMotor->isRunning()) {
        currentStep = EXECUTE_PASSES;
        stepStartTime = millis();
    }
}

//* ************************************************************************
//* ********************** PASS EXECUTION *********************************
//* ************************************************************************

void executeFeedPass() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    switch (currentPassStep) {
        case RETRACT_FEED_CLAMP:
            retractFeedClamp();
            advanceToNextPassStep();
            break;
            
        case MOVE_BALL_SCREW_AWAY:
            if (feedMotor && !feedMotor->isRunning()) {
                float targetPosition = passEndPositions[currentPass];
                moveFeedMotorToPosition(targetPosition);
                advanceToNextPassStep();
            }
            break;
            
        case EXTEND_FEED_RETRACT_SECURE:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                retract2x4SecureClamp();
                passStepStartTime = millis();
                advanceToNextPassStep();
            }
            break;
            
        case WAIT_FOR_CLAMPS:
            if (millis() - passStepStartTime >= CLAMP_DELAY_MS) {
                advanceToNextPassStep();
            }
            break;
            
        case MOVE_BALL_SCREW_BACK:
            if (feedMotor && !feedMotor->isRunning()) {
                float targetPosition = passStartPositions[currentPass];
                moveFeedMotorToPosition(targetPosition);
                advanceToNextPassStep();
            }
            break;
    }
}

//* ************************************************************************
//* ********************** PASS STEP ADVANCEMENT **************************
//* ************************************************************************

void advanceToNextPassStep() {
    currentPassStep = static_cast<PassStep>(static_cast<int>(currentPassStep) + 1);
    passStepStartTime = 0;
    
    // If we've completed all pass steps, move to next pass
    if (currentPassStep > MOVE_BALL_SCREW_BACK) {
        currentPass++;
        currentPassStep = RETRACT_FEED_CLAMP;
    }
}

//* ************************************************************************
//* ********************** COMPLETION CHECK *******************************
//* ************************************************************************

void handleCompletionCheck() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    // Wait for feed motor to complete final movement
    if (feedMotor && !feedMotor->isRunning()) {
        // Check the start cycle switch state
        if (getStartCycleSwitch()->read() == HIGH) {
            changeState(CUTTING);
            setCuttingCycleInProgress(true);
            configureCutMotorForCutting();
            turnYellowLedOn();
            extendFeedClamp();
        } else {
            changeState(IDLE);
        }
    }
}

//* ************************************************************************
//* ********************** HELPER FUNCTIONS *******************************
//* ************************************************************************

void calculatePassPositions() {
    // Calculate start and end positions for each pass
    for (int i = 0; i < TOTAL_PASSES; i++) {
                if (i == 0) {
            // First pass: 0" to 3.9"
            // Note: Position 0.0 is the working zero (0.5" from home sensor)
            // The motor will move backward to reach this position if not already there
            passStartPositions[i] = 0.0;
            passEndPositions[i] = BALL_SCREW_TRAVEL - PASS_OVERLAP;
        } else {
            // Subsequent passes: continue from previous end position
            passStartPositions[i] = passEndPositions[i-1] - PASS_OVERLAP;
            passEndPositions[i] = passStartPositions[i] + BALL_SCREW_TRAVEL - PASS_OVERLAP;
        }
        
        // Ensure the last pass reaches the target distance
        if (i == TOTAL_PASSES - 1) {
            passEndPositions[i] = TOTAL_WOOD_MOVEMENT;
        }
    }
}

//* ************************************************************************
//* ********************** EXTERNAL INTERFACE *****************************
//* ************************************************************************

// Note: executeFeedFirstCutStep() and executeFeedPass() are implemented above
// and called from the StateManager as needed
