#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include <math.h>

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE **************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

// Configuration variables - easily adjustable
const float TOTAL_WOOD_MOVEMENT = 5.0;        // Total distance wood actually moves forward (inches)
const float BALL_SCREW_TRAVEL = 4.0;         // Maximum ball screw travel (inches)
const float PASS_OVERLAP = 0.1;              // Overlap between passes (inches)
const unsigned long CLAMP_DELAY_MS = 300;     // Delay after clamp operations (ms)

// Calculate number of passes needed
const int TOTAL_PASSES = ceil((TOTAL_WOOD_MOVEMENT + PASS_OVERLAP) / (BALL_SCREW_TRAVEL - PASS_OVERLAP));

// State tracking
enum FeedFirstCutStep {
    INIT,
    FEED_PASS,
    COMPLETE
};

static FeedFirstCutStep currentStep = INIT;
static int currentPass = 0;
static unsigned long stepStartTime = 0;
static bool isFirstPass = true;

void executeFeedFirstCutState() {
    executeFeedFirstCutStep();
}

void onEnterFeedFirstCutState() {
    currentStep = INIT;
    currentPass = 0;
    stepStartTime = 0;
    isFirstPass = true;
    //serial.println("FeedFirstCut: Starting feed first cut sequence");
}

void onExitFeedFirstCutState() {
    currentStep = INIT;
    currentPass = 0;
    stepStartTime = 0;
    isFirstPass = false;
    //serial.println("FeedFirstCut: Sequence complete");
}

void executeFeedFirstCutStep() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    if (!feedMotor) return;

    switch (currentStep) {
        case INIT:
            // Start first pass
            currentPass = 1;
            currentStep = FEED_PASS;
            executeFeedPass();
            break;

        case FEED_PASS:
            if (!feedMotor->isRunning()) {
                if (currentPass < TOTAL_PASSES) {
                    // Move to next pass
                    currentPass++;
                    executeFeedPass();
                } else {
                    // All passes complete
                    currentStep = COMPLETE;
                }
            }
            break;

        case COMPLETE:
            if (!feedMotor->isRunning()) {
                // Check start cycle switch and transition to appropriate state
                if (getStartCycleSwitch()->read() == HIGH) {
                    //serial.println("FeedFirstCut: Start cycle switch HIGH - transitioning to CUTTING state");
                    changeState(CUTTING);
                    setCuttingCycleInProgress(true);
                    configureCutMotorForCutting();
                    turnYellowLedOn();
                    extendFeedClamp();
                } else {
                    //serial.println("FeedFirstCut: Start cycle switch LOW - transitioning to IDLE state");
                    changeState(IDLE);
                }
            }
            break;
    }
}

void executeFeedPass() {
    FastAccelStepper* feedMotor = getFeedMotor();
    if (!feedMotor) return;

    // Calculate distances for this pass
    float startPosition, endPosition;
    
    if (isFirstPass) {
        // First pass: start from 0, move forward by ball screw travel minus overlap
        startPosition = 0;
        endPosition = BALL_SCREW_TRAVEL - PASS_OVERLAP; // 3.9 inches
        isFirstPass = false;
        //serial.println("FeedFirstCut: First pass - moving wood from 0 to 3.9 inches");
    } else {
        // Subsequent passes: start from previous end minus overlap, move forward by remaining distance
        float previousEnd = BALL_SCREW_TRAVEL - PASS_OVERLAP; // 3.9 inches from first pass
        startPosition = previousEnd - PASS_OVERLAP; // 3.8 inches
        endPosition = TOTAL_WOOD_MOVEMENT; // 5.0 inches (full target distance)
        //serial.println("FeedFirstCut: Second pass - moving wood from 3.8 to 5.0 inches");
    }

    //serial.print("FeedFirstCut: Pass "); serial.print(currentPass); 
    //serial.print(" - Start: "); serial.print(startPosition, 1); 
    //serial.print(" End: "); serial.println(endPosition, 1);

    // Execute the pass sequence
    executeSinglePass(startPosition, endPosition);
}

void executeSinglePass(float startPos, float endPos) {
    static enum PassStep {
        RETRACT_CLAMP,
        MOVE_AWAY_FROM_HOME,
        EXTEND_CLAMP,
        WAIT_DELAY,
        MOVE_FORWARD_TO_PUSH_WOOD
    } passStep = RETRACT_CLAMP;
    
    static bool passStepInitialized = false;
    
    if (!passStepInitialized) {
        passStep = RETRACT_CLAMP;
        passStepInitialized = true;
        stepStartTime = 0;
    }

    FastAccelStepper* feedMotor = getFeedMotor();
    if (!feedMotor) return;

    switch (passStep) {
        case RETRACT_CLAMP:
            retractFeedClamp();
            retract2x4SecureClamp();
            passStep = MOVE_AWAY_FROM_HOME;
            break;

        case MOVE_AWAY_FROM_HOME:
            if (!feedMotor->isRunning()) {
                // Move feed motor away from home sensor to create space
                moveFeedMotorToPosition(BALL_SCREW_TRAVEL - PASS_OVERLAP);
                passStep = EXTEND_CLAMP;
            }
            break;

        case EXTEND_CLAMP:
            if (!feedMotor->isRunning()) {
                extendFeedClamp();
                retract2x4SecureClamp();
                stepStartTime = millis();
                passStep = WAIT_DELAY;
            }
            break;

        case WAIT_DELAY:
            if (millis() - stepStartTime >= CLAMP_DELAY_MS) {
                passStep = MOVE_FORWARD_TO_PUSH_WOOD;
            }
            break;

        case MOVE_FORWARD_TO_PUSH_WOOD:
            if (!feedMotor->isRunning()) {
                // Move forward to push wood the calculated distance
                float woodPushDistance = endPos - startPos;
                moveFeedMotorToPosition(BALL_SCREW_TRAVEL - PASS_OVERLAP - woodPushDistance);
                // Reset for next pass
                passStepInitialized = false;
            }
            break;
    }
}
