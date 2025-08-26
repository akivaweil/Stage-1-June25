#include "StateMachine/07_FEED_WOOD_FWD_ONE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED WOOD FWD ONE STATE **************************
//* ************************************************************************
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

//! ************************************************************************
//! STEP 1: RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 2: MOVE POSITION MOTOR TO ZERO
//! ************************************************************************

//! ************************************************************************
//! STEP 3: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 4: WAIT 200MS
//! ************************************************************************

//! ************************************************************************
//! STEP 5: MOVE TO TRAVEL DISTANCE
//! ************************************************************************

//! ************************************************************************
//! STEP 6: CHECK START CYCLE SWITCH AND TRANSITION TO APPROPRIATE STATE
//! ************************************************************************

// Static variables for feed wood fwd one state tracking
enum FeedWoodFwdOneStep {
    RETRACT_FEED_CLAMP,
    MOVE_POSITION_MOTOR_TO_ZERO,
    EXTEND_FEED_CLAMP_RETRACT_SECURE,
    WAIT_200MS,
    MOVE_TO_TRAVEL_DISTANCE,
    CHECK_START_CYCLE_SWITCH
};

static FeedWoodFwdOneStep currentStep = RETRACT_FEED_CLAMP;
static unsigned long stepStartTime = 0;

void executeFeedWoodFwdOneState() {
    executeFeedWoodFwdOneStep();
}

void onEnterFeedWoodFwdOneState() {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    //serial.println("FeedWoodFwdOne: Starting feed wood forward one sequence");
}

void onExitFeedWoodFwdOneState() {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    //serial.println("FeedWoodFwdOne: Feed clamp retracted");
}

void executeFeedWoodFwdOneStep() {
    FastAccelStepper* feedMotor = getFeedMotor();

    switch (currentStep) {
        case RETRACT_FEED_CLAMP:
            retractFeedClamp();
            //serial.println("FeedWoodFwdOne: Feed clamp retracted");
            advanceToNextFeedWoodFwdOneStep();
            break;

        case MOVE_POSITION_MOTOR_TO_ZERO:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToHome();
                //serial.println("FeedWoodFwdOne: Moving feed motor to 0");
                advanceToNextFeedWoodFwdOneStep();
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                retract2x4SecureClamp();
                //serial.println("FeedWoodFwdOne: Feed clamp extended, secure 2x4 clamp retracted");
                stepStartTime = millis();
                advanceToNextFeedWoodFwdOneStep();
            }
            break;

        case WAIT_200MS:
            if (millis() - stepStartTime >= 200) {
                //serial.println("FeedWoodFwdOne: Waiting 200ms");
                advanceToNextFeedWoodFwdOneStep();
            }
            break;

        case MOVE_TO_TRAVEL_DISTANCE:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                //serial.println("FeedWoodFwdOne: Moving feed motor to travel distance");
                advanceToNextFeedWoodFwdOneStep();
            }
            break;

        case CHECK_START_CYCLE_SWITCH:
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("FeedWoodFwdOne: Checking start cycle switch for next state");
                
                // Check the start cycle switch state
                if (getStartCycleSwitch()->read() == HIGH) {
                    //serial.println("FeedWoodFwdOne: Start cycle switch HIGH - transitioning to CUTTING state");
                    changeState(CUTTING);
                    setCuttingCycleInProgress(true);
                    configureCutMotorForCutting();
                    turnYellowLedOn();
                    extendFeedClamp();
                } else {
                    //serial.println("FeedWoodFwdOne: Start cycle switch LOW - transitioning to IDLE state");
                    changeState(IDLE);
                }
            }
            break;
    }
}

void advanceToNextFeedWoodFwdOneStep() {
    currentStep = static_cast<FeedWoodFwdOneStep>(static_cast<int>(currentStep) + 1);
    stepStartTime = 0; // Reset step timer
} 