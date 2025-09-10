#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************ RELEVANT CONSTANTS **************************
//* ************************************************************************
// Feed motor absolute position constants for this state (specific to this state)
const float FEED_MOTOR_FIRST_RUN_START_POSITION = -1.2; // inches - absolute position for first run start
const float FEED_MOTOR_FIRST_RUN_END_POSITION = 3.4; // inches - absolute position for first run end (FEED_TRAVEL_DISTANCE)
const float FEED_MOTOR_SECOND_RUN_START_POSITION = -1.2; // inches - absolute position for second run start
const float FEED_MOTOR_SECOND_RUN_END_POSITION = 2.6; // inches - absolute position for second run end (3.4 - 1.4)

// Timing constants for this state
const unsigned long FEED_CLAMP_DELAY_MS = 200; // Delay after extending feed clamp and retracting secure clamp

// Note: FEED_TRAVEL_DISTANCE, FEED_MOTOR_STEPS_PER_INCH, FEED_CLAMP, _2x4_SECURE_CLAMP, 
// and START_CYCLE_SWITCH are already defined in Config files and accessible via includes

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE **************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

//! ************************************************************************
//! STEP 1: RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 2: MOVE TO FIRST RUN START POSITION (-1.2 INCHES)
//! ************************************************************************

//! ************************************************************************
//! STEP 3: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 4: WAIT 200MS
//! ************************************************************************

//! ************************************************************************
//! STEP 5: MOVE TO FIRST RUN END POSITION (3.4 INCHES)
//! ************************************************************************

//! ************************************************************************
//! STEP 6: FIRST RUN COMPLETE - PREPARE FOR SECOND RUN
//! ************************************************************************

//! ************************************************************************
//! STEP 7: RETRACT FEED CLAMP (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 8: MOVE TO SECOND RUN START POSITION (-1.2 INCHES)
//! ************************************************************************

//! ************************************************************************
//! STEP 9: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 10: WAIT 200MS (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 11: MOVE TO SECOND RUN END POSITION (2.0 INCHES)
//! ************************************************************************

//! ************************************************************************
//! STEP 12: CHECK START CYCLE SWITCH AND TRANSITION TO APPROPRIATE STATE
//! ************************************************************************

// Static variables for feed first cut state tracking
enum FeedFirstCutStep {
    RETRACT_FEED_CLAMP,
    MOVE_TO_FIRST_RUN_START_POSITION,
    EXTEND_FEED_CLAMP_RETRACT_SECURE,
    WAIT_200MS,
    MOVE_TO_FIRST_RUN_END_POSITION,
    FIRST_RUN_COMPLETE,
    RETRACT_FEED_CLAMP_SECOND,
    MOVE_TO_SECOND_RUN_START_POSITION,
    EXTEND_FEED_CLAMP_RETRACT_SECURE_SECOND,
    WAIT_200MS_SECOND,
    MOVE_TO_SECOND_RUN_END_POSITION,
    CHECK_START_CYCLE_SWITCH
};

static FeedFirstCutStep currentStep = RETRACT_FEED_CLAMP;
static unsigned long stepStartTime = 0;

void executeFeedFirstCutState() {
    executeFeedFirstCutStep();
}

void onEnterFeedFirstCutState() {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    //serial.println("FeedFirstCut: Starting feed first cut sequence");
}

void onExitFeedFirstCutState() {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    //serial.println("FeedFirstCut: Feed clamp retracted");
}

void executeFeedFirstCutStep() {
    FastAccelStepper* feedMotor = getFeedMotor();
    // FEED_TRAVEL_DISTANCE and FEED_MOTOR_STEPS_PER_INCH are now defined locally

    switch (currentStep) {
        case RETRACT_FEED_CLAMP:
            retractFeedClamp();
            //serial.println("FeedFirstCut: Feed clamp retracted");
            advanceToNextFeedFirstCutStep();
            break;

        case MOVE_TO_FIRST_RUN_START_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_FIRST_RUN_START_POSITION);
                //serial.println("FeedFirstCut: Moving feed motor to first run start position (-1.2 inches)");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                retract2x4SecureClamp();
                //serial.println("FeedFirstCut: Feed clamp extended, secure wood clamp retracted");
                stepStartTime = millis();
                advanceToNextFeedFirstCutStep();
            }
            break;

        case WAIT_200MS:
            if (millis() - stepStartTime >= FEED_CLAMP_DELAY_MS) {
                //serial.println("FeedFirstCut: Waiting 200ms");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case MOVE_TO_FIRST_RUN_END_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_FIRST_RUN_END_POSITION);
                //serial.println("FeedFirstCut: Moving feed motor to first run end position (3.4 inches)");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case FIRST_RUN_COMPLETE:
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("FeedFirstCut: First run complete, starting second run");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case RETRACT_FEED_CLAMP_SECOND:
            retractFeedClamp();
            //serial.println("FeedFirstCut: Feed clamp retracted (second run)");
            advanceToNextFeedFirstCutStep();
            break;

        case MOVE_TO_SECOND_RUN_START_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_SECOND_RUN_START_POSITION);
                //serial.println("FeedFirstCut: Moving feed motor to second run start position (-1.2 inches)");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE_SECOND:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                retract2x4SecureClamp();
                //serial.println("FeedFirstCut: Feed clamp extended, secure wood clamp retracted (second run)");
                stepStartTime = millis();
                advanceToNextFeedFirstCutStep();
            }
            break;

        case WAIT_200MS_SECOND:
            if (millis() - stepStartTime >= FEED_CLAMP_DELAY_MS) {
                //serial.println("FeedFirstCut: Waiting 200ms (second run)");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case MOVE_TO_SECOND_RUN_END_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_SECOND_RUN_END_POSITION);
                //serial.println("FeedFirstCut: Moving feed motor to second run end position (2.0 inches)");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case CHECK_START_CYCLE_SWITCH:
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("FeedFirstCut: Checking start cycle switch for next state");
                
                // Set start switch safety flag as if user flipped the switch
                setStartSwitchSafe(true);
                
                // Check the start cycle switch state
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

void advanceToNextFeedFirstCutStep() {
    currentStep = static_cast<FeedFirstCutStep>(static_cast<int>(currentStep) + 1);
    stepStartTime = 0; // Reset step timer
}
