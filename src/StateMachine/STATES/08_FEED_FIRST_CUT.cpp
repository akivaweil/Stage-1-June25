#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************ RELEVANT CONSTANTS **************************
//* ************************************************************************
// Feed motor movement constants for this state (specific to this state)
const float FEED_MOTOR_RETRACT_POSITION = -4.6; // inches - position for retracting feed motor (maintains same physical movement as original home=3.4 system)
const float FEED_MOTOR_SECOND_RUN_OFFSET = 1.4; // inches - offset for second run final position

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
//! STEP 2: MOVE TO NEGATIVE 4.6 INCHES
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
//! STEP 6: FIRST RUN COMPLETE - PREPARE FOR SECOND RUN
//! ************************************************************************

//! ************************************************************************
//! STEP 7: RETRACT FEED CLAMP (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 8: MOVE TO NEGATIVE 4.6 INCHES (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 9: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 10: WAIT 200MS (SECOND RUN)
//! ************************************************************************

//! ************************************************************************
//! STEP 11: MOVE TO TRAVEL DISTANCE MINUS 1.4 INCHES
//! ************************************************************************

//! ************************************************************************
//! STEP 12: CHECK START CYCLE SWITCH AND TRANSITION TO APPROPRIATE STATE
//! ************************************************************************

// Static variables for feed first cut state tracking
enum FeedFirstCutStep {
    RETRACT_FEED_CLAMP,
    MOVE_TO_NEGATIVE_ONE,
    EXTEND_FEED_CLAMP_RETRACT_SECURE,
    WAIT_200MS,
    MOVE_TO_TRAVEL_DISTANCE,
    FIRST_RUN_COMPLETE,
    RETRACT_FEED_CLAMP_SECOND,
    MOVE_TO_NEGATIVE_TWO,
    EXTEND_FEED_CLAMP_RETRACT_SECURE_SECOND,
    WAIT_200MS_SECOND,
    MOVE_TO_TRAVEL_DISTANCE_MINUS_2_75,
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

        case MOVE_TO_NEGATIVE_ONE:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_RETRACT_POSITION);
                //serial.println("FeedFirstCut: Moving feed motor to -4.6 inch");
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

        case MOVE_TO_TRAVEL_DISTANCE:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(0.0);
                //serial.println("FeedFirstCut: Moving feed motor to travel distance");
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

        case MOVE_TO_NEGATIVE_TWO:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_RETRACT_POSITION);
                //serial.println("FeedFirstCut: Moving feed motor to -4.6 inch (second run)");
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

        case MOVE_TO_TRAVEL_DISTANCE_MINUS_2_75:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(-FEED_MOTOR_SECOND_RUN_OFFSET);
                //serial.println("FeedFirstCut: Moving feed motor to travel distance minus 1.4 inches");
                advanceToNextFeedFirstCutStep();
            }
            break;

        case CHECK_START_CYCLE_SWITCH:
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("FeedFirstCut: Checking start cycle switch for next state");
                
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
