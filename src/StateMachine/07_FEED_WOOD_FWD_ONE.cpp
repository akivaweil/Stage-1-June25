#include "StateMachine/07_FEED_WOOD_FWD_ONE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"

// FEED WOOD FWD ONE STATE
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

// STEP 1: RETRACT FEED CLAMP

// STEP 2: MOVE POSITION MOTOR TO ZERO

// STEP 3: EXTEND FEED CLAMP AND RETRACT TOP CLAMP

// STEP 4: WAIT 200MS

// STEP 5: MOVE TO TRAVEL DISTANCE

// STEP 6: CHECK START CYCLE SWITCH AND BRANCH
// HIGH → CUTTING (immediate). LOW → continue to step 7 for staggered IDLE handoff.

// STEP 7: EXTEND TOP CLAMP BEFORE FEED CLAMP RETRACTS

// STEP 8: WAIT 200MS FOR TOP CLAMP SETTLE → RETRACT FEED CLAMP → IDLE

// Top clamp must be extended this long before feed clamp may retract on the
// IDLE transition path (gives the top clamp time to physically seat on the wood
// so the wood is held throughout the hand-off).
const unsigned long TOP_CLAMP_SETTLE_BEFORE_FEED_RETRACT_MS = 200;

// Static variables for feed wood fwd one state tracking
enum FeedWoodFwdOneStep {
    RETRACT_FEED_CLAMP,
    MOVE_POSITION_MOTOR_TO_ZERO,
    EXTEND_FEED_CLAMP_RETRACT_TOP,
    WAIT_200MS,
    MOVE_TO_TRAVEL_DISTANCE,
    CHECK_START_CYCLE_SWITCH,
    EXTEND_TOP_CLAMP_BEFORE_IDLE,
    WAIT_TOP_CLAMP_SETTLE_BEFORE_IDLE
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
                moveFeedMotorToZero();
                //serial.println("FeedWoodFwdOne: Moving feed motor to 0");
                advanceToNextFeedWoodFwdOneStep();
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_TOP:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                retractTopClamp();
                //serial.println("FeedWoodFwdOne: Feed clamp extended, top clamp retracted");
                stepStartTime = millis();
                advanceToNextFeedWoodFwdOneStep();
            }
            break;

        case WAIT_200MS:
            if (millis() - stepStartTime >= 400) {
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
                    showYellowLed();
                    extendFeedClamp();
                } else {
                    //serial.println("FeedWoodFwdOne: Start cycle switch LOW - extending top clamp before IDLE");
                    advanceToNextFeedWoodFwdOneStep();
                }
            }
            break;

        case EXTEND_TOP_CLAMP_BEFORE_IDLE:
            // Top clamp must be settled on the wood before feed clamp releases,
            // otherwise the wood would be unsupported during the hand-off.
            extendTopClamp();
            stepStartTime = millis();
            currentStep = WAIT_TOP_CLAMP_SETTLE_BEFORE_IDLE;
            break;

        case WAIT_TOP_CLAMP_SETTLE_BEFORE_IDLE:
            if (millis() - stepStartTime >= TOP_CLAMP_SETTLE_BEFORE_FEED_RETRACT_MS) {
                retractFeedClamp();
                //serial.println("FeedWoodFwdOne: Top clamp settled - feed clamp retracted, transitioning to IDLE");
                changeState(IDLE);
            }
            break;
    }
}

void advanceToNextFeedWoodFwdOneStep() {
    currentStep = static_cast<FeedWoodFwdOneStep>(static_cast<int>(currentStep) + 1);
    stepStartTime = 0; // Reset step timer
} 