#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "Config/Config.h"
#include "WebDashboard/WebDashboard.h"

// RELEVANT CONSTANTS
// State-specific constants
const float FEED_MOTOR_FIRST_RUN_START_POSITION = -1.2; // inches - absolute position for first run start
const float FEED_MOTOR_FIRST_RUN_END_POSITION = 3.4; // inches - absolute position for first run end
const float FEED_MOTOR_SECOND_RUN_START_POSITION = -1.2; // inches - absolute position for second run start
const float FEED_MOTOR_SECOND_RUN_END_POSITION = -0.05; // inches - absolute position for second run end
const float FEED_MOTOR_MINIS_SECOND_RUN_OFFSET = -0.25; // inches - in Minis mode (2.65" squares), advance 0.25" less while clamped
const unsigned long FEED_CLAMP_DELAY_MS = 400; // Delay after extending feed clamp and retracting top clamp

extern int getCurrentConfigMode();

// Note: FEED_TRAVEL_DISTANCE, FEED_MOTOR_STEPS_PER_INCH, FEED_CLAMP, TOP_CLAMP,
// and START_CYCLE_SWITCH are already defined in Config files and accessible via includes

// FEED FIRST CUT STATE
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

// STEP 1: RETRACT FEED CLAMP

// STEP 2: MOVE TO FIRST RUN START POSITION (-1.2 INCHES)

// STEP 3: EXTEND FEED CLAMP AND RETRACT TOP CLAMP

// STEP 4: WAIT 200MS

// STEP 5: MOVE TO FIRST RUN END POSITION (3.4 INCHES)

// STEP 6: FIRST RUN COMPLETE - PREPARE FOR SECOND RUN

// STEP 7: RETRACT FEED CLAMP (SECOND RUN)

// STEP 8: MOVE TO SECOND RUN START POSITION (-1.2 INCHES)

// STEP 9: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP (SECOND RUN)

// STEP 10: WAIT 200MS (SECOND RUN)

// STEP 11: MOVE TO SECOND RUN END POSITION (2.0 INCHES)

// STEP 12: CHECK START CYCLE SWITCH AND TRANSITION TO APPROPRIATE STATE

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

void handleFeedFirstCutState() {
    executeFeedFirstCutStep();
}

void onEnterFeedFirstCutState() {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    stopReloadTimer(); // Stop reload time tracking when entering feed first cut state
}

void onExitFeedFirstCutState() {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
}

void executeFeedFirstCutStep() {
    FastAccelStepper* feedMotor = getFeedMotor();
    // FEED_TRAVEL_DISTANCE and FEED_MOTOR_STEPS_PER_INCH are now defined locally

    switch (currentStep) {
        case RETRACT_FEED_CLAMP:
            retractFeedClamp();
            advanceToNextFeedFirstCutStep();
            break;

        case MOVE_TO_FIRST_RUN_START_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_FIRST_RUN_START_POSITION);
                advanceToNextFeedFirstCutStep();
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                // Only retract top clamp if not coming from no-wood situation
                if (!getComingFromNoWoodWithSensorsClear()) {
                    retractTopClamp();
                }
                stepStartTime = millis();
                advanceToNextFeedFirstCutStep();
            }
            break;

        case WAIT_200MS:
            if (millis() - stepStartTime >= FEED_CLAMP_DELAY_MS) {
                advanceToNextFeedFirstCutStep();
            }
            break;

        case MOVE_TO_FIRST_RUN_END_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_FIRST_RUN_END_POSITION);
                advanceToNextFeedFirstCutStep();
            }
            break;

        case FIRST_RUN_COMPLETE:
            if (feedMotor && !feedMotor->isRunning()) {
                advanceToNextFeedFirstCutStep();
            }
            break;

        case RETRACT_FEED_CLAMP_SECOND:
            retractFeedClamp();
            advanceToNextFeedFirstCutStep();
            break;

        case MOVE_TO_SECOND_RUN_START_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_MOTOR_SECOND_RUN_START_POSITION);
                advanceToNextFeedFirstCutStep();
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE_SECOND:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                retractTopClamp();
                stepStartTime = millis();
                advanceToNextFeedFirstCutStep();
            }
            break;

        case WAIT_200MS_SECOND:
            if (millis() - stepStartTime >= FEED_CLAMP_DELAY_MS) {
                advanceToNextFeedFirstCutStep();
            }
            break;

        case MOVE_TO_SECOND_RUN_END_POSITION:
            if (feedMotor && !feedMotor->isRunning()) {
                float endPos = FEED_MOTOR_SECOND_RUN_END_POSITION;
                if (getCurrentConfigMode() == 1) {
                    endPos += FEED_MOTOR_MINIS_SECOND_RUN_OFFSET;
                }
                moveFeedMotorToPosition(endPos);
                advanceToNextFeedFirstCutStep();
            }
            break;

        case CHECK_START_CYCLE_SWITCH:
            if (feedMotor && !feedMotor->isRunning()) {
                
                // Set start switch safety flag as if user flipped the switch
                setStartSwitchSafe(true);
                
                // Reset the no-wood flag when completing feed first cut
                setComingFromNoWoodWithSensorsClear(false);
                
                // Check the start cycle switch state
                if (getStartCycleSwitch()->read() == HIGH) {
                    changeState(STATE_CUTTING);
                    setCuttingCycleInProgress(true);
                    configureCutMotorForCutting();
                    showYellowLed();
                    extendFeedClamp();
                } else {
                    changeState(STATE_IDLE);
                }
            }
            break;
    }
}

void advanceToNextFeedFirstCutStep() {
    currentStep = static_cast<FeedFirstCutStep>(static_cast<int>(currentStep) + 1);
    stepStartTime = 0; // Reset step timer
}
