#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE **************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

void FeedFirstCutState::execute(StateManager& stateManager) {
    executeStep(stateManager);
}

void FeedFirstCutState::onEnter(StateManager& stateManager) {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    Serial.println("FeedFirstCut: Starting feed first cut sequence");
}

void FeedFirstCutState::onExit(StateManager& stateManager) {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    Serial.println("FeedFirstCut: Feed clamp retracted");
}

void FeedFirstCutState::executeStep(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_TRAVEL_DISTANCE;
    // FEED_MOTOR_STEPS_PER_INCH is already declared in General_Functions.h

    switch (currentStep) {
        case RETRACT_FEED_CLAMP:
            retractFeedClamp();
            Serial.println("FeedFirstCut: Feed clamp retracted");
            advanceToNextStep(stateManager);
            break;

        case MOVE_TO_NEGATIVE_ONE:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(-1.0);
                Serial.println("FeedFirstCut: Moving feed motor to -1 inch");
                advanceToNextStep(stateManager);
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE:
            if (feedMotor && !feedMotor->isRunning()) {
                //! STEP 3: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP
                extendFeedClamp();
                retract2x4SecureClamp();
                Serial.println("FeedFirstCut: Feed clamp extended, secure wood clamp retracted");
                stepStartTime = millis();
                advanceToNextStep(stateManager);
            }
            break;

        case WAIT_200MS:
            if (millis() - stepStartTime >= 200) {
                Serial.println("FeedFirstCut: Waiting 200ms");
                advanceToNextStep(stateManager);
            }
            break;

        case MOVE_TO_TRAVEL_DISTANCE:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                Serial.println("FeedFirstCut: Moving feed motor to travel distance");
                advanceToNextStep(stateManager);
            }
            break;

        case FIRST_RUN_COMPLETE:
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("FeedFirstCut: First run complete, starting second run");
                advanceToNextStep(stateManager);
            }
            break;

        case RETRACT_FEED_CLAMP_SECOND:
            retractFeedClamp();
            Serial.println("FeedFirstCut: Feed clamp retracted (second run)");
            advanceToNextStep(stateManager);
            break;

        case MOVE_TO_NEGATIVE_TWO:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(-2.0);
                Serial.println("FeedFirstCut: Moving feed motor to -2 inch (second run)");
                advanceToNextStep(stateManager);
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE_SECOND:
            if (feedMotor && !feedMotor->isRunning()) {
                //! STEP 9: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP (SECOND RUN)
                extendFeedClamp();
                retract2x4SecureClamp();
                Serial.println("FeedFirstCut: Feed clamp extended, secure wood clamp retracted (second run)");
                stepStartTime = millis();
                advanceToNextStep(stateManager);
            }
            break;

        case WAIT_200MS_SECOND:
            if (millis() - stepStartTime >= 200) {
                Serial.println("FeedFirstCut: Waiting 200ms (second run)");
                advanceToNextStep(stateManager);
            }
            break;

        case MOVE_TO_TRAVEL_DISTANCE_MINUS_2_75:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE - 2.75);
                Serial.println("FeedFirstCut: Moving feed motor to travel distance minus 2.75 inches");
                advanceToNextStep(stateManager);
            }
            break;

        case CHECK_START_CYCLE_SWITCH:
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("FeedFirstCut: Checking start cycle switch for next state");
                
                // Check the start cycle switch state
                if (stateManager.getStartCycleSwitch()->read() == HIGH) {
                    Serial.println("FeedFirstCut: Start cycle switch HIGH - transitioning to CUTTING state");
                    stateManager.changeState(CUTTING);
                    stateManager.setCuttingCycleInProgress(true);
                    configureCutMotorForCutting();
                    turnYellowLedOn();
                    extendFeedClamp();
                } else {
                    Serial.println("FeedFirstCut: Start cycle switch LOW - transitioning to IDLE state");
                    stateManager.changeState(IDLE);
                }
            }
            break;
    }
}

void FeedFirstCutState::advanceToNextStep(StateManager& stateManager) {
    currentStep = static_cast<FeedFirstCutStep>(static_cast<int>(currentStep) + 1);
    stepStartTime = 0; // Reset step timer
}
