#include "StateMachine/07_FEED_WOOD_FWD_ONE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED WOOD FWD ONE STATE **************************
//* ************************************************************************
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

void FeedWoodFwdOneState::execute(StateManager& stateManager) {
    executeStep(stateManager);
}

void FeedWoodFwdOneState::onEnter(StateManager& stateManager) {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    Serial.println("FeedWoodFwdOne: Starting feed wood forward one sequence");
}

void FeedWoodFwdOneState::onExit(StateManager& stateManager) {
    currentStep = RETRACT_FEED_CLAMP;
    stepStartTime = 0;
    Serial.println("FeedWoodFwdOne: Feed clamp retracted");
}

void FeedWoodFwdOneState::executeStep(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_TRAVEL_DISTANCE;

    switch (currentStep) {
        case RETRACT_FEED_CLAMP:
            retractFeedClamp();
            Serial.println("FeedWoodFwdOne: Feed clamp retracted");
            advanceToNextStep(stateManager);
            break;

        case MOVE_POSITION_MOTOR_TO_ZERO:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToHome();
                Serial.println("FeedWoodFwdOne: Moving feed motor to 0");
                advanceToNextStep(stateManager);
            }
            break;

        case EXTEND_FEED_CLAMP_RETRACT_SECURE:
            if (feedMotor && !feedMotor->isRunning()) {
                //! STEP 3: EXTEND FEED CLAMP AND RETRACT SECURE WOOD CLAMP
                extendFeedClamp();
                retract2x4SecureClamp();
                Serial.println("FeedWoodFwdOne: Feed clamp extended, secure 2x4 clamp retracted");
                stepStartTime = millis();
                advanceToNextStep(stateManager);
            }
            break;

        case WAIT_200MS:
            if (millis() - stepStartTime >= 200) {
                Serial.println("FeedWoodFwdOne: Waiting 200ms");
                advanceToNextStep(stateManager);
            }
            break;

        case MOVE_TO_TRAVEL_DISTANCE:
            if (feedMotor && !feedMotor->isRunning()) {
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                Serial.println("FeedWoodFwdOne: Moving feed motor to travel distance");
                advanceToNextStep(stateManager);
            }
            break;

        case CHECK_START_CYCLE_SWITCH:
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("FeedWoodFwdOne: Checking start cycle switch for next state");
                
                // Check the start cycle switch state
                if (stateManager.getStartCycleSwitch()->read() == HIGH) {
                    Serial.println("FeedWoodFwdOne: Start cycle switch HIGH - transitioning to CUTTING state");
                    stateManager.changeState(CUTTING);
                    stateManager.setCuttingCycleInProgress(true);
                    configureCutMotorForCutting();
                    turnYellowLedOn();
                    extendFeedClamp();
                } else {
                    Serial.println("FeedWoodFwdOne: Start cycle switch LOW - transitioning to IDLE state");
                    stateManager.changeState(IDLE);
                }
            }
            break;
    }
}

void FeedWoodFwdOneState::advanceToNextStep(StateManager& stateManager) {
    currentStep = static_cast<FeedWoodFwdOneStep>(static_cast<int>(currentStep) + 1);
    stepStartTime = 0; // Reset step timer
} 