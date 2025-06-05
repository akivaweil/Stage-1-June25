#ifndef FEED_FIRST_CUT_H
#define FEED_FIRST_CUT_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

class StateManager; // Forward declaration

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE **************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

class FeedFirstCutState {
public:
    void execute(StateManager& stateManager);
    void onEnter(StateManager& stateManager);
    void onExit(StateManager& stateManager);
    SystemState getStateType() const { return FEED_FIRST_CUT; }

private:
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

    // Member to track the current step in the feed first cut sequence
    FeedFirstCutStep currentStep = RETRACT_FEED_CLAMP;
    
    void executeStep(StateManager& stateManager);
    void advanceToNextStep(StateManager& stateManager);
    unsigned long stepStartTime = 0;
};

#endif // FEED_FIRST_CUT_H 