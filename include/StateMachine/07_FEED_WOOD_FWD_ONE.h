#ifndef FEED_WOOD_FWD_ONE_H
#define FEED_WOOD_FWD_ONE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

class StateManager; // Forward declaration

//* ************************************************************************
//* ********************* FEED WOOD FWD ONE STATE **************************
//* ************************************************************************
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

class FeedWoodFwdOneState {
public:
    void execute(StateManager& stateManager);
    void onEnter(StateManager& stateManager);
    void onExit(StateManager& stateManager);
    SystemState getStateType() const { return FEED_WOOD_FWD_ONE; }

private:
    enum FeedWoodFwdOneStep {
        RETRACT_FEED_CLAMP,
        MOVE_POSITION_MOTOR_TO_ZERO,
        EXTEND_FEED_CLAMP_RETRACT_SECURE,
        WAIT_200MS,
        MOVE_TO_TRAVEL_DISTANCE,
        CHECK_START_CYCLE_SWITCH
    };

    // Member to track the current step in the feed wood forward one sequence
    FeedWoodFwdOneStep currentStep = RETRACT_FEED_CLAMP;
    
    void executeStep(StateManager& stateManager);
    void advanceToNextStep(StateManager& stateManager);
    unsigned long stepStartTime = 0;
};

#endif // FEED_WOOD_FWD_ONE_H 