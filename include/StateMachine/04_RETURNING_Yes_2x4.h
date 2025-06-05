#ifndef _04_RETURNING_YES_2X4_STATE_H
#define _04_RETURNING_YES_2X4_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************ RETURNING YES 2X4 STATE **********************
//* ************************************************************************
// Handles the RETURNING_YES_2x4 cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

class ReturningYes2x4State : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return RETURNING_YES_2x4; }

private:
    // RETURNING_YES_2x4 sequence tracking
    int returningYes2x4SubStep = 0;
    int feedMotorReturnSubStep = 0; // For initial feed motor return sequence
    int feedHomingSubStep = 0; // For feed motor homing sequence
    
    // Helper methods for RETURNING_YES_2x4 sequence
    void handleReturningYes2x4Sequence(StateManager& stateManager);
    void handleFeedMotorReturnSequence(StateManager& stateManager);
    void handleReturningYes2x4FeedMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // _04_RETURNING_YES_2X4_STATE_H 