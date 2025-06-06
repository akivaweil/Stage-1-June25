#ifndef _04_RETURNING_YES_2X4_STATE_H
#define _04_RETURNING_YES_2X4_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ******************** RETURNING YES 2X4 STATE **************************
//* ************************************************************************
// Handles the simultaneous return sequence when wood sensor detects lumber.
// Manages cut motor return to home while feed motor executes multi-step return sequence.
// Includes cut motor error handling with timeout and recovery logic.

class ReturningYes2x4State : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return RETURNING_YES_2x4; }

private:
    // Step tracking variables
    int returningYes2x4SubStep = 0;
    int feedMotorReturnSubStep = 0; // For initial feed motor return sequence
    int feedHomingSubStep = 0; // For feed motor homing sequence
    
    // Cut motor homing verification timing
    unsigned long cutMotorHomingAttemptStartTime = 0;
    bool cutMotorHomingAttemptInProgress = false;
    unsigned long cutMotorFinalVerificationStartTime = 0;
    bool cutMotorFinalVerificationInProgress = false;
    unsigned long cutMotorSensorStabilizationStartTime = 0;  // Separate timing for sensor stabilization
    
    // Helper methods for RETURNING_YES_2x4 sequence
    void handleReturningYes2x4Sequence(StateManager& stateManager);
    void handleFeedMotorReturnSequence(StateManager& stateManager);
    void handleReturningYes2x4FeedMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // _04_RETURNING_YES_2X4_STATE_H 