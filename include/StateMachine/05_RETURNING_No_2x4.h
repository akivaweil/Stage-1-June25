#ifndef _05_RETURNING_NO_2X4_STATE_H
#define _05_RETURNING_NO_2X4_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

class ReturningNo2x4State : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return RETURNING_NO_2x4; }

private:
    // RETURNING_NO_2x4 sequence tracking
    int returningNo2x4Step = 0;
    int returningNo2x4HomingSubStep = 0; // For RETURNING_NO_2x4 feed motor homing sequence
    unsigned long cylinderActionTime = 0;
    bool waitingForCylinder = false;
    
    // Helper methods for RETURNING_NO_2x4 sequence
    void handleReturningNo2x4Sequence(StateManager& stateManager);
    void handleReturningNo2x4Step(StateManager& stateManager, int step);
    void handleReturningNo2x4FeedMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // _05_RETURNING_NO_2X4_STATE_H 