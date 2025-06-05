#ifndef CUTTING_STATE_H
#define CUTTING_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
    // This state manages a multi-step cutting process including YES_WOOD and NO_WOOD sequences.

class CuttingState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return CUTTING; }

private:
    // Main cutting step tracking
    int cuttingStep = 0;
    unsigned long stepStartTime = 0;
    unsigned long signalStartTime = 0;
    bool signalActive = false;
    bool homePositionErrorDetected = false;
    bool rotationClampActivatedThisCycle = false;
    bool rotationServoActivatedThisCycle = false;
    float cutMotorIncrementalMoveTotalInches = 0.0;
    int cuttingSubStep8 = 0; // For feed motor homing sequence
    
    // Concurrent feed motor homing during cutting
    bool feedMotorHomingActive = false;
    int concurrentFeedHomingSubStep = 0;
    bool feedMotorNeedsHoming = false;
    
    // Helper methods for different cutting phases
    void handleCuttingStep0(StateManager& stateManager);
    void handleCuttingStep1(StateManager& stateManager);
    void handleCuttingStep2(StateManager& stateManager);
    void handleCuttingStep3(StateManager& stateManager);
    void handleCuttingStep4(StateManager& stateManager);
    void handleCuttingStep5(StateManager& stateManager);
    void handleCuttingStep8_FeedMotorHomingSequence(StateManager& stateManager);
    void handleCuttingStep9_SuctionErrorRecovery(StateManager& stateManager);
    
    // Helper method for home position error
    void handleHomePositionError(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();

    // Concurrent feed motor homing during cutting
    void handleConcurrentFeedMotorHoming(StateManager& stateManager);
};

#endif // CUTTING_STATE_H 