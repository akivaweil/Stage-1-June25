#ifndef CUTTING_STATE_H
#define CUTTING_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
// This state manages a multi-step cutting process including YES_WOOD and NO_WOOD sequences.

//* ************************************************************************
//* ************************ CUTTING STATE CONFIGURATION ******************
//* ************************************************************************
// Motor speed settings and offsets - defined in Config.h

// Function declarations for CUTTING state
void executeCuttingState();
void onEnterCuttingState();
void onExitCuttingState();

// Helper function declarations for different cutting phases
void handleCuttingStep0();
void handleCuttingStep1();
void handleCuttingStep2();
void handleCuttingStep3();
void handleCuttingStep4();
void handleCuttingStep5();
void handleCuttingStep8_FeedMotorHomingSequence();
void handleCuttingStep9_SuctionErrorRecovery();

// Helper function for home position error
void handleHomePositionError();

// Reset all step counters
void resetCuttingSteps();

#endif // CUTTING_STATE_H 