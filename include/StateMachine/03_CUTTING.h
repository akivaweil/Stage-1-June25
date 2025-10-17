#ifndef CUTTING_STATE_H
#define CUTTING_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation with a clean 3-step process:
// Step 0: Initialize cutting sequence - extend clamps and configure motors
// Step 1: Check suction sensor and handle cut motor movement
// Step 2: Monitor cut motor position, activate rotation components, and complete cut

void executeCuttingState();
void onEnterCuttingState();
void onExitCuttingState();
void handleCuttingStep0();
void handleCuttingStep1();
void handleCuttingStep2();
void handleHomePositionError();
void resetCuttingSteps();

#endif // CUTTING_STATE_H 