#ifndef CUTTING_STATE_H
#define CUTTING_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Completely refactored cutting state with clean, organized structure
// Handles wood cutting with acceleration curve, safety checks, and component activation

// State Management
void executeCuttingState();
void onEnterCuttingState();
void onExitCuttingState();
void resetCuttingState();

// Cutting Steps
void handleCuttingInitialization();
void handleCuttingExecution();
void handleCuttingCompletion();

// Monitoring Functions
void monitorWoodSensor();
void handleAccelerationCurve();
void checkSuctionSensor();
void activateComponentsAtPositions();

// Error Handling
void handleHomePositionError();

// Legacy functions (for compatibility)
void handleCuttingStep0();
void handleCuttingStep1();
void handleCuttingStep2();
void resetCuttingSteps();
void checkWoodPresentSensor();

#endif // CUTTING_STATE_H 