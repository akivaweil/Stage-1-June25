#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** IDLE STATE **********************************
//* ************************************************************************
// Handles the idle state, awaiting user input or automatic cycle start.

// Function declarations for IDLE state
void executeIdleState();
void onEnterIdleState();
void onExitIdleState();

// Helper function declarations
void handleReloadModeLogic();
void checkFirstCutConditions();
void checkStartConditions();

#endif // IDLE_STATE_H 