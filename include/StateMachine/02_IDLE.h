#pragma once

#include "StateMachine/General_Functions.h"

// Idle State
// Handles the idle state, awaiting user input or automatic cycle start.

// Function declarations for IDLE state
void handleIdleState();
void onEnterIdleState();
void onExitIdleState();

// Helper function declarations
void checkFirstCutConditions();
void checkStartConditions();
