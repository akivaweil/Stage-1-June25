#pragma once

#include "StateMachine/General_Functions.h"

// Startup State
// Function-based startup state handling.
// Executes the initial startup state, transitioning to HOMING.

void handleStartupState();
void onEnterStartupState();
void onExitStartupState();
