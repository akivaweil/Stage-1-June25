#pragma once

#include "StateMachine/General_Functions.h"

// Homing State
// Function-based homing state handling.
// Handles the homing sequence for all motors.

void handleHomingState();
void onEnterHomingState();
void onExitHomingState();
