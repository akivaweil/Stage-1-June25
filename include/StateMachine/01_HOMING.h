#ifndef HOMING_STATE_H
#define HOMING_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** HOMING STATE ********************************
//* ************************************************************************
// Function-based homing state handling.
// Handles the homing sequence for all motors.

void executeHomingState();
void onEnterHomingState();
void onExitHomingState();

#endif // HOMING_STATE_H 