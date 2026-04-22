#ifndef STARTUP_STATE_H
#define STARTUP_STATE_H

#include "StateMachine/General_Functions.h"

//* ************************************************************************
//* ************************** STARTUP STATE *******************************
//* ************************************************************************
// Function-based startup state handling.
// Executes the initial startup state, transitioning to HOMING.

void executeStartupState();
void onEnterStartupState();
void onExitStartupState();

#endif // STARTUP_STATE_H 