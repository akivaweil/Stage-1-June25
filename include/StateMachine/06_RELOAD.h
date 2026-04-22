#ifndef RELOAD_STATE_H
#define RELOAD_STATE_H

#include "StateMachine/General_Functions.h"

//* ************************************************************************
//* ************************** RELOAD STATE ********************************
//* ************************************************************************
// Handles reload mode operations when the reload switch is active.
// Provides safe conditions for loading new wood pieces into the system.
// Maintains clamps in retracted position for safe wood loading.

void executeReloadState();
void onEnterReloadState();
void onExitReloadState();

// Helper function declarations
void handleReloadModeLogic();

#endif // RELOAD_STATE_H
