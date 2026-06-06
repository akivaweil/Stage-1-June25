#pragma once

#include "StateMachine/General_Functions.h"

// Reload State
// Handles reload mode operations when the reload switch is active.
// Provides safe conditions for loading new wood pieces into the system.
// Maintains clamps in retracted position for safe wood loading.

void handleReloadState();
void onEnterReloadState();
void onExitReloadState();

// Helper function declarations
void handleReloadModeLogic();

