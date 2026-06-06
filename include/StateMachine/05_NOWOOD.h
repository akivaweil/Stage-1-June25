#pragma once

#include "StateMachine/General_Functions.h"

// Nowood State
// Handles the NOWOOD sequence when no wood is detected.
// Cut motor returns home in the background while feed motor runs its sequence.

void handleNowoodState();
void onEnterNowoodState();
void onExitNowoodState();
void handleNowoodSequence();
void resetNowoodSteps();

