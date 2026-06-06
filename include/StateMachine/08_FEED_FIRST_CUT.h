#pragma once

#include "StateMachine/General_Functions.h"

// Feed First Cut State
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

// Function declarations for FEED_FIRST_CUT state
void handleFeedFirstCutState();
void onEnterFeedFirstCutState();
void onExitFeedFirstCutState();

// Helper function declarations
void executeFeedFirstCutStep();
void advanceToNextFeedFirstCutStep();
