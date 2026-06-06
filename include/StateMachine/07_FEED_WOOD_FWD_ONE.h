#pragma once

#include "StateMachine/General_Functions.h"

// Feed Wood Fwd One State
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

// Function declarations for FEED_WOOD_FWD_ONE state
void handleFeedWoodFwdOneState();
void onEnterFeedWoodFwdOneState();
void onExitFeedWoodFwdOneState();

// Helper function declarations
void executeFeedWoodFwdOneStep();
void advanceToNextFeedWoodFwdOneStep();
