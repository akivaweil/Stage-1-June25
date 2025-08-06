#ifndef FEED_WOOD_FWD_ONE_H
#define FEED_WOOD_FWD_ONE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED WOOD FWD ONE STATE **************************
//* ************************************************************************
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

// Function declarations for FEED_WOOD_FWD_ONE state
void executeFeedWoodFwdOneState();
void onEnterFeedWoodFwdOneState();
void onExitFeedWoodFwdOneState();

// Helper function declarations
void executeFeedWoodFwdOneStep();
void advanceToNextFeedWoodFwdOneStep();

#endif // FEED_WOOD_FWD_ONE_H 