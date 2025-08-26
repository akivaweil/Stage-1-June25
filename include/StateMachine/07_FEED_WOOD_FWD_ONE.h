#ifndef FEED_WOOD_FWD_ONE_H
#define FEED_WOOD_FWD_ONE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED WOOD FWD ONE STATE **************************
//* ************************************************************************
// Handles the feed wood forward one sequence when fix position switch is pressed
// in idle state AND 2x4 sensor reads LOW.

//* ************************************************************************
//* ******************** FEED STATE CONFIGURATION **************************
//* ************************************************************************
// Normal feed operation motor settings - defined in Config.h

// Feed travel distance - defined in Motor_Config.h

// Function declarations for FEED_WOOD_FWD_ONE state
void executeFeedWoodFwdOneState();
void onEnterFeedWoodFwdOneState();
void onExitFeedWoodFwdOneState();

// Helper function declarations
void executeFeedWoodFwdOneStep();
void advanceToNextFeedWoodFwdOneStep();

#endif // FEED_WOOD_FWD_ONE_H 