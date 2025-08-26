#ifndef FEED_FIRST_CUT_H
#define FEED_FIRST_CUT_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE **************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

//* ************************************************************************
//* ******************** FEED STATE CONFIGURATION **************************
//* ************************************************************************
// Configuration variables are defined in the .cpp file for easy adjustment:
// - TOTAL_WOOD_MOVEMENT: Total distance wood actually moves forward (inches)
// - BALL_SCREW_TRAVEL: Maximum ball screw travel (inches)
// - PASS_OVERLAP: Overlap between passes (inches)
// - CLAMP_DELAY_MS: Delay after clamp operations (ms)

// Function declarations for FEED_FIRST_CUT state
void executeFeedFirstCutState();
void onEnterFeedFirstCutState();
void onExitFeedFirstCutState();

// Helper function declarations
void executeFeedFirstCutStep();
void executeFeedPass();
void executeSinglePass(float startPos, float endPos);

#endif // FEED_FIRST_CUT_H 