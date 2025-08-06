#ifndef FEED_FIRST_CUT_H
#define FEED_FIRST_CUT_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************* FEED FIRST CUT STATE **************************
//* ************************************************************************
// Handles the feed first cut sequence when pushwood forward switch is pressed
// in idle state AND 2x4 sensor reads high.

// Function declarations for FEED_FIRST_CUT state
void executeFeedFirstCutState();
void onEnterFeedFirstCutState();
void onExitFeedFirstCutState();

// Helper function declarations
void executeFeedFirstCutStep();
void advanceToNextFeedFirstCutStep();

#endif // FEED_FIRST_CUT_H 