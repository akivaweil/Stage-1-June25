#ifndef _04_RETURNING_YES_2X4_STATE_H
#define _04_RETURNING_YES_2X4_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ******************** RETURNING YES 2X4 STATE **************************
//* ************************************************************************
// Handles the simultaneous return sequence when wood sensor detects lumber.
// Manages cut motor return to home while feed motor executes multi-step return sequence.
// Includes cut motor error handling with timeout and recovery logic.

//* ************************************************************************
//* ******************** RETURNING STATE CONFIGURATION ********************
//* ************************************************************************
// Return stroke motor speed settings - defined in Config.h

// Function declarations for RETURNING_YES_2x4 state
void executeReturningYes2x4State();
void onEnterReturningYes2x4State();
void onExitReturningYes2x4State();

// Helper function declarations for RETURNING_YES_2x4 sequence
void handleReturningYes2x4Sequence();
void handleFeedMotorReturnSequence();
void handleReturningYes2x4FeedMotorHoming();

// Reset all step counters
void resetReturningYes2x4Steps();

#endif // _04_RETURNING_YES_2X4_STATE_H 