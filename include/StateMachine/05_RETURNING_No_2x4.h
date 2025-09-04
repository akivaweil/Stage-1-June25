#ifndef _05_RETURNING_NO_2X4_STATE_H
#define _05_RETURNING_NO_2X4_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

// Function declarations for RETURNING_NO_2x4 state
void executeReturningNo2x4State();
void onEnterReturningNo2x4State();
void onExitReturningNo2x4State();

// Main sequence handlers
void handleReturningNo2x4Sequence();

// Initialization
void initializeReturningNo2x4Sequence();

// No individual step handlers - everything happens sequentially in main function

// Sequence completion
void completeReturningNo2x4Sequence();

// Utility functions
void resetReturningNo2x4Steps();

#endif // _05_RETURNING_NO_2X4_STATE_H 