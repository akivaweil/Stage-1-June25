#ifndef _05_RETURNING_NO_2X4_STATE_H
#define _05_RETURNING_NO_2X4_STATE_H

#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 sequence when no wood is detected.
// Cut motor returns home in the background while feed motor runs its sequence.

void executeReturningNo2x4State();
void onEnterReturningNo2x4State();
void onExitReturningNo2x4State();
void handleReturningNo2x4Sequence();
void resetReturningNo2x4Steps();

#endif // _05_RETURNING_NO_2X4_STATE_H
