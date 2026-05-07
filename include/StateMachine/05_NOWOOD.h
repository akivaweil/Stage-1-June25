#ifndef _05_NOWOOD_H
#define _05_NOWOOD_H

#include "StateMachine/General_Functions.h"

//* ************************************************************************
//* **************************** NOWOOD STATE *****************************
//* ************************************************************************
// Handles the NOWOOD sequence when no wood is detected.
// Cut motor returns home in the background while feed motor runs its sequence.

void executeNowoodState();
void onEnterNowoodState();
void onExitNowoodState();
void handleNowoodSequence();
void resetNowoodSteps();

#endif // _05_NOWOOD_H
