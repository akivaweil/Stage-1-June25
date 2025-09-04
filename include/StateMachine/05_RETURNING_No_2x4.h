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
void processCurrentStep(int step);

// Initialization
void initializeReturningNo2x4Sequence();

// Step handlers
void handleCutMotorWaitAndExtendFeedClamp();
void handleFeedMotorWaitAndRetractFeedClamp();
void handleFeedMotorMoveToPosition1();
void handleFeedMotorWaitAtPosition1AndExtendClamp();
void handleAttentionSequence();
void handleFeedMotorMoveToHome();
void handleFeedMotorWaitAtHomeAndRetractClamp();
void handleFeedMotorMoveToPosition2();
void handleFinalVerificationAndCompletion();

// Sensor verification
void verifyCutHomePosition(FastAccelStepper* cutMotor);
bool checkBothSensorsNotActive();

// Sequence completion
void completeReturningNo2x4Sequence();

// Utility functions
void resetReturningNo2x4Steps();

#endif // _05_RETURNING_NO_2X4_STATE_H 