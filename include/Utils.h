#ifndef UTILS_H
#define UTILS_H

#include "FastAccelStepper.h"
#include <Arduino.h>
#include "Config/Config.h"

//* ************************************************************************
//* ************************ UTILITY FUNCTION DECLARATIONS ***************************
//* ************************************************************************
// This file contains declarations for all utility functions used throughout the Transfer Arm system.

// Movement functions
bool moveToPosition(FastAccelStepper* stepper, float position);
void activateVacuum();
void deactivateVacuum();

// Timing functions
bool Wait(unsigned long delayTime, unsigned long* startTimePtr);
void resetStateTimer(unsigned long* timer);

// Servo functions
void setServoPickup();
void setServoTravel();
void setServoDropoff();
void setServoHome();

// Axis control functions
void setZAxisNormalSpeed();
void setZAxisDropoffSpeed();
void moveZToPickup();
void moveZToUp();
void moveZToDropoff();
void moveXToPickup();
void moveXToDropoff();
void moveXToDropoffOvershoot();
void homeXAxis();

// Status functions
bool isZAxisAtTarget();
bool isXAxisAtTarget();
bool isZAtSuctionStart();

// State functions
const char* getStateString(PickCycleState state);

// Signal functions
void signalStage2();

// Logging functions
void smartLog(const String& message);

#endif  // UTILS_H