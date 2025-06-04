#ifndef HOMING_H
#define HOMING_H

// Include config files
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* **************************** HOMING LOGIC ******************************
//* ************************************************************************
// This file contains the function declarations for the homing sequence of the
// Transfer Arm.

// Forward declaration
class TransferArm;

// Function declarations
void homeSystem();
void homeZAxis();
void homeXAxis();

#endif  // HOMING_H