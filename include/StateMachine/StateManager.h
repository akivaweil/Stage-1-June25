#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************* STATE MANAGER *******************************
//* ************************************************************************
// Function-based state management that coordinates all state operations
// and provides access to system resources.

// Main state machine execution function
void executeStateMachine();

// State transition management functions
void changeState(SystemState newState);
SystemState getCurrentState();
SystemState getPreviousState();

// System resource access functions
FastAccelStepper* getCutMotor();
FastAccelStepper* getFeedMotor();
Servo* getRotationServo();

// Switch access functions
Bounce* getCutHomingSwitch();
Bounce* getFeedHomingSwitch();
Bounce* getReloadSwitch();
Bounce* getStartCycleSwitch();
Bounce* getSuctionSensorBounce();

// System flag access functions
bool getIsReloadMode();
void setIsReloadMode(bool value);

bool get2x4Present();
void set2x4Present(bool value);

bool getWoodSuctionError();
void setWoodSuctionError(bool value);

bool getErrorAcknowledged();
void setErrorAcknowledged(bool value);

bool getCuttingCycleInProgress();
void setCuttingCycleInProgress(bool value);

bool getContinuousModeActive();
void setContinuousModeActive(bool value);

bool getStartSwitchSafe();
void setStartSwitchSafe(bool value);

// Timer access functions
unsigned long getLastBlinkTime();
void setLastBlinkTime(unsigned long value);

unsigned long getLastErrorBlinkTime();
void setLastErrorBlinkTime(unsigned long value);

unsigned long getErrorStartTime();
void setErrorStartTime(unsigned long value);

// LED state access functions
bool getBlinkState();
void setBlinkState(bool value);

bool getErrorBlinkState();
void setErrorBlinkState(bool value);

// Rotation servo timing access functions
unsigned long getRotationServoActiveStartTime();
void setRotationServoActiveStartTime(unsigned long value);

bool getRotationServoIsActiveAndTiming();
void setRotationServoIsActiveAndTiming(bool value);

// Rotation servo safety delay access functions
bool getRotationServoSafetyDelayActive();
void setRotationServoSafetyDelayActive(bool value);

unsigned long getRotationServoSafetyDelayStartTime();
void setRotationServoSafetyDelayStartTime(unsigned long value);

unsigned long getRotationClampExtendTime();
void setRotationClampExtendTime(unsigned long value);

bool getRotationClampIsExtended();
void setRotationClampIsExtended(bool value);

// Signal timing access functions
unsigned long getSignalTAStartTime();
void setSignalTAStartTime(unsigned long value);

bool getSignalTAActive();
void setSignalTAActive(bool value);

// Yeswood counter access functions
int getConsecutiveYeswoodCount();
void incrementConsecutiveYeswoodCount();
void resetConsecutiveYeswoodCount();

// Utility functions
void printStateChange();
void updateSwitches();
void handleCommonOperations();

// Error state handling functions
void handleStandardErrorState();
void handleErrorResetState();
void handleSuctionErrorState();

#endif // STATE_MANAGER_H 