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
// Central state manager that coordinates all state operations
// and provides access to system resources.

class StateManager {
public:
    // Constructor
    StateManager();
    
    // Main state machine execution
    void execute();
    
    // State transition management
    void changeState(SystemState newState);
    SystemState getCurrentState() const { return currentState; }
    SystemState getPreviousState() const { return previousState; }
    
    // System resource access methods
    FastAccelStepper* getCutMotor() { return cutMotor; }
    FastAccelStepper* getFeedMotor() { return feedMotor; }
    Servo* getRotationServo() { return &rotationServo; }
    
    // Switch access methods
    Bounce* getCutHomingSwitch() { return &cutHomingSwitch; }
    Bounce* getFeedHomingSwitch() { return &feedHomingSwitch; }
    Bounce* getReloadSwitch() { return &reloadSwitch; }
    Bounce* getStartCycleSwitch() { return &startCycleSwitch; }
    
    // System flag access methods
    bool getIsReloadMode() const { return isReloadMode; }
    void setIsReloadMode(bool value) { isReloadMode = value; }
    
    bool get2x4Present() const { return _2x4Present; }
    void set2x4Present(bool value) { _2x4Present = value; }
    
    bool getWoodSuctionError() const { return woodSuctionError; }
    void setWoodSuctionError(bool value) { woodSuctionError = value; }
    
    bool getErrorAcknowledged() const { return errorAcknowledged; }
    void setErrorAcknowledged(bool value) { errorAcknowledged = value; }
    
    bool getCuttingCycleInProgress() const { return cuttingCycleInProgress; }
    void setCuttingCycleInProgress(bool value) { cuttingCycleInProgress = value; }
    
    bool getContinuousModeActive() const { return continuousModeActive; }
    void setContinuousModeActive(bool value) { continuousModeActive = value; }
    
    bool getStartSwitchSafe() const { return startSwitchSafe; }
    void setStartSwitchSafe(bool value) { startSwitchSafe = value; }
    
    // Timer access methods
    unsigned long getLastBlinkTime() const { return lastBlinkTime; }
    void setLastBlinkTime(unsigned long value) { lastBlinkTime = value; }
    
    unsigned long getLastErrorBlinkTime() const { return lastErrorBlinkTime; }
    void setLastErrorBlinkTime(unsigned long value) { lastErrorBlinkTime = value; }
    
    unsigned long getErrorStartTime() const { return errorStartTime; }
    void setErrorStartTime(unsigned long value) { errorStartTime = value; }
    
    // LED state access methods
    bool getBlinkState() const { return blinkState; }
    void setBlinkState(bool value) { blinkState = value; }
    
    bool getErrorBlinkState() const { return errorBlinkState; }
    void setErrorBlinkState(bool value) { errorBlinkState = value; }
    
    // Rotation servo timing access
    unsigned long getRotationServoActiveStartTime() const { return rotationServoActiveStartTime; }
    void setRotationServoActiveStartTime(unsigned long value) { rotationServoActiveStartTime = value; }
    
    bool getRotationServoIsActiveAndTiming() const { return rotationServoIsActiveAndTiming; }
    void setRotationServoIsActiveAndTiming(bool value) { rotationServoIsActiveAndTiming = value; }
    
    unsigned long getRotationClampExtendTime() const { return rotationClampExtendTime; }
    void setRotationClampExtendTime(unsigned long value) { rotationClampExtendTime = value; }
    
    bool getRotationClampIsExtended() const { return rotationClampIsExtended; }
    void setRotationClampIsExtended(bool value) { rotationClampIsExtended = value; }
    
    // Signal timing access
    unsigned long getSignalTAStartTime() const { return signalTAStartTime; }
    void setSignalTAStartTime(unsigned long value) { signalTAStartTime = value; }
    
    bool getSignalTAActive() const { return signalTAActive; }
    void setSignalTAActive(bool value) { signalTAActive = value; }

private:
    SystemState previousState;
    
    // Print state changes
    void printStateChange();
    
    // Update all switches - moved from main loop
    void updateSwitches();
    
    // Handle common operations that happen every loop
    void handleCommonOperations();
    
    // Error state handling functions
    void handleStandardErrorState();
    void handleErrorResetState();
    void handleSuctionErrorState();
};

// Global state manager instance
extern StateManager stateManager;

#endif // STATE_MANAGER_H 