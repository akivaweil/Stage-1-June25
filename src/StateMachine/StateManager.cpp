#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/00_STARTUP.h"
#include "StateMachine/STATES/States_Config.h"
#include "ErrorStates/Errors_Functions.h"
#include "ErrorStates/Error_Reset.h"
#include "ErrorStates/Suction_Error.h"
#include "ErrorStates/Cut_Motor_Error.h"

// External references to Bounce objects from main.cpp
extern Bounce cutHomingSwitch;
extern Bounce feedHomingSwitch;
extern Bounce reloadSwitch;
extern Bounce startCycleSwitch;
extern Bounce pushwoodForwardSwitch;
extern Bounce suctionSensorBounce;

//* ************************************************************************
//* ************************* STATE MANAGER *******************************
//* ************************************************************************
// Function-based state manager implementation that coordinates all state operations.

// Global variables for state management
static int consecutiveYeswoodCount = 0;
static SystemState previousState = STARTUP;

// Forward declarations for state execution functions
void executeStartupState();
void executeHomingState();
void executeIdleState();
void executeFeedFirstCutState();
void executeFeedWoodFwdOneState();
void executeCuttingState();
void executeReturningYes2x4State();
void executeReturningNo2x4State();

// Forward declarations for state lifecycle functions
void onEnterStartupState();
void onEnterHomingState();
void onEnterIdleState();
void onEnterFeedFirstCutState();
void onEnterFeedWoodFwdOneState();
void onEnterCuttingState();
void onEnterReturningYes2x4State();
void onEnterReturningNo2x4State();

void onExitStartupState();
void onExitHomingState();
void onExitIdleState();
void onExitFeedFirstCutState();
void onExitFeedWoodFwdOneState();
void onExitCuttingState();
void onExitReturningYes2x4State();
void onExitReturningNo2x4State();

void executeStateMachine() {
    handleCommonOperations();
    
    // Update error LED blinking for ERROR state
    if (currentState == ERROR) {
        handleErrorLedBlink();
    }
    
    switch (currentState) {
        case STARTUP:
            executeStartupState();
            break;
        case HOMING:
            executeHomingState();
            break;
        case IDLE:
            executeIdleState();
            break;
        case FEED_FIRST_CUT:
            executeFeedFirstCutState();
            break;
        case FEED_WOOD_FWD_ONE:
            executeFeedWoodFwdOneState();
            break;
        case CUTTING:
            executeCuttingState();
            break;
        case RETURNING_YES_2x4:
            executeReturningYes2x4State();
            break;
        case RETURNING_NO_2x4:
            executeReturningNo2x4State();
            break;
        case ERROR:
            handleStandardErrorState();
            break;
        case ERROR_RESET:
            handleErrorResetState();
            break;
        case SUCTION_ERROR:
            handleSuctionErrorState();
            break;
        case Cut_Motor_Homing_Error:
            handleCutMotorErrorState();
            break;
    }
}

void changeState(SystemState newState) {
    if (currentState != newState) {
        // Call onExit for the current state before changing
        switch (currentState) {
            case STARTUP: onExitStartupState(); break;
            case HOMING: onExitHomingState(); break;
            case IDLE: onExitIdleState(); break;
            case FEED_FIRST_CUT: onExitFeedFirstCutState(); break;
            case FEED_WOOD_FWD_ONE: onExitFeedWoodFwdOneState(); break;
            case CUTTING: onExitCuttingState(); break;
            case RETURNING_YES_2x4: onExitReturningYes2x4State(); break;
            case RETURNING_NO_2x4: onExitReturningNo2x4State(); break;
            // Error states don't have onExit handlers
            default: break;
        }
        
        previousState = currentState;
        currentState = newState;
        
        // Call onEnter for the new state after changing
        switch (newState) {
            case STARTUP: onEnterStartupState(); break;
            case HOMING: onEnterHomingState(); break;
            case IDLE: onEnterIdleState(); break;
            case FEED_FIRST_CUT: onEnterFeedFirstCutState(); break;
            case FEED_WOOD_FWD_ONE: onEnterFeedWoodFwdOneState(); break;
            case CUTTING: onEnterCuttingState(); break;
            case RETURNING_YES_2x4: onEnterReturningYes2x4State(); break;
            case RETURNING_NO_2x4: onEnterReturningNo2x4State(); break;
            // Error states don't have onEnter handlers
            default: break;
        }
    }
}

//* ************************************************************************
//* ************************* ACCESS FUNCTIONS *****************************
//* ************************************************************************

SystemState getCurrentState() {
    return currentState;
}

SystemState getPreviousState() {
    return previousState;
}

FastAccelStepper* getCutMotor() {
    return cutMotor;
}

FastAccelStepper* getFeedMotor() {
    return feedMotor;
}

Servo* getRotationServo() {
    extern Servo rotationServo; // From main.cpp
    return &rotationServo;
}

Bounce* getCutHomingSwitch() {
    return &cutHomingSwitch;
}

Bounce* getFeedHomingSwitch() {
    return &feedHomingSwitch;
}

Bounce* getReloadSwitch() {
    return &reloadSwitch;
}

Bounce* getStartCycleSwitch() {
    return &startCycleSwitch;
}

Bounce* getSuctionSensorBounce() {
    return &suctionSensorBounce;
}

bool getIsReloadMode() {
    return isReloadMode;
}

void setIsReloadMode(bool value) {
    isReloadMode = value;
}

bool get2x4Present() {
    return _2x4Present;
}

void set2x4Present(bool value) {
    _2x4Present = value;
}

bool getWoodSuctionError() {
    return woodSuctionError;
}

void setWoodSuctionError(bool value) {
    woodSuctionError = value;
}

bool getErrorAcknowledged() {
    return errorAcknowledged;
}

void setErrorAcknowledged(bool value) {
    errorAcknowledged = value;
}

bool getCuttingCycleInProgress() {
    return cuttingCycleInProgress;
}

void setCuttingCycleInProgress(bool value) {
    cuttingCycleInProgress = value;
}

bool getContinuousModeActive() {
    return continuousModeActive;
}

void setContinuousModeActive(bool value) {
    continuousModeActive = value;
}

bool getStartSwitchSafe() {
    return startSwitchSafe;
}

void setStartSwitchSafe(bool value) {
    startSwitchSafe = value;
}

unsigned long getLastBlinkTime() {
    return lastBlinkTime;
}

void setLastBlinkTime(unsigned long value) {
    lastBlinkTime = value;
}

unsigned long getLastErrorBlinkTime() {
    return lastErrorBlinkTime;
}

void setLastErrorBlinkTime(unsigned long value) {
    lastErrorBlinkTime = value;
}

unsigned long getErrorStartTime() {
    return errorStartTime;
}

void setErrorStartTime(unsigned long value) {
    errorStartTime = value;
}

bool getBlinkState() {
    return blinkState;
}

void setBlinkState(bool value) {
    blinkState = value;
}

bool getErrorBlinkState() {
    return errorBlinkState;
}

void setErrorBlinkState(bool value) {
    errorBlinkState = value;
}

unsigned long getRotationServoActiveStartTime() {
    return rotationServoActiveStartTime;
}

void setRotationServoActiveStartTime(unsigned long value) {
    rotationServoActiveStartTime = value;
}

bool getRotationServoIsActiveAndTiming() {
    return rotationServoIsActiveAndTiming;
}

void setRotationServoIsActiveAndTiming(bool value) {
    rotationServoIsActiveAndTiming = value;
}

bool getRotationServoSafetyDelayActive() {
    extern bool rotationServoSafetyDelayActive; // From main.cpp
    return rotationServoSafetyDelayActive;
}

void setRotationServoSafetyDelayActive(bool value) {
    extern bool rotationServoSafetyDelayActive; // From main.cpp
    rotationServoSafetyDelayActive = value;
}

unsigned long getRotationServoSafetyDelayStartTime() {
    extern unsigned long rotationServoSafetyDelayStartTime; // From main.cpp
    return rotationServoSafetyDelayStartTime;
}

void setRotationServoSafetyDelayStartTime(unsigned long value) {
    extern unsigned long rotationServoSafetyDelayStartTime; // From main.cpp
    rotationServoSafetyDelayStartTime = value;
}

// Rotation servo return delay timing functions
unsigned long getRotationServoReturnDelayStartTime() {
    extern unsigned long rotationServoReturnDelayStartTime; // From main.cpp
    return rotationServoReturnDelayStartTime;
}

void setRotationServoReturnDelayStartTime(unsigned long value) {
    extern unsigned long rotationServoReturnDelayStartTime; // From main.cpp
    rotationServoReturnDelayStartTime = value;
}

unsigned long getRotationClampExtendTime() {
    return rotationClampExtendTime;
}

void setRotationClampExtendTime(unsigned long value) {
    rotationClampExtendTime = value;
}

bool getRotationClampIsExtended() {
    return rotationClampIsExtended;
}

void setRotationClampIsExtended(bool value) {
    rotationClampIsExtended = value;
}

unsigned long getSignalTAStartTime() {
    return signalTAStartTime;
}

void setSignalTAStartTime(unsigned long value) {
    signalTAStartTime = value;
}

bool getSignalTAActive() {
    return signalTAActive;
}

void setSignalTAActive(bool value) {
    signalTAActive = value;
}

int getConsecutiveYeswoodCount() {
    return consecutiveYeswoodCount;
}

void incrementConsecutiveYeswoodCount() {
    consecutiveYeswoodCount++;
}

void resetConsecutiveYeswoodCount() {
    consecutiveYeswoodCount = 0;
}

//* ************************************************************************
//* ************************* UTILITY FUNCTIONS ****************************
//* ************************************************************************

void printStateChange() {
    if (currentState != previousState) {
        // Serial.print("Current State: ");
        // switch (currentState) {
        //     case STARTUP: Serial.println("STARTUP"); break;
        //     case HOMING: Serial.println("HOMING"); break;
        //     case IDLE: Serial.println("IDLE"); break;
        //     case FEED_FIRST_CUT: Serial.println("FEED_FIRST_CUT"); break;
        //     case FEED_WOOD_FWD_ONE: Serial.println("FEED_WOOD_FWD_ONE"); break;
        //     case CUTTING: Serial.println("CUTTING"); break;
        //     case RETURNING_YES_2x4: Serial.println("RETURNING_YES_2x4"); break;
        //     case RETURNING_NO_2x4: Serial.println("RETURNING_NO_2x4"); break;
        //     case ERROR: Serial.println("ERROR"); break;
        //     case ERROR_RESET: Serial.println("ERROR_RESET"); break;
        //     case SUCTION_ERROR: Serial.println("SUCTION_ERROR"); break;
        //     case Cut_Motor_Homing_Error: Serial.println("Cut_Motor_Homing_Error"); break;
        //     default: Serial.println("UNKNOWN"); break;
        // }
        previousState = currentState;
    }
}

void updateSwitches() {
    // Update all debounced switches - moved from main loop
    cutHomingSwitch.update();
    feedHomingSwitch.update();
    reloadSwitch.update();
    startCycleSwitch.update();
    pushwoodForwardSwitch.update();
    suctionSensorBounce.update();
}

void handleCommonOperations() {
    // Update all switches first
    updateSwitches();
    
    // Check for cut motor hitting home sensor during RETURNING_YES_2x4 return
    extern bool cutMotorInReturningYes2x4Return; // This global flag is still in main.cpp
    if (cutMotorInReturningYes2x4Return && cutMotor && cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
        //serial.println("Cut motor hit homing sensor during RETURNING_YES_2x4 return - stopping immediately!");
        cutMotor->forceStopAndNewPosition(0);  // Stop immediately and set position to 0
    }
    // Handle rotation servo return with safety delay logic
    if (rotationServoIsActiveAndTiming && millis() - rotationServoActiveStartTime >= ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS) {
        extern const int WOOD_SUCTION_CONFIRM_SENSOR; // This is in main.cpp
        
        // Check if suction sensor reads HIGH (wood is properly grabbed by transfer arm)
        // Using debounced reading with 15ms debounce time
        if (suctionSensorBounce.read() == HIGH) {
            // Check if we've been waiting for more than 3 seconds due to failure to suction
            if (millis() - rotationServoActiveStartTime >= ROTATION_SERVO_EXTENDED_WAIT_THRESHOLD_MS) {
                // We've been waiting for extended time due to failure to suction - apply safety delay
                extern bool rotationServoSafetyDelayActive; // From main.cpp
                extern unsigned long rotationServoSafetyDelayStartTime; // From main.cpp
                if (!rotationServoSafetyDelayActive) {
                    // Start the safety delay period
                    rotationServoSafetyDelayActive = true;
                    rotationServoSafetyDelayStartTime = millis();
                    //serial.println("Servo was waiting for extended time due to failure to suction. Starting 2-second safety delay before returning to home.");
                } else if (millis() - rotationServoSafetyDelayStartTime >= ROTATION_SERVO_SAFETY_DELAY_MS) {
                    // Safety delay complete - now start return delay
                    extern bool rotationServoReturnDelayActive; // From main.cpp
                    extern unsigned long rotationServoReturnDelayStartTime; // From main.cpp
                    if (!rotationServoReturnDelayActive) {
                        // Start the return delay period
                        rotationServoReturnDelayActive = true;
                        rotationServoReturnDelayStartTime = millis();
                        //serial.println("Safety delay complete. Starting 150ms return delay before returning servo to home.");
                    } else if (millis() - rotationServoReturnDelayStartTime >= ROTATION_SERVO_RETURN_DELAY_MS) {
                        // Return delay complete - now return servo to home
                        handleRotationServoReturn();
                        //serial.println("Return delay complete. Returning rotation servo to home position.");
                        rotationServoIsActiveAndTiming = false;
                        rotationServoSafetyDelayActive = false; // Reset safety delay flag
                        rotationServoReturnDelayActive = false; // Reset return delay flag
                    }
                } else {
                    // Still in safety delay period
                    static unsigned long lastSafetyMessage = 0;
                    if (millis() - lastSafetyMessage >= 500) { // Update message every 500ms
                        unsigned long remainingMs = ROTATION_SERVO_SAFETY_DELAY_MS - (millis() - rotationServoSafetyDelayStartTime);
                                //Serial.print("Safety delay active. Servo will return home in ");
        //Serial.print(remainingMs);
        //serial.println("ms to allow operator to move hand away.");
                        lastSafetyMessage = millis();
                    }
                }
            } else {
                // Normal operation - no extended wait, but still apply 150ms return delay
                extern bool rotationServoReturnDelayActive; // From main.cpp
                extern unsigned long rotationServoReturnDelayStartTime; // From main.cpp
                if (!rotationServoReturnDelayActive) {
                    // Start the return delay period
                    rotationServoReturnDelayActive = true;
                    rotationServoReturnDelayStartTime = millis();
                    //serial.println("Starting 150ms return delay before returning servo to home.");
                } else if (millis() - rotationServoReturnDelayStartTime >= ROTATION_SERVO_RETURN_DELAY_MS) {
                    // Return delay complete - now return servo to home
                    handleRotationServoReturn();
                    //serial.println("Return delay complete. Returning rotation servo to home position.");
                    rotationServoIsActiveAndTiming = false;
                    rotationServoReturnDelayActive = false; // Reset return delay flag
                }
            }
        } else {
            // Suction sensor still reads LOW - wood not grabbed by transfer arm, continue waiting
            //serial.println("Waiting for WAS_WOOD_SUCTIONED_SENSOR to read HIGH before returning rotation servo...");
        }
    }

    // Handle Rotation Clamp retraction after 1 second
    if (rotationClampIsExtended && (millis() - rotationClampExtendTime >= ROTATION_CLAMP_EXTEND_DURATION_MS)) {
        retractRotationClamp();
        //serial.println("Rotation Clamp retracted after 1 second.");
    }

    // 2x4 sensor - Update global _2x4Present flag
    extern const int _2x4_PRESENT_SENSOR; // This is in main.cpp
    _2x4Present = (digitalRead(_2x4_PRESENT_SENSOR) == LOW);
    
    // Handle start switch safety check
    if (!startSwitchSafe && startCycleSwitch.fell()) {
        startSwitchSafe = true;
    }
    
    // Handle error acknowledgment separately
    if (reloadSwitch.rose() && currentState == ERROR) {
        changeState(ERROR_RESET);
        errorAcknowledged = true;
    }
    
    // Check for continuous mode activation/deactivation - modified to include safety check
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
    }
    
    // Handle TA signal timeout after TA_SIGNAL_DURATION
    if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
        extern const int TRANSFER_ARM_SIGNAL_PIN; // This is in main.cpp
        digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
        signalTAActive = false;
        //serial.println("Signal to Transfer Arm (TA) timed out and reset to LOW"); 
    }
}

//* ************************************************************************
//* ************************* ERROR STATE HANDLERS ************************
//* ************************************************************************

void handleStandardErrorState() {
    // Handle standard error state with basic error LED blinking
    handleErrorLedBlink();
    
    // Check for error acknowledgment
    if (reloadSwitch.rose()) {
        changeState(ERROR_RESET);
        errorAcknowledged = true;
        //serial.println("Standard error acknowledged by reload switch.");
    }
}

void handleErrorResetState() {
    // Call the actual error reset handling function from Error_Reset.cpp
    ::handleErrorResetState();
}

