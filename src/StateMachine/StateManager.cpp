#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "StateMachine/00_STARTUP.h"
#include "StateMachine/ErrorHandlers.h"
#include "StateMachine/11_ERROR_RESET.h"
#include "StateMachine/09_SUCTION_ERROR.h"
#include "StateMachine/10_CUT_MOTOR_ERROR.h"
#include "WebSocketDashboard/websocket_dashboard.h"
#include "Config/Config.h"

// Forward declaration (in case header resolution fails)
int getCurrentConfigMode();

// External references to Bounce objects from main.cpp
extern Bounce cutHomingSwitch;
extern Bounce feedHomingSwitch;
extern Bounce reloadSwitch;
extern Bounce startCycleSwitch;
extern Bounce pushwoodForwardSwitch;
extern Bounce suctionSensorBounce;
extern Bounce woodPresentSensorBounce;

// External references to global variables from main.cpp
extern bool comingFromNoWoodWithSensorsClear;

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
void executeReloadState();

// Forward declarations for state lifecycle functions
void onEnterStartupState();
void onEnterHomingState();
void onEnterIdleState();
void onEnterFeedFirstCutState();
void onEnterFeedWoodFwdOneState();
void onEnterCuttingState();
void onEnterReturningYes2x4State();
void onEnterReturningNo2x4State();
void onEnterReloadState();

void onExitStartupState();
void onExitHomingState();
void onExitIdleState();
void onExitFeedFirstCutState();
void onExitFeedWoodFwdOneState();
void onExitCuttingState();
void onExitReturningYes2x4State();
void onExitReturningNo2x4State();
void onExitReloadState();

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
        case RELOAD:
            executeReloadState();
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
            case RELOAD: onExitReloadState(); break;
            // Error states don't have onExit handlers
            default: break;
        }
        
        previousState = currentState;
        currentState = newState;
        
        // Notify dashboard of state change
        onStateChange(newState);
        
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
            case RELOAD: onEnterReloadState(); break;
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

Bounce* getWoodPresentSensorBounce() {
    return &woodPresentSensorBounce;
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

bool getRotationServoActive() {
    return rotationServoActive;
}

void setRotationServoActive(bool value) {
    rotationServoActive = value;
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
    return taSignalActive;
}

void setSignalTAActive(bool value) {
    taSignalActive = value;
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
    woodPresentSensorBounce.update();
}

void handleCommonOperations() {
    // Update all switches first
    updateSwitches();

    // Flip rotationServoKnownHome true once the post-command travel buffer elapses.
    updateRotationServoHomeStatus();
    
    // Check for cut motor hitting home sensor during RETURNING_YES_2x4 return
    extern bool cutMotorInReturningYes2x4Return; // This global flag is still in main.cpp
    if (cutMotorInReturningYes2x4Return && cutMotor && cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
        //serial.println("Cut motor hit homing sensor during RETURNING_YES_2x4 return - stopping immediately!");
        cutMotor->forceStopAndNewPosition(0);  // Stop immediately and set position to 0
        delay(50); // Allow sensor to settle after force-stop to prevent false negative verification
    }
    // Handle rotation servo return logic
    // After cooldown period, monitor suction sensor for Transfer Arm grabbing wood
    // Transfer Arm is a separate machine that grabs the cut wood diamond and transfers it to Stage 2
    // The suction sensor detects when the Transfer Arm suction has grabbed (HIGH) or released (LOW) the wood
    // Only monitor suction sensor when in active cutting/returning states (not in error states)
    if (rotationServoActive && 
        millis() - rotationServoActiveStartTime >= ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS &&
        currentState != SUCTION_ERROR && 
        currentState != ERROR && 
        currentState != ERROR_RESET &&
        currentState != Cut_Motor_Homing_Error) {
        extern const int WOOD_SUCTION_CONFIRM_SENSOR; // This is in main.cpp
        
        static unsigned long suctionHighDetectedTime = 0;
        static bool waitingForSuctionDelay = false;
        static bool cooldownEntered = false;
        
        // Reset waiting flags when entering cooldown period for first time
        if (!cooldownEntered) {
            suctionHighDetectedTime = 0;
            waitingForSuctionDelay = false;
            cooldownEntered = true;
        }
        
        // Check if suction sensor reads HIGH (Transfer Arm suction grabbed wood)
        // Sensor only works when servo is at active position
        // HIGH = Transfer Arm suction grabbed wood (active), LOW = Transfer Arm suction not active
        // Using debounced reading with 15ms debounce time
        if (suctionSensorBounce.read() == HIGH && !rotationServoReturnCompleted) {
            if (!waitingForSuctionDelay) {
                // First time detecting HIGH - start timer
                suctionHighDetectedTime = millis();
                waitingForSuctionDelay = true;
                String message = "Transfer Arm suction grabbed wood - waiting before returning servo to home.";
                addSerialLog(message);
            } else {
                // Continuously check that sensor remains HIGH - reset timer if it goes LOW
                if (suctionSensorBounce.read() == LOW) {
                    // Sensor went LOW during wait - reset timer
                    suctionHighDetectedTime = millis();
                    String message = "Suction sensor went LOW during wait - timer reset.";
                    addSerialLog(message);
                }
                
                // Check if wait duration has passed (faster return after transfer arm grabs wood)
                if (millis() - suctionHighDetectedTime >= ROTATION_SERVO_SUCTION_HOLD_DURATION_MS) {
                    // Wait duration has passed continuously - return servo to home
                    String message = "Transfer Arm suction grabbed wood after " + String(millis() - rotationServoActiveStartTime) + "ms - returning rotation servo to home.";
                    addSerialLog(message);
                    returnRotationServoHome();
                    rotationServoActive = false;
                    //! rotationServoKnownHome is handled by the pending-buffer in
                    //! returnRotationServoHome() — it flips true once the travel
                    //! buffer elapses via updateRotationServoHomeStatus().
                    rotationServoReturnCompleted = true; // Mark return as completed to prevent repeated calls
                    waitingForSuctionDelay = false; // Reset for next cycle
                    cooldownEntered = false; // Reset for next cycle
                }
            }
        } else if (suctionSensorBounce.read() == LOW) {
            // Suction sensor still reads LOW - Transfer Arm suction not active yet, continue waiting
            waitingForSuctionDelay = false; // Reset if sensor goes LOW
            static unsigned long suctionWaitDebugTime = 0;
            if (millis() - suctionWaitDebugTime >= 500) {
                String message = "Waiting for Transfer Arm suction to grab wood - sensor still LOW after " + String(millis() - rotationServoActiveStartTime) + "ms";
                addSerialLog(message);
                suctionWaitDebugTime = millis();
            }
        }
    }

    // Handle Rotation Clamp retraction after configured duration (later for NO_2x4 state)
    // Check if wood present to determine if we'll need extra time
    // Only retract if either: 1) wood present (normal timing), or 2) no wood AND extra time has passed
    if (rotationClampIsExtended) {
        unsigned long rotationClampRetractDelay = ROTATION_CLAMP_EXTEND_DURATION_MS;
        
        // In Minis mode (config mode 1), add an additional fixed 50ms
        // so the catcher stays engaged longer than in 3 Inch mode.
        if (getCurrentConfigMode() == 1) {
            rotationClampRetractDelay += 50;
        }
        
        // Add extra delay if no wood detected (applies during CUTTING and RETURNING_NO_2x4)
        if (!get2x4Present()) {
            extern unsigned long ROTATION_CLAMP_NO2X4_EXTRA_DELAY_MS; // From 05_RETURNING_No_2x4.cpp
            rotationClampRetractDelay += ROTATION_CLAMP_NO2X4_EXTRA_DELAY_MS; // Extra time for NO_2x4 scenario
        }
        
        if (millis() - rotationClampExtendTime >= rotationClampRetractDelay) {
            retractRotationClamp();
            //serial.println("Rotation Clamp retracted after configured duration.");
        }
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
    // Note: Start delay is now handled inside handleTASignalTiming() which is called from general functions if needed,
    // but here we just need to ensure the main state manager loop calls it or handles the logic.
    // The previous implementation had logic here. Let's update it to support the delay.
    
    extern bool taSignalDelayPending;
    extern unsigned long taSignalDelayStartTime;
    
    // Check start delay
    if (taSignalDelayPending) {
        if (millis() - taSignalDelayStartTime >= 500) {
            // Delay finished, activate signal
            digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
            signalTAStartTime = millis();
            taSignalActive = true;
            taSignalDelayPending = false;
        }
    }
    
    // Check signal duration
    if (taSignalActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
        extern const int TRANSFER_ARM_SIGNAL_PIN; // This is in main.cpp
        digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
        taSignalActive = false;
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

