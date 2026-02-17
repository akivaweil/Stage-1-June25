#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "WebSocketDashboard/websocket_dashboard.h"
#include "OTAUpdater/ota_updater.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚔️ CUTTING STATE ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Handles the wood cutting operation with a clean 4-step process:
// Step 0: Initialize cutting sequence - extend clamps and configure motors
// Step 1: Check suction sensor and start cut motor movement
// Step 2: Monitor cut motor position, activate rotation components, and complete cut
// Step 3: Handle reload switch interrupt return to home
// 
// After cutting completion, transitions to appropriate RETURNING state based on wood detection.
// All post-cutting logic (return sequences, homing, continuous mode) is handled by RETURNING states.

// Configuration Settings
// Rotation Servo Settings (ROTATION_SERVO_HOME_POSITION set via web dashboard)
int ROTATION_SERVO_ACTIVE_POSITION = 108;                        // Servo active rotation angle
unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2200;     // Duration to hold active position
unsigned long ROTATION_SERVO_RETURN_DELAY_MS = 150;              // Delay before returning to home
unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS = 300;        // Wait dration at home position
unsigned long ROTATION_SERVO_SUCTION_HOLD_DURATION_MS = 300;     // Wait time after suction detected before returning
float ROTATION_SERVO_ACTIVATION_DISTANCE = 8.2;                  // Servo activation distance from start of cut (inches)

// Rotation Clamp Configuration
unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 2200;          // Time for clamp to fully extend 
float ROTATION_CLAMP_ACTIVATION_DISTANCE = 5.75;                  // Clamp activation distance from start of cut (inches)

// Transfer Arm Configuration
unsigned long TA_SIGNAL_DURATION = 2000;                         // Transfer arm signal duration
float TA_SIGNAL_ACTIVATION_DISTANCE = 8.5;                       // TA signal activation distance from start of cut (inches)

// Cut Motor Timing & Recovery
unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000;              // Recovery operation timeout
unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20;              // Motor state verification delay

// Suction Sensor Configuration
unsigned long SUCTION_WAIT_TIMEOUT_MS = 1500;                      // Timeout for waiting for suction sensor to go HIGH (ms)

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 📊 STATE VARIABLES ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
namespace {
    struct CuttingStateContext {
        int step = 0;
        bool rotationClampActivated = false;
        bool rotationServoActivated = false;
        bool transferArmSignalSent = false;
        bool waitingForServoHome = false;
        bool servoReturnStarted = false;
        unsigned long servoHomeWaitStartedAt = 0;
        bool waitingForSuction = false;
        unsigned long suctionWaitStartTime = 0;
    };

    CuttingStateContext cuttingContext;
    bool homePositionErrorDetected = false;
}

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 HELPER FUNCTIONS ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

// Updates LED based on wood present sensor reading
void updateWoodPresentLed() {
    Bounce* woodPresentSensor = getWoodPresentSensorBounce();
    if (woodPresentSensor) {
        bool woodPresent = (woodPresentSensor->read() == LOW);
        if (woodPresent) {
            showYellowLed();
        } else {
            // Show LED wave pattern when no wood is present
            handleNoWoodLedWavePattern();
        }
    }
}

// Checks if wood is properly grabbed by transfer arm
bool isWoodProperlyGrabbed() {
    Bounce* suctionSensor = getSuctionSensorBounce();
    return (suctionSensor && suctionSensor->read() == HIGH);
}

// Checks if wood is present in the system
bool isWoodPresent() {
    Bounce* woodPresentSensor = getWoodPresentSensorBounce();
    return (woodPresentSensor && woodPresentSensor->read() == LOW);
}

// Waits for rotation servo to return home before starting cut
bool waitForServoHomeIfNeeded() {
    if (!isWoodProperlyGrabbed()) {
        cuttingContext.waitingForServoHome = false;
        return false;
    }

    extern bool rotationServoIsActiveAndTiming;

    if (rotationServoIsActiveAndTiming && !cuttingContext.waitingForServoHome) {
        cuttingContext.waitingForServoHome = true;
        cuttingContext.servoHomeWaitStartedAt = millis();
        return true;
    }

    if (cuttingContext.waitingForServoHome) {
        if (millis() - cuttingContext.servoHomeWaitStartedAt < ROTATION_SERVO_HOME_WAIT_DURATION_MS) {
            return true;
        }
        cuttingContext.waitingForServoHome = false;
    }

    return false;
}

void configureCutMotorForCurrentCut() {
    if (isWoodPresent()) {
        configureCutMotorForCutting();
    } else {
        configureCutMotorForCuttingSlow();
    }
}

void handleSuctionFailure(FastAccelStepper* cutMotor) {
    FastAccelStepper* feedMotor = getFeedMotor();
    if (feedMotor && feedMotor->isRunning()) {
        feedMotor->stopMove();
    }

    if (cutMotor) {
        configureCutMotorForReturn();
        moveCutMotorToHome();
    }

    setCuttingCycleInProgress(false);
    onErrorOccurred("Wood suction not confirmed");
    changeState(SUCTION_ERROR);
    resetCuttingSteps();
}

void startCutMotorReturnSequence() {
    configureCutMotorForReturn();
    moveCutMotorToHome();
    cuttingContext.transferArmSignalSent = false;
}

void updateLedsForReturnState(bool no2x4Detected) {
    if (no2x4Detected) {
        showBlueLed();
        turnYellowLedOff();
    } else {
        showYellowLed();
        turnBlueLedOff();
    }
}

// Activates a component when cut motor reaches a position that is
// a specified distance BEFORE the end of the cut travel
void activateComponentAtPosition(bool& activatedFlag, float activationOffsetInches, 
                                 void (*activationFunction)()) {
    if (activatedFlag) return;
    
    FastAccelStepper* cutMotor = getCutMotor();
    if (!cutMotor) return;
    
    extern float getCutTravelDistance();
    long activationSteps = (getCutTravelDistance() - activationOffsetInches) * CUT_MOTOR_STEPS_PER_INCH;
    
    if (cutMotor->getCurrentPosition() >= activationSteps) {
        activationFunction();
        activatedFlag = true;
    }
}

// Activates a component when cut motor has traveled a specified
// distance FROM THE START (home) position
void activateComponentAtDistanceFromStart(bool& activatedFlag, float activationDistanceInches,
                                          void (*activationFunction)()) {
    if (activatedFlag) return;

    FastAccelStepper* cutMotor = getCutMotor();
    if (!cutMotor) return;

    long activationSteps = activationDistanceInches * CUT_MOTOR_STEPS_PER_INCH;

    if (cutMotor->getCurrentPosition() >= activationSteps) {
        activationFunction();
        activatedFlag = true;
    }
}

void onEnterCuttingState() {
    resetCuttingSteps();
    startCuttingCycleTimer();
    stopReloadTimer(); // Stop reload time tracking when entering cutting state
    
    // Check wood presence before resetting to preserve yellow LED if wood is present
    bool woodPresent = isWoodPresent();
    resetNoWoodLedWavePattern(woodPresent); // Reset LED wave pattern, preserving yellow LED if wood is present
}

void onExitCuttingState() {
    resetCuttingSteps();
}

void executeCuttingState() {
    // Check if reload switch is activated - if so, return motor to home and transition to reload state
    if (getReloadSwitch()->read() == HIGH && cuttingContext.step != 3) {
        FastAccelStepper* cutMotor = getCutMotor();
        FastAccelStepper* feedMotor = getFeedMotor();
        
        if (cutMotor) cutMotor->stopMove();
        if (feedMotor) feedMotor->stopMove();
        
        // Retract secondary components
        retractRotationClamp();
        handleRotationServoReturn();
        
        // Initiate return
        startCutMotorReturnSequence();
        
        cuttingContext.step = 3;
    }

    if (homePositionErrorDetected) {
        handleHomePositionError();
        return;
    }

    switch (cuttingContext.step) {
        case 0: 
            handleCuttingStep0();
            break;
        case 1: 
            handleCuttingStep1();
            break;
        case 2: 
            handleCuttingStep2();
            break;
        case 3: 
            handleCuttingStep3();
            break;
        default:
            cuttingContext.step = 0;
            break;
    }
}

void handleCuttingStep0() {
    //! Check for OTA upload at the beginning of cutting state
    handleOTA();
    
    //! Wait for rotation servo to return home if needed
    if (waitForServoHomeIfNeeded()) {
        return; // Still waiting, exit and check again next cycle
    }
        
    //! Extend clamps to secure wood
    extend2x4SecureClamp();
    extendFeedClamp();

    //! Check suction sensor before starting cut motor
    if (!isWoodProperlyGrabbed()) {
        // Check if servo is home - if so, ignore suction error and proceed
        if (!getRotationServoIsActiveAndTiming()) {
            // Clear waiting flags
            cuttingContext.waitingForSuction = false;
            cuttingContext.suctionWaitStartTime = 0;
            // Proceed to next block (skip return)
        } else {
            // Sensor is LOW - start waiting if not already waiting
            if (!cuttingContext.waitingForSuction) {
                cuttingContext.waitingForSuction = true;
                cuttingContext.suctionWaitStartTime = millis();
            }
            
            // Check if timeout has expired
            if (millis() - cuttingContext.suctionWaitStartTime >= SUCTION_WAIT_TIMEOUT_MS) {
                // Timeout expired - transition to suction error
                FastAccelStepper* cutMotor = getCutMotor();
                handleSuctionFailure(cutMotor);
                return;
            }
            
            // Still within timeout - stay in Step 0 and keep checking
            return;
        }
    }
    
    //! Sensor is HIGH (or went HIGH during wait) - clear waiting flags and proceed
    cuttingContext.waitingForSuction = false;
    cuttingContext.suctionWaitStartTime = 0;

    //! Home rotation servo if wood is properly grabbed (always ensure it's at home position)
    if (isWoodProperlyGrabbed()) {
        extern bool rotationServoIsActiveAndTiming;
        if (!cuttingContext.servoReturnStarted) {
            handleRotationServoReturn();
            cuttingContext.servoReturnStarted = true;
        }
    }

    //! Configure cut motor speed based on wood detection
    configureCutMotorForCurrentCut();
    moveCutMotorToCut();

    cuttingContext.rotationClampActivated = false;
    cuttingContext.rotationServoActivated = false;
    cuttingContext.transferArmSignalSent = false;
    cuttingContext.step = 1;
}

void handleCuttingStep1() {
    //! Update LED based on wood present sensor
    updateWoodPresentLed();

    //! Continue to step 2
    cuttingContext.step = 2;
}

void handleCuttingStep2() {
    //! Update LED based on wood present sensor
    updateWoodPresentLed();
    
    //! Activate components at their respective positions
    activateComponentAtDistanceFromStart(cuttingContext.rotationClampActivated, 
                                         ROTATION_CLAMP_ACTIVATION_DISTANCE,
                                         extendRotationClamp);
    
    activateComponentAtDistanceFromStart(cuttingContext.rotationServoActivated, 
                                         ROTATION_SERVO_ACTIVATION_DISTANCE,
                                         activateRotationServo);
    
    activateComponentAtDistanceFromStart(cuttingContext.transferArmSignalSent, 
                                         TA_SIGNAL_ACTIVATION_DISTANCE,
                                         sendSignalToTA);
    
    //! Check if cut is complete
    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && !cutMotor->isRunning()) {
        startCutMotorReturnSequence();

        const bool no2x4Detected = !isWoodPresent();
        updateLedsForReturnState(no2x4Detected);
        if (no2x4Detected) {
            changeState(RETURNING_NO_2x4);
        } else {
            changeState(RETURNING_YES_2x4);
        }
    }
}

void handleCuttingStep3() {
    //! ************************************************************************
    //! STEP 3: WAIT FOR CUT MOTOR TO REACH HOME BEFORE TRANSITIONING TO RELOAD
    //! ************************************************************************
    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && !cutMotor->isRunning()) {
        getCutHomingSwitch()->update();
        if (getCutHomingSwitch()->read() == HIGH) {
            changeState(RELOAD);
        } else {
            // If motor stopped but not at home, try moving home again
            moveCutMotorToHome();
        }
    }
}


void handleHomePositionError() {
    unsigned long lastErrorBlinkTime = getLastErrorBlinkTime();
    bool errorBlinkState = getErrorBlinkState();
    
    if (millis() - lastErrorBlinkTime > 100) { 
        errorBlinkState = !errorBlinkState;
        setErrorBlinkState(errorBlinkState);
        if(errorBlinkState) showRedLed(); else turnRedLedOff();
        if(!errorBlinkState) showYellowLed(); else turnYellowLedOff();
        setLastErrorBlinkTime(millis());
    }
    
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
    if (feedMotor) feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
    
    extend2x4SecureClamp();
    
    if (getReloadSwitch()->rose()) {
        homePositionErrorDetected = false;
        addEventToLog("Home position error - acknowledged");
        changeState(ERROR_RESET);
        setErrorAcknowledged(true);
    }
}

void resetCuttingSteps() {
    cuttingContext = CuttingStateContext{};
    homePositionErrorDetected = false;
}

bool isCuttingStateStep0() {
    return cuttingContext.step == 0;
}
