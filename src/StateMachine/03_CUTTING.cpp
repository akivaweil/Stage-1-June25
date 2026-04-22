#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "Config/Config.h"
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

// LED Wave Pattern (cutting-state local; not a system-wide config)
const float NO_WOOD_LED_WAVE_SPEED_MULTIPLIER = 5.0f;             // How much slower to blink vs RETURNING_NO_2x4 state when no wood detected during cut

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 📊 STATE VARIABLES ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
namespace {
    // Phase of the suction-retry state machine. Mutually exclusive — replaces
    // the three separate inSuctionRetryPhase1/2/Gap booleans, so a single
    // assignment transitions phases without having to clear the others.
    enum class SuctionRetryPhase {
        Idle,       // No retry in progress
        Phase1,     // First wait before issuing TA signal
        Phase2,     // TA signal fired; waiting to decide whether to retry again
        Gap,        // TA line forced LOW between the two retry pulses
    };

    struct CuttingStateContext {
        int step = 0;
        bool rotationClampActivated = false;
        bool rotationServoActivated = false;
        bool transferArmSignalSent = false;
        bool servoReturnStarted = false;
        bool clampsExtended = false;
        bool waitingForSuction = false;
        unsigned long suctionWaitStartTime = 0;
        SuctionRetryPhase suctionRetryPhase = SuctionRetryPhase::Idle;
        unsigned long suctionRetryTimer = 0;
        bool suctionRetryInProgress = false;
        bool suctionRetrySucceeded = false;
        unsigned long suctionRetrySuccessTime = 0;
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
            // Show LED wave pattern when no wood is present (5x slower than RETURNING_NO_2x4 state)
            handleNoWoodLedWavePattern(NO_WOOD_LED_WAVE_SPEED_MULTIPLIER);
        }
    }
}

// Checks if wood is properly grabbed by transfer arm
bool suctionConfirmed() {
    Bounce* suctionSensor = getSuctionSensorBounce();
    return (suctionSensor && suctionSensor->read() == HIGH);
}

// Checks if wood is present in the system
bool isWoodPresent() {
    Bounce* woodPresentSensor = getWoodPresentSensorBounce();
    return (woodPresentSensor && woodPresentSensor->read() == LOW);
}

void configureCutMotorForCurrentCut() {
    if (isWoodPresent()) {
        configureCutMotorForCutting();
    } else {
        configureCutMotorForCuttingSlow();
    }
}

void enterSuctionError(FastAccelStepper* cutMotor) {
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
        returnRotationServoHome();
        
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

    //! Extend clamps to secure wood - once per CUTTING entry, not every tick.
    //! Retractions only happen on state exit / reload interrupt, and resetCuttingSteps
    //! on entry zeroes the flag, so this is safe and removes redundant digitalWrites.
    if (!cuttingContext.clampsExtended) {
        extend2x4SecureClamp();
        extendFeedClamp();
        cuttingContext.clampsExtended = true;
    }

    //! If retry was in progress and sensor just went HIGH, enforce minimum wait before proceeding
    if (cuttingContext.suctionRetryInProgress && suctionConfirmed()) {
        cuttingContext.suctionRetryInProgress = false;
        cuttingContext.suctionRetrySucceeded = true;
        cuttingContext.suctionRetrySuccessTime = millis();
        //! Drop phase state now so a sensor flutter during the grace period restarts
        //! the retry clock cleanly instead of jumping back into an expired phase
        cuttingContext.suctionRetryPhase = SuctionRetryPhase::Idle;
        cuttingContext.suctionRetryTimer = 0;
        cuttingContext.waitingForSuction = false;
        cuttingContext.suctionWaitStartTime = 0;
    }
    if (cuttingContext.suctionRetrySucceeded) {
        if (millis() - cuttingContext.suctionRetrySuccessTime < SUCTION_RETRY_SUCCESS_WAIT_MS) {
            return; // Hold for minimum 1 second after retry success
        }
        cuttingContext.suctionRetrySucceeded = false;
        // Fall through - sensor is HIGH so outer if below will be skipped and we proceed
    }

    //! Check suction sensor before starting cut motor
    //! Gate is the physical WOOD_SUCTION_CONFIRM_SENSOR only — no software-flag
    //! bypass, so a reset with wood still presented can't sneak past this check
    //! and command the servo home into a stuck piece.
    if (!suctionConfirmed()) {
        // Sensor is LOW - start waiting if not already waiting
        if (!cuttingContext.waitingForSuction) {
            cuttingContext.waitingForSuction = true;
            cuttingContext.suctionWaitStartTime = millis();
        }

        // Check if timeout has expired
        if (millis() - cuttingContext.suctionWaitStartTime >= SUCTION_WAIT_TIMEOUT_MS) {
            switch (cuttingContext.suctionRetryPhase) {
                case SuctionRetryPhase::Idle:
                    // Start Phase 1
                    cuttingContext.suctionRetryPhase = SuctionRetryPhase::Phase1;
                    cuttingContext.suctionRetryInProgress = true;
                    cuttingContext.suctionRetryTimer = millis();
                    return; // Stay in Step 0

                case SuctionRetryPhase::Phase1:
                    if (millis() - cuttingContext.suctionRetryTimer >= SUCTION_RETRY_PHASE1_WAIT_MS) {
                        // Phase 1 complete, send TA signal and start Phase 2
                        sendSignalToTA();
                        cuttingContext.suctionRetryPhase = SuctionRetryPhase::Phase2;
                        cuttingContext.suctionRetryTimer = millis();
                    }
                    return; // Stay in Step 0

                case SuctionRetryPhase::Phase2:
                    if (millis() - cuttingContext.suctionRetryTimer >= SUCTION_RETRY_PHASE2_WAIT_MS) {
                        //! Force the TA line LOW so the second pulse is a distinct rising edge.
                        //! Without this the first 5s pulse is still HIGH when the second call fires,
                        //! and the TA only sees one long merged pulse instead of two triggers.
                        digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
                        taSignalActive = false;
                        taSignalDelayPending = false;
                        cuttingContext.suctionRetryPhase = SuctionRetryPhase::Gap;
                        cuttingContext.suctionRetryTimer = millis();
                    }
                    return; // Stay in Step 0

                case SuctionRetryPhase::Gap:
                    if (millis() - cuttingContext.suctionRetryTimer >= SUCTION_RETRY_INTER_PULSE_GAP_MS) {
                        // Gap complete - send the second TA signal, then fail
                        sendSignalToTA();
                        cuttingContext.suctionRetryPhase = SuctionRetryPhase::Idle;
                        FastAccelStepper* cutMotor = getCutMotor();
                        enterSuctionError(cutMotor);
                    }
                    return; // Stay in Step 0
            }
        }

        // Still within timeout - stay in Step 0 and keep checking
        return;
    }

    //! Sensor is HIGH (or went HIGH during wait) - clear waiting flags and proceed
    cuttingContext.waitingForSuction = false;
    cuttingContext.suctionWaitStartTime = 0;
    cuttingContext.suctionRetryPhase = SuctionRetryPhase::Idle;
    cuttingContext.suctionRetryTimer = 0;
    cuttingContext.suctionRetryInProgress = false;
    cuttingContext.suctionRetrySucceeded = false;
    cuttingContext.suctionRetrySuccessTime = 0;

    //! Ensure the rotation servo is home before moving the cut motor.
    //! rotationServoKnownHome is the single source of truth:
    //!   - false on boot (servo position unknown → always home-and-wait first cut)
    //!   - cleared when activateRotationServo() moves the servo to ACTIVE
    //!   - flipped true ROTATION_SERVO_HOME_WAIT_DURATION_MS after any
    //!     returnRotationServoHome() call, via updateRotationServoHomeStatus()
    //! Step 0 issues the return command once per entry (if needed) and simply
    //! waits across ticks until the flag flips.
    if (!rotationServoKnownHome) {
        if (!cuttingContext.servoReturnStarted) {
            returnRotationServoHome();
            rotationServoActive = false;
            cuttingContext.servoReturnStarted = true;
        }
        return; // wait for the travel buffer to elapse
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
    
    activateComponentAtPosition(cuttingContext.transferArmSignalSent, 
                               TA_SIGNAL_OFFSET_FROM_END,
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

int getCuttingStateStep() {
    return cuttingContext.step;
}
