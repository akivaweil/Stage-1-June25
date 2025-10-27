#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚔️ CUTTING STATE                                                     ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Handles the wood cutting operation with a clean 3-step process:
// Step 0: Initialize cutting sequence - extend clamps and configure motors
// Step 1: Check suction sensor and start cut motor movement
// Step 2: Monitor cut motor position, activate rotation components, and complete cut
// 
// After cutting completion, transitions to appropriate RETURNING state based on wood detection.
// All post-cutting logic (return sequences, homing, continuous mode) is handled by RETURNING states.

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 📊 STATE VARIABLES                                                   ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
static int cuttingStep = 0;
static unsigned long stepStartTime = 0;
static bool homePositionErrorDetected = false;
static bool rotationClampActivatedThisCycle = false;
static bool rotationServoActivatedThisCycle = false;
static bool transferArmSignalSentThisCycle = false;
static unsigned long servoHomeWaitStartTime = 0;
static bool waitingForServoHome = false;
static unsigned long cuttingLastDebugTime = 0;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 HELPER FUNCTIONS                                                  ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

// Updates LED based on wood present sensor reading
void updateWoodPresentLed() {
    Bounce* woodPresentSensor = getWoodPresentSensorBounce();
    if (woodPresentSensor) {
        bool woodPresent = (woodPresentSensor->read() == LOW);
        if (woodPresent) {
            showYellowLed();
        } else {
            showBlueLed();
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
        Serial.println("WARNING: Wood not properly grabbed by transfer arm - skipping servo wait for safety");
        return false; // Don't wait, proceed immediately
    }
    
    extern bool rotationServoIsActiveAndTiming;
    
    if (rotationServoIsActiveAndTiming && !waitingForServoHome) {
        waitingForServoHome = true;
        servoHomeWaitStartTime = millis();
        Serial.println("Waiting for rotation servo to return home before starting cut...");
        return true; // Need to wait
    }
    
    if (waitingForServoHome) {
        if (millis() - servoHomeWaitStartTime < ROTATION_SERVO_HOME_WAIT_DURATION_MS) {
            return true; // Still waiting
        }
        Serial.print("Servo home wait complete (");
        Serial.print(millis() - servoHomeWaitStartTime);
        Serial.println("ms) - proceeding with cut");
        waitingForServoHome = false;
    }
    
    return false; // No wait needed
}

// Activates a component when cut motor reaches specified position
void activateComponentAtPosition(bool& activatedFlag, float activationOffsetInches, 
                                 const char* componentName, void (*activationFunction)()) {
    if (activatedFlag) return;
    
    FastAccelStepper* cutMotor = getCutMotor();
    if (!cutMotor) return;
    
    extern float getCutTravelDistance();
    long activationSteps = (getCutTravelDistance() - activationOffsetInches) * CUT_MOTOR_STEPS_PER_INCH;
    
    if (cutMotor->getCurrentPosition() >= activationSteps) {
        activationFunction();
        activatedFlag = true;
        Serial.print(componentName);
        Serial.print(" activated at ");
        Serial.print((float)activationSteps / CUT_MOTOR_STEPS_PER_INCH, 2);
        Serial.println(" inches");
    }
}

// Outputs debug information about cut motor position
void outputCutMotorDebug() {
    if (millis() - cuttingLastDebugTime >= 1000) {
        FastAccelStepper* cutMotor = getCutMotor();
        if (cutMotor) {
            long currentPosition = cutMotor->getCurrentPosition();
            float currentPositionInches = (float)currentPosition / CUT_MOTOR_STEPS_PER_INCH;
            Serial.print("Cut position: ");
            Serial.print(currentPositionInches, 2);
            Serial.print("/");
            extern float getCutTravelDistance();
            Serial.print(getCutTravelDistance());
            Serial.print(" inches, Running: ");
            Serial.println(cutMotor->isRunning() ? "YES" : "NO");
        }
        cuttingLastDebugTime = millis();
    }
}

void onEnterCuttingState() {
    resetCuttingSteps();
    startCuttingCycleTimer();
    stopReloadTimer(); // Stop reload time tracking when entering cutting state
}

void onExitCuttingState() {
    resetCuttingSteps();
}

void executeCuttingState() {
    if (homePositionErrorDetected) {
        handleHomePositionError();
        return;
    }

    switch (cuttingStep) {
        case 0: 
            handleCuttingStep0();
            break;
        case 1: 
            handleCuttingStep1();
            break;
        case 2: 
            handleCuttingStep2();
            break;
    }
}

void handleCuttingStep0() {
    //! Wait for rotation servo to return home if needed
    if (waitForServoHomeIfNeeded()) {
        return; // Still waiting, exit and check again next cycle
    }
    
    Serial.println("Starting cut motion");
        
    //! Extend clamps to secure wood
    extend2x4SecureClamp();
    extendFeedClamp();

    //! Home rotation servo if wood is properly grabbed (always ensure it's at home position)
    if (isWoodProperlyGrabbed()) {
        extern bool rotationServoIsActiveAndTiming;
        handleRotationServoReturn();
        //! Verification delay: Allow servo to start rotating before cut motor moves
        delay(100); // 100ms delay to ensure servo has started rotating back to home
        if (rotationServoIsActiveAndTiming) {
            Serial.println("Rotation servo homed for cut cycle - wood properly grabbed by transfer arm");
        } else {
            Serial.println("Rotation servo homed for first cut cycle - wood properly grabbed by transfer arm");
        }
    }

    //! Configure cut motor speed based on wood detection
    if (!isWoodPresent()) {
        configureCutMotorForCuttingSlow();
        Serial.println("No wood detected - cutting at 60% speed");
    } else {
        configureCutMotorForCutting();
    }
    moveCutMotorToCut();
    
    rotationClampActivatedThisCycle = false;
    cuttingStep = 1;
}

void handleCuttingStep1() {
    if (stepStartTime == 0) {
        stepStartTime = millis();
    }

    //! Update LED based on wood present sensor
    updateWoodPresentLed();

    //! Check suction sensor when cut motor reaches check distance
    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && cutMotor->getCurrentPosition() >= SUCTION_SENSOR_CHECK_DISTANCE_STEPS) {
        Bounce* suctionSensor = getSuctionSensorBounce();
        if (suctionSensor && suctionSensor->read() == LOW) {
            //! No suction detected - error condition
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
            stepStartTime = 0;
            return;
        }
        
        //! Suction OK - continue to step 2
        cuttingStep = 2;
        stepStartTime = 0;
    }
}

void handleCuttingStep2() {
    //! Update LED based on wood present sensor
    updateWoodPresentLed();
    
    //! Output debug information
    outputCutMotorDebug();
    
    //! Activate components at their respective positions
    activateComponentAtPosition(rotationClampActivatedThisCycle, 
                                ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES,
                                "Rotation clamp", 
                                extendRotationClamp);
    
    activateComponentAtPosition(rotationServoActivatedThisCycle, 
                                ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES,
                                "Rotation servo", 
                                activateRotationServo);
    
    activateComponentAtPosition(transferArmSignalSentThisCycle, 
                                TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES,
                                "TA signal", 
                                sendSignalToTA);
    
    //! Check if cut is complete
    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cut cycle complete - transitioning to return sequence");
        
        //! Configure motor for return and start movement
        configureCutMotorForReturn();
        moveCutMotorToHome();
        transferArmSignalSentThisCycle = false;

        //! Determine next state based on wood detection
        bool no2x4Detected = !isWoodPresent();
        
        //! Update LED before state transition for visual feedback
        if (no2x4Detected) {
            showBlueLed();
            turnYellowLedOff();
        } else {
            showYellowLed();
            turnBlueLedOff();
        }
        
        if (no2x4Detected) {
            changeState(RETURNING_NO_2x4);
        } else {
            changeState(RETURNING_YES_2x4);
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
        onErrorOccurred("Home position error - acknowledged");
        changeState(ERROR_RESET);
        setErrorAcknowledged(true);
    }
}

void resetCuttingSteps() {
    cuttingStep = 0;
    stepStartTime = 0;
    homePositionErrorDetected = false;
    rotationClampActivatedThisCycle = false;
    rotationServoActivatedThisCycle = false;
    transferArmSignalSentThisCycle = false;
    servoHomeWaitStartTime = 0;
    waitingForServoHome = false;
}
