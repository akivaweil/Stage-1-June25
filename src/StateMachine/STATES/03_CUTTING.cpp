#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Completely refactored cutting state with clean, organized structure
// Handles wood cutting with acceleration curve, safety checks, and component activation

//* ************************************************************************
//* ************************ STATE VARIABLES *******************************
//* ************************************************************************
static int cuttingStep = 0;
static unsigned long stepStartTime = 0;
static bool homePositionErrorDetected = false;
static bool rotationClampActivated = false;
static bool rotationServoActivated = false;
static bool transferArmSignalSent = false;
static unsigned long lastWoodSensorCheck = 0;
static unsigned long lastPositionDebug = 0;

//* ************************************************************************
//* ************************ STATE MANAGEMENT ******************************
//* ************************************************************************

void onEnterCuttingState() {
    resetCuttingState();
    startCuttingCycleTimer();
    stopReloadTimer();
    Serial.println("=== ENTERING CUTTING STATE ===");
}

void onExitCuttingState() {
    resetCuttingState();
    Serial.println("=== EXITING CUTTING STATE ===");
}

void executeCuttingState() {
    if (homePositionErrorDetected) {
        handleHomePositionError();
        return;
    }

    switch (cuttingStep) {
        case 0: handleCuttingInitialization(); break;
        case 1: handleCuttingExecution(); break;
        case 2: handleCuttingCompletion(); break;
        default: 
            Serial.println("ERROR: Invalid cutting step!");
            changeState(ERROR_RESET);
            break;
    }
}

//* ************************************************************************
//* ************************ CUTTING STEPS *********************************
//* ************************************************************************

void handleCuttingInitialization() {
    Serial.println("CUTTING: Step 0 - Initialization");
    
    // Extend clamps for secure cutting
    extend2x4SecureClamp();
    extendFeedClamp();
    
    // Safety check: Only home rotation servo if wood is properly grabbed
    Bounce* suctionSensor = getSuctionSensorBounce();
    if (suctionSensor && suctionSensor->read() == HIGH) {
        handleRotationServoReturn();
        Serial.println("CUTTING: Rotation servo homed - wood properly secured");
    } else {
        Serial.println("CUTTING: WARNING - Wood not properly secured, servo NOT homed");
    }
    
    // Start cut motor with acceleration curve
    moveCutMotorToCut();
    
    // Reset activation flags
    rotationClampActivated = false;
    rotationServoActivated = false;
    transferArmSignalSent = false;
    
    cuttingStep = 1;
    stepStartTime = millis();
}

void handleCuttingExecution() {
    if (stepStartTime == 0) {
        stepStartTime = millis();
    }
    
    // Continuous monitoring during cutting
    monitorWoodSensor();
    handleAccelerationCurve();
    checkSuctionSensor();
    activateComponentsAtPositions();
    
    // Check if cut is complete
    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && !cutMotor->isRunning()) {
        cuttingStep = 2;
        stepStartTime = 0;
    }
}

void handleCuttingCompletion() {
    Serial.println("CUTTING: Step 2 - Completion");
    
    // Configure motor for return
    configureCutMotorForReturn();
    
    // Determine next state based on wood detection
    extern const int _2x4_PRESENT_SENSOR;
    int sensorValue = digitalRead(_2x4_PRESENT_SENSOR);
    bool no2x4Detected = (sensorValue == HIGH);
    
    if (no2x4Detected) {
        Serial.println("CUTTING: No 2x4 detected - transitioning to RETURNING_NO_2x4");
        changeState(RETURNING_NO_2x4);
    } else {
        Serial.println("CUTTING: 2x4 detected - transitioning to RETURNING_YES_2x4");
        changeState(RETURNING_YES_2x4);
    }
}

//* ************************************************************************
//* ************************ MONITORING FUNCTIONS **************************
//* ************************************************************************

void monitorWoodSensor() {
    extern const int _2x4_PRESENT_SENSOR;
    
    // Check every 100ms for stable readings
    if (millis() - lastWoodSensorCheck >= 100) {
        // Triple reading for stability
        int readings[3];
        readings[0] = digitalRead(_2x4_PRESENT_SENSOR);
        delay(2);
        readings[1] = digitalRead(_2x4_PRESENT_SENSOR);
        delay(2);
        readings[2] = digitalRead(_2x4_PRESENT_SENSOR);
        
        // Majority vote
        int lowCount = 0;
        for (int i = 0; i < 3; i++) {
            if (readings[i] == LOW) lowCount++;
        }
        
        bool woodPresent = (lowCount >= 2);
        
        // Update LED status
        if (woodPresent) {
            turnYellowLedOn(); // Wood present
        } else {
            turnBlueLedOn(); // No wood
        }
        
        lastWoodSensorCheck = millis();
    }
}

void handleAccelerationCurve() {
    // Handle the reverse acceleration curve (fast-slow-fast)
    handleCutMotorReverseAccelerationCurve();
}

void checkSuctionSensor() {
    FastAccelStepper* cutMotor = getCutMotor();
    if (!cutMotor) return;
    
    // Check suction sensor after initial movement
    if (cutMotor->getCurrentPosition() >= SUCTION_SENSOR_CHECK_DISTANCE_STEPS) {
        Bounce* suctionSensor = getSuctionSensorBounce();
        if (suctionSensor && suctionSensor->read() == LOW) {
            // No suction detected - emergency stop
            Serial.println("CUTTING: ERROR - Wood suction not confirmed!");
            
            // Stop all motors
            FastAccelStepper* feedMotor = getFeedMotor();
            if (feedMotor && feedMotor->isRunning()) {
                feedMotor->stopMove();
            }
            
            if (cutMotor) {
                configureCutMotorForReturn();
                moveCutMotorToHome();
            }
            
            // Transition to error state
            setCuttingCycleInProgress(false);
            onErrorOccurred("Wood suction not confirmed");
            changeState(SUCTION_ERROR);
        }
    }
}

void activateComponentsAtPositions() {
    FastAccelStepper* cutMotor = getCutMotor();
    if (!cutMotor) return;
    
    long currentPosition = cutMotor->getCurrentPosition();
    
    // Activate rotation clamp at specified position
    if (!rotationClampActivated && currentPosition >= ROTATION_CLAMP_ACTIVATION_POSITION_STEPS) {
        extendRotationClamp();
        rotationClampActivated = true;
        float positionInches = (float)ROTATION_CLAMP_ACTIVATION_POSITION_STEPS / CUT_MOTOR_STEPS_PER_INCH;
        Serial.printf("CUTTING: Rotation clamp activated at %.2f inches\n", positionInches);
    }
    
    // Activate rotation servo at specified position
    if (!rotationServoActivated && currentPosition >= ROTATION_SERVO_ACTIVATION_POSITION_STEPS) {
        activateRotationServo();
        rotationServoActivated = true;
        float positionInches = (float)ROTATION_SERVO_ACTIVATION_POSITION_STEPS / CUT_MOTOR_STEPS_PER_INCH;
        Serial.printf("CUTTING: Rotation servo activated at %.2f inches\n", positionInches);
    }
    
    // Send transfer arm signal at specified position
    if (!transferArmSignalSent && currentPosition >= TA_SIGNAL_ACTIVATION_POSITION_STEPS) {
        sendSignalToTA();
        transferArmSignalSent = true;
        float positionInches = (float)TA_SIGNAL_ACTIVATION_POSITION_STEPS / CUT_MOTOR_STEPS_PER_INCH;
        Serial.printf("CUTTING: TA signal sent at %.2f inches\n", positionInches);
    }
    
    // Debug position every 2 seconds
    if (millis() - lastPositionDebug >= 2000) {
        float currentPositionInches = (float)currentPosition / CUT_MOTOR_STEPS_PER_INCH;
        Serial.printf("CUTTING: Position %.2f/%.2f inches, Running: %s\n", 
                     currentPositionInches, CUT_TRAVEL_DISTANCE, 
                     cutMotor->isRunning() ? "YES" : "NO");
        lastPositionDebug = millis();
    }
}

//* ************************************************************************
//* ************************ ERROR HANDLING *******************************
//* ************************************************************************

void handleHomePositionError() {
    // Blink red and yellow LEDs to indicate error
    static unsigned long lastBlinkTime = 0;
    static bool blinkState = false;
    
    if (millis() - lastBlinkTime > 100) {
        blinkState = !blinkState;
        if (blinkState) {
            turnRedLedOn();
            turnYellowLedOff();
        } else {
            turnRedLedOff();
            turnYellowLedOn();
        }
        lastBlinkTime = millis();
    }
    
    // Stop all motors immediately
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
    if (feedMotor) feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
    
    // Extend secure clamp for safety
    extend2x4SecureClamp();
    
    // Check for error acknowledgment
    if (getReloadSwitch()->rose()) {
        homePositionErrorDetected = false;
        onErrorOccurred("Home position error - acknowledged");
        changeState(ERROR_RESET);
        setErrorAcknowledged(true);
    }
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetCuttingState() {
    cuttingStep = 0;
    stepStartTime = 0;
    homePositionErrorDetected = false;
    rotationClampActivated = false;
    rotationServoActivated = false;
    transferArmSignalSent = false;
    lastWoodSensorCheck = 0;
    lastPositionDebug = 0;
}

//* ************************************************************************
//* ************************ LEGACY FUNCTIONS *****************************
//* ************************************************************************
// Legacy functions for compatibility with existing code

void handleCuttingStep0() {
    handleCuttingInitialization();
}

void handleCuttingStep1() {
    handleCuttingExecution();
}

void handleCuttingStep2() {
    handleCuttingCompletion();
}

void resetCuttingSteps() {
    resetCuttingState();
}

void checkWoodPresentSensor() {
    monitorWoodSensor();
}