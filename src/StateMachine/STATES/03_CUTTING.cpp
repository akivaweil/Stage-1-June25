#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation with a clean 3-step process:
// Step 0: Initialize cutting sequence - extend clamps and configure motors
// Step 1: Check suction sensor and start cut motor movement
// Step 2: Monitor cut motor position, activate rotation components, and complete cut
// 
// After cutting completion, transitions to appropriate RETURNING state based on wood detection.
// All post-cutting logic (return sequences, homing, continuous mode) is handled by RETURNING states.

// Static variables for cutting state tracking
static int cuttingStep = 0;
static unsigned long stepStartTime = 0;
static bool homePositionErrorDetected = false;
static bool rotationClampActivatedThisCycle = false;
static bool rotationServoActivatedThisCycle = false;
static bool transferArmSignalSentThisCycle = false;
static unsigned long lastWoodSensorCheckTime = 0;

void onEnterCuttingState() {
    resetCuttingSteps();
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
    Serial.println("Starting cut motion");
        
    extend2x4SecureClamp();
    extendFeedClamp();

    // Only home rotation servo if wood is properly grabbed (safety check)
    Bounce* suctionSensor = getSuctionSensorBounce();
    if (suctionSensor && suctionSensor->read() == HIGH) {
        handleRotationServoReturn();
        Serial.println("Rotation servo homed for cut cycle - wood properly grabbed by transfer arm");
    } else {
        Serial.println("WARNING: Wood not properly grabbed by transfer arm - rotation servo NOT homed for safety");
    }

    configureCutMotorForCutting();
    moveCutMotorToCut();
    
    rotationClampActivatedThisCycle = false;
    cuttingStep = 1;
}

void handleCuttingStep1() {
    extern const int WOOD_SUCTION_CONFIRM_SENSOR;
    
    if (stepStartTime == 0) {
        stepStartTime = millis();
    }

    //! ************************************************************************
    //! WOOD SENSOR CHECKING: Monitor wood present sensor every 100ms
    //! ************************************************************************
    checkWoodPresentSensor();

    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && cutMotor->getCurrentPosition() >= SUCTION_SENSOR_CHECK_DISTANCE_STEPS) {
        Bounce* suctionSensor = getSuctionSensorBounce();
        if (suctionSensor && suctionSensor->read() == LOW) {
            // No suction detected - error condition
            FastAccelStepper* feedMotor = getFeedMotor();
            
            if (feedMotor && feedMotor->isRunning()) {
                feedMotor->stopMove();
            }
            
            if (cutMotor) {
                configureCutMotorForReturn();
                moveCutMotorToHome();
            }
            
            setCuttingCycleInProgress(false);
            changeState(SUCTION_ERROR);
            stepStartTime = 0;
            return;
        } else {
            // Suction OK - continue cutting
            cuttingStep = 2;
            stepStartTime = 0;
        }
    }
}

void handleCuttingStep2() {
    FastAccelStepper* cutMotor = getCutMotor();
    extern const int _2x4_PRESENT_SENSOR;
    
    //! ************************************************************************
    //! WOOD SENSOR CHECKING: Monitor wood present sensor every 100ms
    //! ************************************************************************
    checkWoodPresentSensor();
    
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 1000) {
        if (cutMotor) {
            long currentPosition = cutMotor->getCurrentPosition();
            float currentPositionInches = (float)currentPosition / CUT_MOTOR_STEPS_PER_INCH;
            Serial.print("Cut position: ");
            Serial.print(currentPositionInches, 2);
            Serial.print("/");
            Serial.print(CUT_TRAVEL_DISTANCE);
            Serial.print(" inches, Running: ");
            Serial.println(cutMotor->isRunning() ? "YES" : "NO");
        }
        lastDebugTime = millis();
    }
    
    if (!rotationClampActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ROTATION_CLAMP_ACTIVATION_POSITION_STEPS) {
        extendRotationClamp();
        rotationClampActivatedThisCycle = true;
        Serial.print("Rotation clamp activated at ");
        Serial.print((float)ROTATION_CLAMP_ACTIVATION_POSITION_STEPS / CUT_MOTOR_STEPS_PER_INCH, 2);
        Serial.println(" inches");
    }
    
    if (!rotationServoActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ROTATION_SERVO_ACTIVATION_POSITION_STEPS) {
        activateRotationServo();
        rotationServoActivatedThisCycle = true;
        Serial.print("Rotation servo activated at ");
        Serial.print((float)ROTATION_SERVO_ACTIVATION_POSITION_STEPS / CUT_MOTOR_STEPS_PER_INCH, 2);
        Serial.println(" inches");
    }
    
    if (!transferArmSignalSentThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= TA_SIGNAL_ACTIVATION_POSITION_STEPS) {
        sendSignalToTA();
        transferArmSignalSentThisCycle = true;
        Serial.print("TA signal sent at ");
        Serial.print((float)TA_SIGNAL_ACTIVATION_POSITION_STEPS / CUT_MOTOR_STEPS_PER_INCH, 2);
        Serial.println(" inches (early activation)");
    }
    
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cut cycle complete - transitioning to return sequence");
        configureCutMotorForReturn();
        transferArmSignalSentThisCycle = false;

        int sensorValue = digitalRead(_2x4_PRESENT_SENSOR);
        bool no2x4Detected = (sensorValue == HIGH);
        
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
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        setLastErrorBlinkTime(millis());
    }
    
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
    if (feedMotor) feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
    
    extend2x4SecureClamp();
    
    if (getReloadSwitch()->rose()) {
        homePositionErrorDetected = false;
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
    lastWoodSensorCheckTime = 0;
}

//* ************************************************************************
//* *********************** WOOD SENSOR CHECKING ***************************
//* ************************************************************************
// Checks wood present sensor every 100ms during cutting and updates LED accordingly
// Blue LED = wood present (sensor LOW), Yellow LED = no wood (sensor HIGH)

void checkWoodPresentSensor() {
    extern const int _2x4_PRESENT_SENSOR;
    
    // Check every 100ms
    if (millis() - lastWoodSensorCheckTime >= 100) {
        int sensorValue = digitalRead(_2x4_PRESENT_SENSOR);
        bool woodPresent = (sensorValue == LOW); // Active LOW sensor
        
        if (woodPresent) {
            turnBlueLedOn(); // Wood present - blue LED
        } else {
            turnYellowLedOn(); // No wood - yellow LED
        }
        
        lastWoodSensorCheckTime = millis();
    }
} 