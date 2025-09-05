#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
// This state manages a multi-step cutting process.
// It includes logic for normal cutting, deciding between a YES_WOOD Sequence and a NO_WOOD Sequence, and error handling.

//! ************************************************************************
//! STEP 0: START CUT MOTION - CONFIGURE AND MOVE CUT MOTOR
//! ************************************************************************

//! ************************************************************************
//! STEP 1: CHECK WOOD SUCTION SENSOR AND START CUT MOTOR
//! ************************************************************************

//! ************************************************************************
//! STEP 2: MONITOR CUT MOTOR POSITION AND ACTIVATE ROTATION CLAMP/SERVO
//! ************************************************************************

//! ************************************************************************
//! STEP 3: WAIT FOR CUT MOTOR TO COMPLETE TRAVEL
//! ************************************************************************
// NOTE: Steps 3-9 below contain functionality that should be moved to RETURNING states
// The CUTTING state should only handle the actual cutting operation (steps 0-2)

//! ************************************************************************
//! STEP 4: DETERMINE WOOD PRESENCE AND TRANSITION TO APPROPRIATE STATE
//! ************************************************************************

//! ************************************************************************
//! STEP 5: HANDLE CONTINUOUS MODE LOGIC
//! ************************************************************************

//! ************************************************************************
//! STEP 8: FEED MOTOR HOMING SEQUENCE (ERROR RECOVERY)
//! ************************************************************************

//! ************************************************************************
//! STEP 9: SUCTION ERROR RECOVERY SEQUENCE
//! ************************************************************************

// Static variables for cutting state tracking
static int cuttingStep = 0;
static unsigned long stepStartTime = 0;
static unsigned long signalStartTime = 0;
static bool signalActive = false;
static bool homePositionErrorDetected = false;
static bool rotationClampActivatedThisCycle = false;
static bool rotationServoActivatedThisCycle = false;
static bool taSignalSentThisCycle = false; // Track TA signal per cycle

void onEnterCuttingState() {
    // Reset all step counters when entering cutting state
    resetCuttingSteps();
}

void onExitCuttingState() {
    // Reset all step counters when exiting cutting state
    resetCuttingSteps();
}

void executeCuttingState() {
    // Throttle state logging to every 2 seconds
    static unsigned long lastStateLogTime = 0;
    if (millis() - lastStateLogTime >= 2000) {
        //serial.println("Current State: CUTTING");
        lastStateLogTime = millis();
    }
    
    if (homePositionErrorDetected) {
        handleHomePositionError();
        return;
    }
    
    // Handle signal timing independently of motor movements
    if (signalActive && millis() - signalStartTime >= 2000) { // Original was 2000ms
        signalActive = false;
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
        // Steps 3-9 removed - functionality moved to RETURNING states
    }
}

void handleCuttingStep0() {
    Serial.println("Starting cut motion");
        
    extend2x4SecureClamp();
    extendFeedClamp();

    // CRITICAL SAFETY CHECK: Only home the rotation servo if wood is properly grabbed by transfer arm
    // LOW = NO SUCTION detected (wood not grabbed) - DO NOT MOVE SERVO (wood could be stuck)
    // HIGH = Wood is properly grabbed by transfer arm suction - SAFE TO MOVE SERVO
    Bounce* suctionSensor = getSuctionSensorBounce();
    if (suctionSensor && suctionSensor->read() == HIGH) {
        // Wood properly grabbed by transfer arm - safe to home the rotation servo
        handleRotationServoReturn();
        Serial.println("Rotation servo homed for cut cycle - wood properly grabbed by transfer arm");
    } else {
        // Wood not properly grabbed - DO NOT move servo for safety (could ram into stuck wood)
        Serial.println("WARNING: Wood not properly grabbed by transfer arm - rotation servo NOT homed for safety");
    }

    // Configure and move motor
    configureCutMotorForCutting();
    moveCutMotorToCut();
    
    rotationClampActivatedThisCycle = false; // Reset for this cut cycle
    cuttingStep = 1;
}

void handleCuttingStep1() {
    // CUTTING (Step 1): Check WAS_WOOD_SUCTIONED_SENSOR, start cut motor, monitor position for servo activation
    extern const int WOOD_SUCTION_CONFIRM_SENSOR; // From main.cpp
    
    if (stepStartTime == 0) {
        stepStartTime = millis();
    }

    // Check suction sensor after cut motor has traveled the required distance
    FastAccelStepper* cutMotor = getCutMotor();
    if (cutMotor && cutMotor->getCurrentPosition() >= (SUCTION_SENSOR_CHECK_DISTANCE_INCHES * CUT_MOTOR_STEPS_PER_INCH)) {
        extern const int WOOD_SUCTION_CONFIRM_SENSOR; // From main.cpp
        
        // Check suction sensor - LOW means NO SUCTION detected (Error condition)
        // HIGH would mean wood is properly grabbed by transfer arm suction
        // Using debounced reading with 15ms debounce time
        Bounce* suctionSensor = getSuctionSensorBounce();
        if (suctionSensor && suctionSensor->read() == LOW) {
            //serial.println("Cutting Step 1: No suction detected after cut motor traveled required distance. Error detected. Returning cut motor home before manual reset.");
            
            // Stop feed motor with controlled deceleration but return cut motor home safely
            FastAccelStepper* cutMotor = getCutMotor();
            FastAccelStepper* feedMotor = getFeedMotor();
            
            if (feedMotor && feedMotor->isRunning()) {
                feedMotor->stopMove(); // Use controlled deceleration instead of force stop
                //serial.println("Feed motor stopping with deceleration due to suction error.");
            }
            
            // Configure cut motor for safe return home
            if (cutMotor) {
                configureCutMotorForReturn();
                moveCutMotorToHome();
                //serial.println("Cut motor returning home due to suction error.");
            }
            
            // Set cutting cycle flag to false to prevent continuous operation
            setCuttingCycleInProgress(false);
            
            // Transition to suction error recovery step
            cuttingStep = 9; // New step for suction error recovery
            stepStartTime = 0; // Reset step timer
            return;
        } else {
            // Suction OK - wood is properly grabbed by transfer arm (HIGH signal)
            configureCutMotorForCutting();
            moveCutMotorToCut();
            
            cuttingStep = 2;
            stepStartTime = 0; // Reset for next step
        }
    }
}

void handleCuttingStep2() {
    FastAccelStepper* cutMotor = getCutMotor();
    extern const int _2x4_PRESENT_SENSOR; // From main.cpp
    extern const float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    extern const float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    extern const float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    // CUT_TRAVEL_DISTANCE and CUT_MOTOR_STEPS_PER_INCH are already declared in General_Functions.h
    
    // Reduced debug logging to every 1000ms to minimize stuttering
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
    
    // Early Rotation Clamp Activation (matching old catcher clamp logic)
    if (!rotationClampActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        extendRotationClamp();
        rotationClampActivatedThisCycle = true;
        Serial.print("Rotation clamp activated at ");
        Serial.print(CUT_TRAVEL_DISTANCE - ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES);
        Serial.println(" inches");
    }
    
    // Early Rotation Servo Activation (matching old catcher servo logic)
    if (!rotationServoActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        activateRotationServo();
        rotationServoActivatedThisCycle = true;
        Serial.print("Rotation servo activated at ");
        Serial.print(CUT_TRAVEL_DISTANCE - ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES);
        Serial.println(" inches");
    }
    
    // Early TA Signal Activation - Send signal before cut completes
    if (!taSignalSentThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        sendSignalToTA();
        taSignalSentThisCycle = true;
        Serial.print("TA signal sent at ");
        Serial.print(CUT_TRAVEL_DISTANCE - TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES);
        Serial.println(" inches (early activation)");
    }
    
    // Check if motor finished moving to cut position
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cut cycle complete - transitioning to return sequence");
        configureCutMotorForReturn();
        
        // Reset TA signal flag for next cycle
        taSignalSentThisCycle = false;

        int sensorValue = digitalRead(_2x4_PRESENT_SENSOR);
        bool no2x4Detected = (sensorValue == HIGH);
        
        if (no2x4Detected) {
            changeState(RETURNING_NO_2x4);
        } else {
            changeState(RETURNING_YES_2x4);
        }
    }
}

void handleCuttingStep3() {
    // This step is no longer used - functionality moved to RETURNING states
    // CUTTING state now only handles steps 0-2 (actual cutting operation)
}

void handleCuttingStep4() {
    // This step is no longer used - functionality moved to RETURNING states
    // CUTTING state now only handles steps 0-2 (actual cutting operation)
}

void handleCuttingStep5() {
    // This step is no longer used - functionality moved to RETURNING states
    // CUTTING state now only handles steps 0-2 (actual cutting operation)
}

void handleCuttingStep8_FeedMotorHomingSequence() {
    // This step is no longer used - functionality moved to RETURNING states
    // CUTTING state now only handles steps 0-2 (actual cutting operation)
}

void handleCuttingStep9_SuctionErrorRecovery() {
    // This step is no longer used - functionality moved to RETURNING states
    // CUTTING state now only handles steps 0-2 (actual cutting operation)
}

void handleHomePositionError() {
    //serial.println("Home position error detected during cutting operation."); 
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
        //serial.println("Home position error acknowledged by reload switch."); 
    }
}

void resetCuttingSteps() {
    cuttingStep = 0;
    signalStartTime = 0;
    signalActive = false;
    homePositionErrorDetected = false;
    rotationClampActivatedThisCycle = false;
    rotationServoActivatedThisCycle = false;
    taSignalSentThisCycle = false; // Reset TA signal flag
} 