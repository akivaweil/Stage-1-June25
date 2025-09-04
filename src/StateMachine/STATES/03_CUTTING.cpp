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
static float cutMotorIncrementalMoveTotalInches = 0.0;
static int cuttingSubStep8 = 0; // For feed motor homing sequence

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
        case 3: 
            handleCuttingStep3();
            break;
        case 4: 
            handleCuttingStep4();
            break;
        case 5: 
            handleCuttingStep5();
            break;
        case 8:
            handleCuttingStep8_FeedMotorHomingSequence();
            break;
        case 9:
            handleCuttingStep9_SuctionErrorRecovery();
            break;
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
    //serial.println("Cutting Step 3: (Should be bypassed for wood path) Initial position move complete.");
    FastAccelStepper* feedMotor = getFeedMotor();
    if (feedMotor && !feedMotor->isRunning()) {
        retract2x4SecureClamp();
        //serial.println("Feed clamp and 2x4 secure clamp retracted.");

        configureFeedMotorForReturn();
        moveFeedMotorToHome();
        //serial.println("Feed motor moving to home (0).");
    }
}

void handleCuttingStep4() {
    //serial.println("Cutting Step 4: (Logic moved to Step 7 for wood path) Feed motor at home (0).");
    FastAccelStepper* feedMotor = getFeedMotor();
    FastAccelStepper* cutMotor = getCutMotor();
    extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES; // From main.cpp
    extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES; // From main.cpp
    // CUT_MOTOR_STEPS_PER_INCH, FEED_TRAVEL_DISTANCE are already declared in General_Functions.h
    
    if (feedMotor && !feedMotor->isRunning()) {
        retract2x4SecureClamp();
        //serial.println("Feed clamp retracted.");

        if (cutMotor && !cutMotor->isRunning()) {
            //serial.println("Cut motor also at home. Checking cut motor position switch.");
            bool sensorDetectedHome = false;
            for (int i = 0; i < 3; i++) {
                delay(30);
                getCutHomingSwitch()->update();
                Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); //serial.println(getCutHomingSwitch()->read());
                if (getCutHomingSwitch()->read() == HIGH) {
                    sensorDetectedHome = true;
                    if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                    //serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                    break;
                }
            }
            if (!sensorDetectedHome) {
                //serial.println("ERROR: Cut motor position switch did not detect home after return attempt.");
                if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
                    Serial.print("Attempting incremental move. Total moved: ");
                    Serial.print(cutMotorIncrementalMoveTotalInches);
                    //serial.println(" inches.");
                    cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                    cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                    // Stay in cuttingStep 4 to re-check sensor after move
                } else {
                    //serial.println("ERROR: Cut motor position switch did not detect home after MAX incremental moves!");
                    stopCutMotor();
                    stopFeedMotor();
                    extend2x4SecureClamp();
                    turnRedLedOn();
                    turnYellowLedOff();
                    changeState(ERROR);
                    setErrorStartTime(millis());
                    resetCuttingSteps();
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset for next attempt
                    //serial.println("Transitioning to ERROR state due to cut motor homing failure after cut.");
                }
            } else {
                //serial.println("Cut motor position switch confirmed home. Moving feed motor to final position.");
                cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                cuttingStep = 5; 
            }
        }
    }
}

void handleCuttingStep5() {
    FastAccelStepper* feedMotor = getFeedMotor();
    if (feedMotor && !feedMotor->isRunning()) {
        //serial.println("Cutting Step 5: Feed motor at final position. Starting end-of-cycle feed motor homing sequence."); 
        
        //! ************************************************************************
        //! STEP 6: RETRACT FEED CLAMP AND START FEED MOTOR HOMING SEQUENCE
        //! ************************************************************************
        retract2x4SecureClamp();
        //serial.println("Feed clamp retracted. Starting feed motor homing sequence...");
        
        // Transition to new step 8 for feed motor homing sequence
        cuttingStep = 8;  
        cuttingSubStep8 = 0; // Initialize homing substep
        //serial.println("Transitioning to feed motor homing sequence (Step 8)."); 
    }
}

void handleCuttingStep8_FeedMotorHomingSequence() {
    FastAccelStepper* feedMotor = getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    // FEED_TRAVEL_DISTANCE and FEED_MOTOR_STEPS_PER_INCH are already declared in General_Functions.h
    
    // Non-blocking feed motor homing sequence
    switch (cuttingSubStep8) {
        case 0: // Start homing - move toward home sensor
            //serial.println("Feed Motor Homing Step 8.0: Moving toward home sensor.");
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward sensor
            }
            cuttingSubStep8 = 1;
            break;
            
        case 1: // Wait for home sensor to trigger
            getFeedHomingSwitch()->update();
            if (getFeedHomingSwitch()->read() == LOW) {
                //serial.println("Feed Motor Homing Step 8.1: Home sensor triggered. Stopping motor.");
                if (feedMotor) {
                    feedMotor->stopMove();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                //serial.println("Feed motor hit home sensor.");
                cuttingSubStep8 = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.2 inch from sensor
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("Feed Motor Homing Step 8.2: Moving to -0.2 inch from home sensor to establish working zero.");
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                cuttingSubStep8 = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("Feed Motor Homing Step 8.3: Setting new working zero position.");
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                //serial.println("Feed motor homed: 0.2 inch from sensor set as position 0.");
                
                configureFeedMotorForNormalOperation();
                cuttingSubStep8 = 4;
            }
            break;
            
        case 4: // Homing complete - check for continuous mode or finish cycle
            //serial.println("Feed Motor Homing Step 8.4: Homing sequence complete.");
            extend2x4SecureClamp();
            //serial.println("2x4 secure clamp extended."); 
            turnYellowLedOff();
            setCuttingCycleInProgress(false);
            
            // Check if start cycle switch is active for continuous operation
            if (getStartCycleSwitch()->read() == HIGH && getStartSwitchSafe()) {
                //serial.println("Start cycle switch is active - continuing with another cut cycle.");
                // Prepare for next cycle
                extend2x4SecureClamp();
                extendRotationClamp(); // Extend rotation clamp for next cutting cycle
                configureCutMotorForCutting(); // Ensure cut motor is set to proper cutting speed
                turnYellowLedOn();
                setCuttingCycleInProgress(true);
                changeState(CUTTING);
                resetCuttingSteps();
                //serial.println("Transitioning to CUTTING state for continuous operation.");
            } else {
                //serial.println("Cycle complete. Transitioning to IDLE state.");
                changeState(IDLE);
                resetCuttingSteps();
            }
            break;
    }
}

void handleCuttingStep9_SuctionErrorRecovery() {
    // CUTTING (Step 9): Suction Error Recovery - Wait for cut motor to return home, then transition to error hold
    FastAccelStepper* cutMotor = getCutMotor();
    
    if (cutMotor && !cutMotor->isRunning()) {
        //serial.println("Cutting Step 9: Cut motor returned home after suction error. Checking home sensor.");
        
        // Check cut motor home sensor
        bool sensorDetectedHome = false;
        for (int i = 0; i < 3; i++) {
            delay(30);
            getCutHomingSwitch()->update();
            Serial.print("Cut position switch read attempt "); 
            Serial.print(i+1); 
            Serial.print(": "); 
            //serial.println(getCutHomingSwitch()->read());
            
            if (getCutHomingSwitch()->read() == HIGH) {
                sensorDetectedHome = true;
                if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                //serial.println("Cut motor position switch detected HIGH after suction error recovery.");
                break;
            }
        }
        
        if (sensorDetectedHome) {
            //serial.println("Cut motor successfully returned home after suction error. Transitioning to SUCTION_ERROR for manual reset.");
            changeState(SUCTION_ERROR);
            resetCuttingSteps();
        } else {
            //serial.println("WARNING: Cut motor home sensor not detected after suction error recovery. Proceeding to SUCTION_ERROR anyway.");
            changeState(SUCTION_ERROR);
            resetCuttingSteps();
        }
    } else if (cutMotor) {
        // Motor still running - provide status update
        static unsigned long lastStatusTime = 0;
        if (millis() - lastStatusTime >= 1000) {
            //serial.println("Cutting Step 9: Cut motor returning home after suction error...");
            lastStatusTime = millis();
        }
    }
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
    cutMotorIncrementalMoveTotalInches = 0.0;
    cuttingSubStep8 = 0; // Reset position motor homing substep
} 