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

// Static variables for reverse acceleration curve
static bool reverseAccelCurveActive = false;
static bool firstSegmentComplete = false;
static bool middleSegmentComplete = false;
static bool finalSegmentStarted = false;

// Static variables for multi-segment approach
static int currentSegment = 0; // 0=first fast, 1=middle slow, 2=final fast
static bool segmentTransitionPending = false;

// Static variables for gradual speed transitions
static float currentCutMotorSpeed = 0;
static bool transitioningToSlow = false;
static bool transitioningToFast = false;
static unsigned long lastSpeedUpdateTime = 0;

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
    Serial.println("Starting cut motion with reverse acceleration curve");
        
    extend2x4SecureClamp();
    extendFeedClamp();

    // Configure and move motor with reverse acceleration curve
    moveCutMotorToCutWithReverseAcceleration();
    
    // Initialize reverse acceleration curve tracking
    reverseAccelCurveActive = true;
    firstSegmentComplete = false;
    middleSegmentComplete = false;
    finalSegmentStarted = false;
    
    // Initialize multi-segment tracking
    currentSegment = 0;
    segmentTransitionPending = false;
    
    // Initialize gradual speed transition tracking (legacy)
    currentCutMotorSpeed = CUT_MOTOR_FAST_SPEED;
    transitioningToSlow = false;
    transitioningToFast = false;
    lastSpeedUpdateTime = millis();
    
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
        // Check if rotation servo has started rotating (been activated at least once this cycle)
        // This checks if servo started rotating, not necessarily that it's fully back to 24 degrees
        if (rotationServoActivatedThisCycle) { // Check if servo was activated during this cutting cycle
            extern const int WOOD_SUCTION_CONFIRM_SENSOR; // From main.cpp
            
            // Check suction sensor - LOW means NO SUCTION (Error condition)
            if (digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == LOW) {
                //serial.println("Cutting Step 1: Rotation servo started rotating but no suction detected. Error detected. Returning cut motor home before manual reset.");
                
                // Stop feed motor immediately but return cut motor home safely
                FastAccelStepper* cutMotor = getCutMotor();
                FastAccelStepper* feedMotor = getFeedMotor();
                
                if (feedMotor && feedMotor->isRunning()) {
                    feedMotor->forceStop();
                    //serial.println("Feed motor stopped due to suction error.");
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
                // Suction OK - proceed with cutting
                // Don't call moveCutMotorToCutWithReverseAcceleration() again - already started in step 0
                
                // Initialize reverse acceleration curve tracking if not already active
                if (!reverseAccelCurveActive) {
                    reverseAccelCurveActive = true;
                    firstSegmentComplete = false;
                    middleSegmentComplete = false;
                    finalSegmentStarted = false;
                    currentCutMotorSpeed = CUT_MOTOR_FAST_SPEED;
                    transitioningToSlow = false;
                    transitioningToFast = false;
                    lastSpeedUpdateTime = millis();
                }
                
                cuttingStep = 2;
                stepStartTime = 0; // Reset for next step
            }
        } else {
            // Servo hasn't been activated yet - continue waiting or proceed normally
            // This handles the case where the cut motor reaches 1 inch before servo activation
            // Don't call moveCutMotorToCutWithReverseAcceleration() again - already started in step 0
            
            // Initialize reverse acceleration curve tracking if not already active
            if (!reverseAccelCurveActive) {
                reverseAccelCurveActive = true;
                firstSegmentComplete = false;
                middleSegmentComplete = false;
                finalSegmentStarted = false;
                currentCutMotorSpeed = CUT_MOTOR_FAST_SPEED;
                transitioningToSlow = false;
                transitioningToFast = false;
                lastSpeedUpdateTime = millis();
            }
            
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
    
    // Multi-Segment Reverse Acceleration Curve Management
    if (reverseAccelCurveActive && cutMotor) {
        float currentPositionInches = (float)cutMotor->getCurrentPosition() / CUT_MOTOR_STEPS_PER_INCH;
        
        // Check if current segment is complete and start next segment
        if (!cutMotor->isRunning() && !segmentTransitionPending) {
            segmentTransitionPending = true;
            
            if (currentSegment == 0) {
                // First segment complete (0-2.0"), start middle segment (2.0"-6.1")
                Serial.println("// First segment complete, starting middle segment at slow speed (100 steps/sec)");
                cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_SLOW_SPEED);
                cutMotor->setAcceleration((uint32_t)CUT_MOTOR_CUTTING_ACCELERATION); // Keep low acceleration
                cutMotor->moveTo((CUT_TRAVEL_DISTANCE - CUT_MOTOR_TRANSITION_START_OFFSET) * CUT_MOTOR_STEPS_PER_INCH);
                currentSegment = 1;
                segmentTransitionPending = false;
                
            } else if (currentSegment == 1) {
                // Middle segment complete (2.0"-6.1"), start final segment (6.1"-9.1")
                Serial.println("// Middle segment complete, starting final segment at fast speed (2000 steps/sec)");
                cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_FAST_SPEED);
                cutMotor->setAcceleration((uint32_t)CUT_MOTOR_CUTTING_ACCELERATION); // Keep low acceleration
                cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
                currentSegment = 2;
                segmentTransitionPending = false;
                
            } else if (currentSegment == 2) {
                // All segments complete
                Serial.println("// All reverse acceleration curve segments complete");
                reverseAccelCurveActive = false;
                currentSegment = 0;
                segmentTransitionPending = false;
            }
        }
        
        // Debug output every 1000ms
        static unsigned long lastSegmentDebugTime = 0;
        if (millis() - lastSegmentDebugTime >= 1000) {
            Serial.print("// Segment: ");
            Serial.print(currentSegment);
            Serial.print(", Position: ");
            Serial.print(currentPositionInches, 2);
            Serial.print("\", Running: ");
            Serial.println(cutMotor->isRunning() ? "YES" : "NO");
            lastSegmentDebugTime = millis();
        }
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
    if (cutMotor && !cutMotor->isRunning() && !reverseAccelCurveActive) {
        Serial.println("Cut cycle complete - transitioning to return sequence");
        configureCutMotorForReturn();
        
        // Reset all tracking variables
        currentSegment = 0;
        segmentTransitionPending = false;
        firstSegmentComplete = false;
        middleSegmentComplete = false;
        finalSegmentStarted = false;
        
        // Reset gradual speed transition variables
        currentCutMotorSpeed = 0;
        transitioningToSlow = false;
        transitioningToFast = false;
        lastSpeedUpdateTime = 0;
        
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
    
    // Reset reverse acceleration curve variables
    reverseAccelCurveActive = false;
    firstSegmentComplete = false;
    middleSegmentComplete = false;
    finalSegmentStarted = false;
    
    // Reset multi-segment variables
    currentSegment = 0;
    segmentTransitionPending = false;
    
    // Reset gradual speed transition variables
    currentCutMotorSpeed = 0;
    transitioningToSlow = false;
    transitioningToFast = false;
    lastSpeedUpdateTime = 0;
}