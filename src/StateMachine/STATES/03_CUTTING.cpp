#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "Config/Pins_Definitions.h"

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

// Global variables for multi-segment cutting
bool segmentTransitionPending = false;

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
    // External sensor declarations
    extern const int WOOD_SUCTION_CONFIRM_SENSOR; // From main.cpp
    extern Bounce woodSuctionConfirmSensor;
    
    // Check if wood is still being held by suction
    woodSuctionConfirmSensor.update();
    if (woodSuctionConfirmSensor.read() == LOW) {
        // Wood is still being held, proceed with cutting
        
        // Check if we're using reverse acceleration curve
        if (useReverseAccelerationCurve) {
            // Initialize multi-segment cutting if starting
            if (currentSegment == 0 && !segmentTransitionPending) {
                moveCutMotorToCutWithReverseAcceleration();
                segmentTransitionPending = true;
            }
            
            // Handle segment transitions
            if (segmentTransitionPending && cutMotor && !cutMotor->isRunning()) {
                if (currentSegment == 0) {
                    // First segment complete (0-2.0"), start middle segment (2.0"-6.1")
                    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_SLOW_SPEED);
                    cutMotor->setAcceleration((uint32_t)CUT_MOTOR_CUTTING_ACCELERATION); // Keep low acceleration
                    cutMotor->moveTo((CUT_TRAVEL_DISTANCE - CUT_MOTOR_TRANSITION_START_OFFSET) * CUT_MOTOR_STEPS_PER_INCH);
                    currentSegment = 1;
                    segmentTransitionPending = false;
                    
                } else if (currentSegment == 1) {
                    // Middle segment complete (2.0"-6.1"), start final segment (6.1"-9.1")
                    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_FAST_SPEED);
                    cutMotor->setAcceleration((uint32_t)CUT_MOTOR_CUTTING_ACCELERATION); // Keep low acceleration
                    cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
                    currentSegment = 2;
                    segmentTransitionPending = false;
                    
                } else if (currentSegment == 2) {
                    // All segments complete, proceed to next step
                    cuttingStep = 2;
                    
                    // Reset segment tracking for next cycle
                    currentSegment = 0;
                    segmentTransitionPending = false;
                }
            }
        } else {
            // Use normal cutting approach
            if (cutMotor && !cutMotor->isRunning()) {
                configureCutMotorForCutting();
                moveCutMotorToCut();
                cuttingStep = 2;
            }
        }
    } else {
        // Wood is not being held by suction - transition to suction error state
        currentState = SUCTION_ERROR;
        cuttingStep = 1; // Reset for next time
        
        // Reset segment tracking
        currentSegment = 0;
        segmentTransitionPending = false;
    }
}

void handleCuttingStep2() {
    // Wait for cut motor to complete its travel
    if (cutMotor && !cutMotor->isRunning()) {
        // Cut motor has completed travel, configure for return and move to next step
        configureCutMotorForReturn();
        moveCutMotorToHome();
        cuttingStep = 3;
    }
}

void handleCuttingStep3() {
    // Wait for cut motor to return to home position
    if (cutMotor && !cutMotor->isRunning()) {
        // Cut motor has returned to home, check if we're dealing with 2x4s
        if (is2x4Mode) {
            // 2x4 mode - transition to RETURNING_Yes_2x4 state
            currentState = RETURNING_Yes_2x4;
        } else {
            // Non-2x4 mode - transition to RETURNING_No_2x4 state
            currentState = RETURNING_No_2x4;
        }
        cuttingStep = 1; // Reset for next time
        
        // Reset segment tracking
        currentSegment = 0;
        segmentTransitionPending = false;
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