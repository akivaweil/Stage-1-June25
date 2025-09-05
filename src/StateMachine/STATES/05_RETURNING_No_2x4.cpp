#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

// Timing constants for this state
const unsigned long ATTENTION_SEQUENCE_DELAY_MS = 50; // Delay between feed clamp movements in attention sequence

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.
// Latest working code - committed: 2025-09-02 14:56:20

//! ************************************************************************
//! STEP 1: INITIALIZE SEQUENCE - MOVE CUT MOTOR HOME AND RETRACT 2X4 CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 2: WAIT FOR CUT MOTOR HOME AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 3: WAIT FOR FEED MOTOR HOME AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 4: MOVE FEED MOTOR TO 2.0 INCHES
//! ************************************************************************

//! ************************************************************************
//! STEP 5: WAIT FOR FEED MOTOR AT 2.0 AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 6: ATTENTION GETTING SEQUENCE - INTENSE FEED CLAMP EXTENSION/RETRACTION (9 MOVEMENTS)
//! ************************************************************************

//! ************************************************************************
//! STEP 7: MOVE FEED MOTOR TO HOME
//! ************************************************************************

//! ************************************************************************
//! STEP 8: WAIT FOR FEED MOTOR HOME AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 9: MOVE FEED MOTOR TO FINAL POSITION
//! ************************************************************************

//! ************************************************************************
//! STEP 10: VERIFY CUT HOME POSITION AND COMPLETE SEQUENCE
//! ************************************************************************

// Static variables for returning no 2x4 state tracking
static int returningNo2x4Step = 0;
static unsigned long cylinderActionTime = 0;
static bool waitingForCylinder = false;

// Cut motor homing recovery tracking
static float cutMotorIncrementalMoveTotalInches = 0.0;

// Feed motor homing sequence tracking
static int feedMotorHomingSubStep = 0;

void executeReturningNo2x4State() {
    handleReturningNo2x4Sequence(); 
}

void onEnterReturningNo2x4State() {
    //! ************************************************************************
    //! STEP 1: INITIALIZE RETURNING NO 2X4 SEQUENCE
    //! ************************************************************************
    
    // Reset consecutive yeswood counter when nowood state occurs
    resetConsecutiveYeswoodCount();
    
    // Initialize RETURNING_NO_2x4 sequence from CUTTING_state logic
    configureCutMotorForReturn();
    moveCutMotorToHome();
    configureFeedMotorForNormalOperation();

    turnBlueLedOn();
    turnYellowLedOff();
    
    // Initialize step tracking
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    feedMotorHomingSubStep = 0;
}

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

void handleReturningNo2x4Sequence() {
    // RETURNING_NO_2x4 sequence logic
    FastAccelStepper* feedMotor = getFeedMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (returningNo2x4Step == 0) { // First time entering this specific RETURNING_NO_2x4 logic path
        retract2x4SecureClamp();
        if (feedMotor) {
            if (feedMotor->getCurrentPosition() != 0 || feedMotor->isRunning()) {
                feedMotor->moveTo(0);
            }
        }
        returningNo2x4Step = 1;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        returningNo2x4Step++; 
    }
    
    if (!waitingForCylinder) {
        handleReturningNo2x4Step(returningNo2x4Step);
    }
}

void handleReturningNo2x4Step(int step) {
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then extend feed clamp
            if (cutMotor && !cutMotor->isRunning()) {
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause returningNo2x4Step to increment to 2 after delay
            }
            break;
            
        case 2: // Was original returningNo2x4Step 1: wait for feed motor, then retract feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original returningNo2x4Step 2: move feed motor to 2.0 inches
            configureFeedMotorForNormalOperation(); // Ensure correct config
            moveFeedMotorToPosition(2.0);
            returningNo2x4Step = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original returningNo2x4Step 3: wait for feed motor at 2.0, extend feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                //serial.println("ReturningNo2x4: Feed clamp extended at 2.0 inches");
                returningNo2x4Step = 5; // Move to attention sequence
            }
            break;
            
        case 5: // Attention-getting sequence: retract, extend, retract, extend, retract, extend, retract, extend, retract (9 movements total)
            static int attentionStep = 0;
            static unsigned long attentionStartTime = 0;
            
            if (attentionStep == 0) {
                // First step: retract feed clamp
                retractFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - retracting feed clamp (1/9)");
                attentionStep = 1;
                attentionStartTime = millis();
            } else if (attentionStep == 1 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Second step: extend feed clamp
                extendFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - extending feed clamp (2/9)");
                attentionStep = 2;
                attentionStartTime = millis();
            } else if (attentionStep == 2 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Third step: retract feed clamp again
                retractFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - retracting feed clamp (3/9)");
                attentionStep = 3;
                attentionStartTime = millis();
            } else if (attentionStep == 3 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Fourth step: extend feed clamp again
                extendFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - extending feed clamp (4/9)");
                attentionStep = 4;
                attentionStartTime = millis();
            } else if (attentionStep == 4 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Fifth step: retract feed clamp again
                retractFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - retracting feed clamp (5/9)");
                attentionStep = 5;
                attentionStartTime = millis();
            } else if (attentionStep == 5 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Sixth step: extend feed clamp again
                extendFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - extending feed clamp (6/9)");
                attentionStep = 6;
                attentionStartTime = millis();
            } else if (attentionStep == 6 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Seventh step: retract feed clamp again
                retractFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - retracting feed clamp (7/9)");
                attentionStep = 7;
                attentionStartTime = millis();
            } else if (attentionStep == 7 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Eighth step: extend feed clamp again
                extendFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - extending feed clamp (8/9)");
                attentionStep = 8;
                attentionStartTime = millis();
            } else if (attentionStep == 8 && millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
                // Ninth step: final extension to ensure clamp is extended at end of attention sequence
                extendFeedClamp();
                //serial.println("ReturningNo2x4: Attention sequence - final extension (9/9)");
                attentionStep = 0; // Reset for next time
                returningNo2x4Step = 6; // Move to next step
            }
            break;
            
        case 6: // Was original returningNo2x4Step 4: move feed motor to home
            configureFeedMotorForNormalOperation();
            moveFeedMotorToHome();
            returningNo2x4Step = 7; // Directly advance step
            break;
            
        case 7: // Was original returningNo2x4Step 5: wait for feed motor at home, retract feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 8
            }
            break;
            
        case 8: // Was original returningNo2x4Step 6: move feed motor to final position
            configureFeedMotorForNormalOperation();
            moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
            returningNo2x4Step = 9; // Directly advance step
            break;
            
        case 9: // Execute comprehensive cut motor homing with incremental moves
            handleCutMotorHomingWithIncrementalMoves();
            break;
            
        case 10: // Execute feed motor homing sequence
            handleFeedMotorHomingSequenceNo2x4();
            break;
            
        case 11: // Final step: wait for both sensors to be not active, then finish sequence
            if (feedMotor && !feedMotor->isRunning()) {
                // Wait until both sensors are not active before proceeding
                if (checkBothSensorsNotActive()) {
                    // Both sensors not active - extend secure wood clamp and complete sequence
                    extend2x4SecureClamp();
                    //serial.println("ReturningNo2x4: Both sensors not active - extending secure wood clamp and completing sequence");
                    
                    turnYellowLedOff();
                    turnBlueLedOn(); 

                    resetReturningNo2x4Steps();
                    setCuttingCycleInProgress(false);
                    
                    // Check if cycle switch is currently ON - if yes, require cycling
                    if (getStartCycleSwitch()->read() == HIGH) {
                        setStartSwitchSafe(false);
                    }
                    
                    // Set flag to indicate we're coming from no-wood cycle with sensors clear
                    setComingFromNoWoodWithSensorsClear(true);
                    
                    // Transition to IDLE state - secure clamp will remain extended
                    changeState(IDLE);
                } else {
                    // At least one sensor is still active - keep waiting
                    // Keep both clamps retracted while waiting
                    retractFeedClamp();
                    retract2x4SecureClamp();
                    //serial.println("ReturningNo2x4: Waiting for both sensors to be not active...");
                }
            }
            break;
    }
}

bool checkBothSensorsNotActive() {
    // Read both sensors with multiple samples for reliability
    bool firstCutSensorNotActive = true;
    bool woodPresentSensorNotActive = true;
    
    // Take multiple readings for stability
    for (int i = 0; i < 3; i++) {
        delay(10); // Small delay between readings
        
        // Check FIRST_CUT_OR_WOOD_FWD_ONE sensor (HIGH = not active)
        if (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == LOW) {
            firstCutSensorNotActive = false;
        }
        
        // Check _2x4_PRESENT_SENSOR (HIGH = not active)  
        if (digitalRead(_2x4_PRESENT_SENSOR) == LOW) {
            woodPresentSensorNotActive = false;
        }
    }
    
    // Return true only if both sensors are not active (both HIGH)
    return (firstCutSensorNotActive && woodPresentSensorNotActive);
}

//* ************************************************************************
//* ****************** CUT MOTOR HOMING WITH INCREMENTAL MOVES ************
//* ************************************************************************
// Handles comprehensive cut motor homing with incremental move recovery

void handleCutMotorHomingWithIncrementalMoves() {
    FastAccelStepper* cutMotor = getCutMotor();
    extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
    extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
    extern const float CUT_MOTOR_STEPS_PER_INCH;
    
    bool sensorDetectedHome = false;
    for (int i = 0; i < 3; i++) {
        delay(30);  
        getCutHomingSwitch()->update();
        bool sensorReading = getCutHomingSwitch()->read();
        
        if (sensorReading == HIGH) {
            sensorDetectedHome = true;
            if (cutMotor) cutMotor->setCurrentPosition(0); 
            cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
            returningNo2x4Step = 10; // Move to feed motor homing
            break;  
        }
    }
    
    if (!sensorDetectedHome) {
        if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
            Serial.print("Attempting incremental move. Total moved: ");
            Serial.print(cutMotorIncrementalMoveTotalInches);
            Serial.println(" inches.");
            if (cutMotor) {
                cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
            }
            // Stay in same step to re-check sensor after move
        } else {
            // Max incremental moves exceeded - transition to error
            Serial.println("ERROR: Cut motor position switch did not detect home after MAX incremental moves!");
            if (cutMotor) cutMotor->forceStop();
            extend2x4SecureClamp();
            turnRedLedOn();
            turnYellowLedOff();
            changeState(ERROR);
            setErrorStartTime(millis());
            resetReturningNo2x4Steps();
        }
    }
}

//* ************************************************************************
//* ****************** FEED MOTOR HOMING SEQUENCE **************************
//* ************************************************************************
// Handles the comprehensive feed motor homing sequence

void handleFeedMotorHomingSequenceNo2x4() {
    FastAccelStepper* feedMotor = getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED;
    extern const float FEED_TRAVEL_DISTANCE;
    extern const float FEED_MOTOR_STEPS_PER_INCH;
    
    // Non-blocking feed motor homing sequence
    switch (feedMotorHomingSubStep) {
        case 0: // Start homing - move toward home sensor
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward sensor
            }
            feedMotorHomingSubStep = 1;
            break;
            
        case 1: // Wait for home sensor to trigger
            getFeedHomingSwitch()->update();
            if (getFeedHomingSwitch()->read() == LOW) {
                if (feedMotor) {
                    feedMotor->stopMove();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                feedMotorHomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.1 inch from sensor
            if (feedMotor && !feedMotor->isRunning()) {
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                feedMotorHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (feedMotor && !feedMotor->isRunning()) {
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                configureFeedMotorForNormalOperation();
                feedMotorHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - transition to final step
            returningNo2x4Step = 11; // Move to final completion step
            break;
    }
}

void resetReturningNo2x4Steps() {
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    feedMotorHomingSubStep = 0;
} 