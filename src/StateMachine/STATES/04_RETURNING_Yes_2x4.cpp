#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "../../../include/StateMachine/04_RETURNING_Yes_2x4.h"
#include "../../../include/StateMachine/StateManager.h"
#include "../../../include/StateMachine/FUNCTIONS/General_Functions.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/StateMachine/STATES/States_Config.h"

//* ************************************************************************
//* ******************** RETURNING YES 2X4 STATE **************************
//* ************************************************************************
// Handles the simultaneous return sequence when wood sensor detects lumber.
// Manages cut motor return to home while feed motor executes multi-step return sequence.
// Includes final feed wood movement to 3.4 inches before transitioning to next cycle or IDLE.
// 
// Feed clamp extension occurs immediately after feed motor completion.

// Static variables for returning yes 2x4 state tracking
static int returningYes2x4SubStep = 0;
static int feedMotorReturnSubStep = 0; // For initial feed motor return sequence

// Cut motor homing recovery timing
static unsigned long cutMotorHomingAttemptStartTime = 0;
static bool cutMotorHomingAttemptInProgress = false;
static float cutMotorIncrementalMoveTotalInches = 0.0;

// Feed wood movement sequence tracking
static int feedMotorHomingSubStep = 0;

// Feed clamp extension variables (no delay needed)

void executeReturningYes2x4State() {
    handleReturningYes2x4Sequence();
}

void onEnterReturningYes2x4State() {
    //! ************************************************************************
    //! STEP 1: START CUT MOTOR RETURN AND RETRACT 2X4 SECURE CLAMP
    //! ************************************************************************
    
    // Increment consecutive yeswood counter
    incrementConsecutiveYeswoodCount();
    
    // Enable cut motor homing sensor monitoring during return
    extern bool cutMotorInReturningYes2x4Return;
    cutMotorInReturningYes2x4Return = true;
    
    moveCutMotorToHome();
    retract2x4SecureClamp();
    
    // Initialize step tracking
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    feedMotorHomingSubStep = 0;
}

void onExitReturningYes2x4State() {
    resetReturningYes2x4Steps();
}

//* ************************************************************************
//* ******************** MAIN SEQUENCE HANDLER ****************************
//* ************************************************************************
// Manages the complete RETURNING_YES_2x4 sequence through multiple substeps

void handleReturningYes2x4Sequence() {
    FastAccelStepper* feedMotor = getFeedMotor();
    FastAccelStepper* cutMotor = getCutMotor();
    extern const float FEED_TRAVEL_DISTANCE;
    extern bool cutMotorInReturningYes2x4Return;
    
    switch (returningYes2x4SubStep) {
        case 0: // Execute feed motor return sequence (without homing)
            handleFeedMotorReturnSequence();
            break;
            
        case 1: // Wait for feed motor to complete return movement (no homing)
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 2: FEED MOTOR RETURN COMPLETE - EXTEND FEED CLAMP IMMEDIATELY
                //! ************************************************************************
                extendFeedClamp();
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor completion
            // Wait for cut motor to complete return home, then execute homing sequence
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //! ************************************************************************
                //! STEP 3: CUT MOTOR RETURN COMPLETE - START HOMING VERIFICATION SEQUENCE
                //! ************************************************************************
                cutMotorInReturningYes2x4Return = false;
                
                bool sensorDetectedHome = false;
                
                // Execute homing verification sequence - 3-attempt verification
                for (int i = 0; i < 3; i++) {
                    delay(30);
                    getCutHomingSwitch()->update();
                    bool sensorReading = getCutHomingSwitch()->read();
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        break;
                    }
                }
                
                if (sensorDetectedHome) {
                //! ************************************************************************
                //! STEP 4: HOMING VERIFIED - SET POSITION TO 0 AND PROCEED WITH FEED WOOD MOVEMENT
                //! ************************************************************************
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                    
                    retract2x4SecureClamp();
                    configureFeedMotorForNormalOperation();
                    moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                    returningYes2x4SubStep = 3;
                } else {
                    // Home switch not detected - try incremental move recovery
                    extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                    extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
                    extern const float CUT_MOTOR_STEPS_PER_INCH;
                    
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
                        if (feedMotor) feedMotor->forceStop();
                        extend2x4SecureClamp();
                        turnRedLedOn();
                        turnYellowLedOff();
                        changeState(ERROR);
                        setErrorStartTime(millis());
                        resetReturningYes2x4Steps();
                        return;
                    }
                }
            }
            break;
            
        case 3: // Execute feed wood movement to 3.4 inches
            handleFeedWoodMovement();
            break;
            
        case 4: // Complete sequence - check for continuous operation or return to IDLE
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 5: SEQUENCE COMPLETE - CHECK FOR CONTINUOUS OPERATION OR RETURN TO IDLE
                //! ************************************************************************
                extend2x4SecureClamp();
                turnYellowLedOff();
                setCuttingCycleInProgress(false);
                
                // Reset consecutive yeswood counter only when it reaches 3
                if (getConsecutiveYeswoodCount() >= 3) {
                    resetConsecutiveYeswoodCount();
                }
                
                // Check for continuous operation mode
                if (getStartCycleSwitch()->read() == HIGH && getStartSwitchSafe()) {
                    extendFeedClamp();
                    configureCutMotorForCutting();
                    turnYellowLedOn();
                    setCuttingCycleInProgress(true);
                    changeState(CUTTING);
                    resetReturningYes2x4Steps();
                } else {
                    changeState(IDLE);
                    resetReturningYes2x4Steps();
                }
            }
            break;
    }
}

//* ************************************************************************
//* ****************** FEED MOTOR RETURN SEQUENCE **************************
//* ************************************************************************
// Handles the feed motor return sequence during simultaneous operation (no homing)

void handleFeedMotorReturnSequence() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    switch (feedMotorReturnSubStep) {
        case 0: // Move feed motor back specified distance
            //! ************************************************************************
            //! STEP 6: MOVE FEED MOTOR RETURN DISTANCE
            //! ************************************************************************
            configureFeedMotorForReturn();
            if (feedMotor) {
                feedMotor->move(-FEED_MOTOR_RETURN_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            }
            feedMotorReturnSubStep = 1;
            break;
            
        case 1: // Wait for move completion then adjust clamps
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 7: RETRACT FEED CLAMP AND EXTEND 2X4 SECURE CLAMP
                //! ************************************************************************
                retractFeedClamp();
                extend2x4SecureClamp();
                feedMotorReturnSubStep = 2;
            }
            break;
            
        case 2: // Start feed motor return to home
            //! ************************************************************************
            //! STEP 8: RETURN FEED MOTOR TO HOME POSITION
            //! ************************************************************************
            if (feedMotor) {
                moveFeedMotorToHome();
            }
            returningYes2x4SubStep = 1;
            break;
    }
}

//* ************************************************************************
//* ****************** FEED WOOD MOVEMENT SEQUENCE *************************
//* ************************************************************************
// Handles the feed wood movement to 3.4 inches with feed clamp extended

void handleFeedWoodMovement() {
    FastAccelStepper* feedMotor = getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED;
    extern const float FEED_TRAVEL_DISTANCE;
    extern const float FEED_MOTOR_STEPS_PER_INCH;
    
    // Non-blocking feed wood movement to 3.4 inches
    switch (feedMotorHomingSubStep) {
        case 0: // Start feed wood movement to 3.4 inches
            if (feedMotor) {
                configureFeedMotorForNormalOperation();
                feedMotor->moveTo(3.4 * FEED_MOTOR_STEPS_PER_INCH);
            }
            feedMotorHomingSubStep = 1;
            break;
            
        case 1: // Wait for movement to complete
            if (feedMotor && !feedMotor->isRunning()) {
                feedMotorHomingSubStep = 2;
            }
            break;
            
        case 2: // Movement complete - transition to final step
            extend2x4SecureClamp();
            returningYes2x4SubStep = 4; // Move to final completion step
            break;
    }
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetReturningYes2x4Steps() {
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    feedMotorHomingSubStep = 0;
} 