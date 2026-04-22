#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "StateMachine/04_RETURNING_Yes_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pin_Def.h"
#include "Config/Config.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ✅ RETURNING YES 2X4 STATE                                           ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Handles the simultaneous return sequence when wood sensor detects lumber.
// Manages cut motor return to home while feed motor executes multi-step return sequence.
// Includes final feed wood movement to configured distance before transitioning to next cycle or IDLE.
// 
// Feed clamp extension occurs immediately after feed motor completion.

// Static variables for returning yes 2x4 state tracking
static int returningYes2x4SubStep = 0;
static int feedMotorReturnSubStep = 0; // For initial feed motor return sequence
static unsigned long stepStartTime = 0; // For 200ms delay

// Cut motor homing recovery timing
static unsigned long cutMotorHomingAttemptStartTime = 0;
static bool cutMotorHomingAttemptInProgress = false;
static float cutMotorIncrementalMoveTotalInches = 0.0;

// Feed clamp extension variables (no delay needed)

void executeReturningYes2x4State() {
    handleReturningYes2x4Sequence();
}

void onEnterReturningYes2x4State() {
    //╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
    //║ STEP 1: START CUT MOTOR RETURN (SECURE CLAMP REMAINS EXTENDED)       ║
    //╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
    
    // Increment consecutive yeswood counter
    incrementConsecutiveYeswoodCount();
    
    // Enable cut motor homing sensor monitoring during return
    extern bool cutMotorInReturningYes2x4Return;
    cutMotorInReturningYes2x4Return = true;
    
    // Cut motor already started in CUTTING state
    // Secure 2x4 clamp remains extended during first feed motor movement
    
    // Initialize step tracking
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    stepStartTime = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
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
    extern bool cutMotorInReturningYes2x4Return;
    
    switch (returningYes2x4SubStep) {
        case 0: // Execute feed motor return sequence (without homing)
            handleFeedMotorReturnSequence();
            break;
            
        case 1: // Wait for feed motor to complete return movement (no homing)
            if (feedMotor && !feedMotor->isRunning()) {
                //╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
                //║ STEP 2: FEED MOTOR RETURN COMPLETE - PROCEED TO CUT MOTOR WAIT       ║
                //╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor completion
            // Wait for cut motor to complete return home, then execute homing sequence
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //╔═══╗ ══════════════════════════════════════════════════════════════════ ╔═══╗
                //║ STEP 3: CUT MOTOR RETURN COMPLETE - START HOMING VERIFICATION SEQUENCE ║
                //╚═══╝ ══════════════════════════════════════════════════════════════════ ╚═══╝
                cutMotorInReturningYes2x4Return = false;
                
                bool sensorDetectedHome = false;
                
                // Execute homing verification sequence - 3-attempt verification
                for (int i = 0; i < 3; i++) {
                    delay(5);
                    getCutHomingSwitch()->update();
                    bool sensorReading = getCutHomingSwitch()->read();
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        break;
                    }
                }
                
                if (sensorDetectedHome) {
                //! ************************************************************************
                //! STEP 4: HOMING VERIFIED - SET POSITION TO 0 AND PROCEED TO COMPLETION
                //! ************************************************************************
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                    
                    // Feed motor already at travel distance from return sequence, proceed to completion
                    returningYes2x4SubStep = 3;
                } else {
                    // Home switch not detected - try incremental move recovery
                    
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
                        onErrorOccurred("Cut motor home switch not detected after max moves");
                        if (cutMotor) cutMotor->forceStop();
                        if (feedMotor) feedMotor->forceStop();
                        showRedLed();
                        turnYellowLedOff();
                        changeState(ERROR);
                        setErrorStartTime(millis());
                        resetReturningYes2x4Steps();
                        return;
                    }
                }
            }
            break;
            
        case 3: // Feed motor already at travel distance - extend secure clamp
            //! ************************************************************************
            //! STEP 4: EXTEND SECURE CLAMP AFTER FEED WOOD MOVEMENT COMPLETE
            //! ************************************************************************
            extend2x4SecureClamp();
            returningYes2x4SubStep = 4;
            break;
            
        case 4: // Complete sequence - check for continuous operation or return to IDLE
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 5: SEQUENCE COMPLETE - CHECK FOR CONTINUOUS OPERATION OR RETURN TO IDLE
                //! ************************************************************************
                turnYellowLedOff();
                incrementCuttingCycleCounter();
                setCuttingCycleInProgress(false);
                
                // Reset consecutive yeswood counter only when it reaches 3
                if (getConsecutiveYeswoodCount() >= 3) {
                    resetConsecutiveYeswoodCount();
                }
                
                
                // Check for continuous operation mode
                if (getStartCycleSwitch()->read() == HIGH && getStartSwitchSafe()) {
                    extendFeedClamp();
                    configureCutMotorForCutting();
                    showYellowLed();
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
        case 0: // Retract feed clamp first
            //! ************************************************************************
            //! STEP 6: RETRACT FEED CLAMP
            //! ************************************************************************
            retractFeedClamp();
            feedMotorReturnSubStep = 1;
            break;
            
        case 1: // Move feed motor to zero (home position)
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 7: MOVE FEED MOTOR TO ZERO
                //! ************************************************************************
                configureFeedMotorForNormalOperation();
                moveFeedMotorToHome();
                feedMotorReturnSubStep = 2;
            }
            break;
            
        case 2: // Wait for move to zero completion, then extend feed clamp and retract secure clamp
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 8: EXTEND FEED CLAMP AND RETRACT SECURE CLAMP WITH 200MS DELAY
                //! ************************************************************************
                extendFeedClamp();
                retract2x4SecureClamp();
                stepStartTime = millis();
                feedMotorReturnSubStep = 3;
            }
            break;
            
        case 3: // Wait minimum 200ms delay AND verify cut motor is home before moving to travel distance
            // Check that minimum 200ms has elapsed since clamp extension
            bool minDelayMet = (millis() - stepStartTime >= 200);
            
            // Safety check: Ensure cut motor is home before moving feed motor forward
            getCutHomingSwitch()->update();
            bool cutMotorIsHome = (getCutHomingSwitch()->read() == HIGH);
            
            // Both conditions must be met: minimum delay AND cut motor home
            if (minDelayMet && cutMotorIsHome && feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 9: MOVE TO TRAVEL DISTANCE (200MS PASSED AND CUT MOTOR HOME VERIFIED)
                //! ************************************************************************
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                returningYes2x4SubStep = 1;
            }
            break;
            
    }
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetReturningYes2x4Steps() {
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    stepStartTime = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
} 