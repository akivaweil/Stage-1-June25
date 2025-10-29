#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "../../../include/StateMachine/04_RETURNING_Yes_2x4.h"
#include "../../../include/StateMachine/StateManager.h"
#include "../../../include/StateMachine/FUNCTIONS/General_Functions.h"
#include "../../../include/Config/Pin_Def.h"
#include "../../../include/Config/Motor_Config.h"
#include "../../../include/StateMachine/STATES/States_Config.h"
#include "../../../include/WebSocketDashboard/websocket_dashboard.h"

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
static bool feedWoodMoveStarted = false; // Track if feed wood move has started

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
    feedWoodMoveStarted = false;
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
                //║ STEP 2: FEED MOTOR RETURN COMPLETE - RETRACT SECURE CLAMP AND EXTEND FEED CLAMP ║
                //╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
                retract2x4SecureClamp();
                extendFeedClamp();
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor completion
            // Wait for cut motor to complete return home, then execute homing sequence
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
                //║ STEP 3: CUT MOTOR RETURN COMPLETE - START HOMING VERIFICATION SEQUENCE ║
                //╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
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
                //! STEP 4: HOMING VERIFIED - SET POSITION TO 0 AND PROCEED WITH FEED WOOD MOVEMENT
                //! ************************************************************************
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                    
                    // Advance to feed wood movement sequence - it will handle the motor config and move
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
            
        case 3: // Execute feed wood movement to configured distance
            if (!feedWoodMoveStarted) {
                //! ************************************************************************
                //! STEP 4: START FEED WOOD MOVEMENT
                //! ************************************************************************
                configureFeedMotorForNormalOperation();
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                feedWoodMoveStarted = true;
            }
            
            if (feedMotor && !feedMotor->isRunning() && feedWoodMoveStarted) {
                // Movement complete
                extend2x4SecureClamp();
                returningYes2x4SubStep = 4;
            }
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
            
        case 1: // Move feed motor back by the same distance it will later move forward
            //! ************************************************************************
            //! STEP 7: MOVE FEED MOTOR RETURN DISTANCE
            //! ************************************************************************
            configureFeedMotorForNormalOperation();
            if (feedMotor) {
                feedMotor->move(-FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            }
            feedMotorReturnSubStep = 2;
            break;
            
        case 2: // Wait for move completion
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 8: FEED MOTOR RETURN MOVE COMPLETE
                //! ************************************************************************
                feedMotorReturnSubStep = 3;
            }
            break;
            
        case 3: // Start feed motor return to home
            //! ************************************************************************
            //! STEP 9: RETURN FEED MOTOR TO HOME POSITION
            //! ************************************************************************
            if (feedMotor) {
                // Configure for normal speed before moving to home to prevent stalling
                configureFeedMotorForNormalOperation();
                // Use relative move instead of absolute to position 0 to avoid large position jumps
                feedMotor->move(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            }
            returningYes2x4SubStep = 1;
            break;
    }
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetReturningYes2x4Steps() {
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    feedWoodMoveStarted = false;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
} 