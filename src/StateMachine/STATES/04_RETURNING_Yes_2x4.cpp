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
// Includes final feed motor homing sequence before transitioning to next cycle or IDLE.

void ReturningYes2x4State::execute(StateManager& stateManager) {
    handleReturningYes2x4Sequence(stateManager);
}

void ReturningYes2x4State::onEnter(StateManager& stateManager) {
    //! ************************************************************************
    //! INITIALIZE RETURNING YES 2X4 SEQUENCE - SIMULTANEOUS RETURN
    //! ************************************************************************
    
    // Enable cut motor homing sensor monitoring during return
    extern bool cutMotorInReturningYes2x4Return;
    cutMotorInReturningYes2x4Return = true;
    
    //! ************************************************************************
    //! STEP 1: START CUT MOTOR RETURN AND RETRACT 2X4 SECURE CLAMP
    //! ************************************************************************
    moveCutMotorToHome();
    retract2x4SecureClamp();
    
    // Initialize step tracking
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorFinalVerificationStartTime = 0;
    cutMotorFinalVerificationInProgress = false;
}

void ReturningYes2x4State::onExit(StateManager& stateManager) {
    resetSteps();
}

//* ************************************************************************
//* ******************** MAIN SEQUENCE HANDLER ****************************
//* ************************************************************************
// Manages the complete RETURNING_YES_2x4 sequence through multiple substeps

void ReturningYes2x4State::handleReturningYes2x4Sequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float FEED_TRAVEL_DISTANCE;
    extern bool cutMotorInReturningYes2x4Return;
    
    switch (returningYes2x4SubStep) {
        case 0: // Execute feed motor return sequence
            handleFeedMotorReturnSequence(stateManager);
            break;
            
        case 1: // Wait for feed motor to complete return home
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP: FEED MOTOR HOME COMPLETE - ENGAGE FEED CLAMP
                //! ************************************************************************
                extendFeedClamp();
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor to complete return home and verify position
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //! ************************************************************************
                //! STEP: CUT MOTOR HOME COMPLETE - VERIFY POSITION WITH ERROR HANDLING
                //! ************************************************************************
                cutMotorInReturningYes2x4Return = false;

                // Non-blocking sensor stabilization - start timing if not already started
                if (cutMotorFinalVerificationStartTime == 0) {
                    cutMotorFinalVerificationStartTime = millis();
                    return; // Exit and wait for stabilization period
                }
                
                // Check if stabilization period has elapsed
                if (millis() - cutMotorFinalVerificationStartTime < SENSOR_STABILIZATION_DELAY_MS) {
                    return; // Still waiting for stabilization
                }
                
                // Stabilization complete - read sensor
                stateManager.getCutHomingSwitch()->update();
                bool sensorReading = stateManager.getCutHomingSwitch()->read();
                
                if (sensorReading == HIGH) {
                    // Initial homing successful - start final verification
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    
                    cutMotorFinalVerificationInProgress = true;
                    cutMotorFinalVerificationStartTime = millis();
                    // DO NOT proceed to feed motor operations yet - wait for final verification
                } else {
                    // Home switch not detected - start recovery attempt and BLOCK feed motor operations
                    cutMotorHomingAttemptInProgress = true;
                    cutMotorHomingAttemptStartTime = millis();
                    
                    // Move cut motor toward home switch
                    if (cutMotor) {
                        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
                        cutMotor->moveTo(-LARGE_POSITION_VALUE); // Move toward home switch
                    }
                    // DO NOT advance returningYes2x4SubStep - stay in case 2 until cut motor homes
                }
            }
            
            // Handle cut motor homing recovery attempt - FEED MOTOR OPERATIONS REMAIN BLOCKED
            if (cutMotorHomingAttemptInProgress) {
                stateManager.getCutHomingSwitch()->update();
                bool sensorReading = stateManager.getCutHomingSwitch()->read();
                
                if (sensorReading == HIGH) {
                    // Recovery successful - start final verification
                    if (cutMotor) {
                        cutMotor->forceStop();
                        cutMotor->setCurrentPosition(0);
                    }
                    cutMotorHomingAttemptInProgress = false;
                    
                    cutMotorFinalVerificationInProgress = true;
                    cutMotorFinalVerificationStartTime = millis();
                    // DO NOT proceed to feed motor operations yet - wait for final verification
                } else if (millis() - cutMotorHomingAttemptStartTime > CUT_MOTOR_RECOVERY_TIMEOUT_MS) {
                    // Timeout exceeded - transition to error state
                    if (cutMotor) cutMotor->forceStop();
                    cutMotorHomingAttemptInProgress = false;
                    
                    stateManager.changeState(Cut_Motor_Homing_Error);
                    resetSteps();
                    return;
                }
            }
            
            // Handle final verification after successful cut motor homing
            if (cutMotorFinalVerificationInProgress) {
                if (millis() - cutMotorFinalVerificationStartTime >= CUT_MOTOR_VERIFICATION_DELAY_MS) {
                    // Verification period elapsed - perform final verification
                    stateManager.getCutHomingSwitch()->update();
                    bool finalSensorReading = stateManager.getCutHomingSwitch()->read();
                    
                    if (finalSensorReading == HIGH) {
                        // FINAL VERIFICATION SUCCESSFUL - NOW safe to proceed with feed motor
                        cutMotorFinalVerificationInProgress = false;
                        
                        //! ************************************************************************
                        //! STEP: VERIFICATION COMPLETE - FEED MOTOR OPERATIONS NOW AUTHORIZED
                        //! ************************************************************************
                        retract2x4SecureClamp();
                        configureFeedMotorForNormalOperation();
                        moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                        returningYes2x4SubStep = 3;
                    } else {
                        // FINAL VERIFICATION FAILED - cut motor not stable at home
                        cutMotorFinalVerificationInProgress = false;
                        stateManager.changeState(Cut_Motor_Homing_Error);
                        resetSteps();
                        return;
                    }
                }
            }
            break;
            
        case 3: // Wait for feed motor final positioning
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP: START END-OF-CYCLE FEED MOTOR HOMING SEQUENCE
                //! ************************************************************************
                retractFeedClamp();
                returningYes2x4SubStep = 4;
                feedHomingSubStep = 0;
            }
            break;
            
        case 4: // Execute feed motor homing sequence
            handleReturningYes2x4FeedMotorHoming(stateManager);
            break;
    }
}

//* ************************************************************************
//* ****************** FEED MOTOR RETURN SEQUENCE **************************
//* ************************************************************************
// Handles the multi-step feed motor return sequence during simultaneous operation

void ReturningYes2x4State::handleFeedMotorReturnSequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    
    switch (feedMotorReturnSubStep) {
        case 0: // Move feed motor back specified distance
            //! ************************************************************************
            //! STEP 2: MOVE FEED MOTOR RETURN DISTANCE
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
                //! STEP 3: RETRACT FEED CLAMP AND EXTEND 2X4 SECURE CLAMP
                //! ************************************************************************
                retractFeedClamp();
                extend2x4SecureClamp();
                feedMotorReturnSubStep = 2;
            }
            break;
            
        case 2: // Start feed motor return to home
            //! ************************************************************************
            //! STEP 4: RETURN FEED MOTOR TO HOME POSITION
            //! ************************************************************************
            if (feedMotor) {
                moveFeedMotorToHome();
            }
            returningYes2x4SubStep = 1;
            break;
    }
}

//* ************************************************************************
//* **************** FEED MOTOR HOMING SEQUENCE ****************************
//* ************************************************************************
// Executes final feed motor homing sequence before cycle completion

void ReturningYes2x4State::handleReturningYes2x4FeedMotorHoming(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    
    switch (feedHomingSubStep) {
        case 0: // Start homing movement toward switch
            //! ************************************************************************
            //! HOMING STEP 1: MOVE TOWARD HOME SWITCH
            //! ************************************************************************
            extend2x4SecureClamp();
            
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(LARGE_POSITION_VALUE * FEED_MOTOR_STEPS_PER_INCH);
            }
            feedHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch trigger
            //! ************************************************************************
            //! HOMING STEP 2: DETECT HOME SWITCH ACTIVATION
            //! ************************************************************************
            stateManager.getFeedHomingSwitch()->update();
            if (stateManager.getFeedHomingSwitch()->read() == HIGH) {
                if (feedMotor) {
                    feedMotor->forceStop();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                feedHomingSubStep = 2;
            }
            break;
            
        case 2: // Move to working position offset from switch
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! HOMING STEP 3: MOVE TO WORKING ZERO POSITION
                //! ************************************************************************
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - FEED_MOTOR_OFFSET_FROM_SWITCH * FEED_MOTOR_STEPS_PER_INCH);
                feedHomingSubStep = 3;
            }
            break;
            
        case 3: // Set new zero position
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! HOMING STEP 4: ESTABLISH NEW WORKING ZERO
                //! ************************************************************************
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                configureFeedMotorForNormalOperation();
                feedHomingSubStep = 4;
            }
            break;
            
        case 4: // Complete homing and determine next state
            //! ************************************************************************
            //! HOMING COMPLETE: CHECK FOR CONTINUOUS OPERATION OR RETURN TO IDLE
            //! ************************************************************************
            extend2x4SecureClamp();
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Check for continuous operation mode
            if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                extendFeedClamp();
                configureCutMotorForCutting();
                turnYellowLedOn();
                stateManager.setCuttingCycleInProgress(true);
                stateManager.changeState(CUTTING);
                resetSteps();
            } else {
                stateManager.changeState(IDLE);
                resetSteps();
            }
            break;
    }
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void ReturningYes2x4State::resetSteps() {
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    feedHomingSubStep = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorFinalVerificationStartTime = 0;
    cutMotorFinalVerificationInProgress = false;
} 