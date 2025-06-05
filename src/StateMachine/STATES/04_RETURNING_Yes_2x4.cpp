#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "../../../include/StateMachine/04_RETURNING_Yes_2x4.h"
#include "../../../include/StateMachine/StateManager.h"
#include "../../../include/StateMachine/FUNCTIONS/General_Functions.h"
#include "../../../include/Config/Pins_Definitions.h"

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
    Serial.println("Entering RETURNING_YES_2x4 state");
    
    //! ************************************************************************
    //! INITIALIZE RETURNING YES 2X4 SEQUENCE - SIMULTANEOUS RETURN
    //! ************************************************************************
    Serial.println("RETURNING_YES_2x4 state - Wood sensor reads LOW. Starting simultaneous return sequence.");
    
    // Enable cut motor homing sensor monitoring during return
    extern bool cutMotorInReturningYes2x4Return;
    cutMotorInReturningYes2x4Return = true;
    
    //! ************************************************************************
    //! STEP 1: START CUT MOTOR RETURN AND RETRACT 2X4 SECURE CLAMP
    //! ************************************************************************
    moveCutMotorToHome();
    Serial.println("Cut motor started returning home.");
    
    retract2x4SecureClamp();
    Serial.println("Step 1: 2x4 secure clamp retracted.");
    
    // Initialize step tracking
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorFinalVerificationStartTime = 0;
    cutMotorFinalVerificationInProgress = false;
}

void ReturningYes2x4State::onExit(StateManager& stateManager) {
    Serial.println("Exiting RETURNING_YES_2x4 state");
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
                Serial.println("RETURNING_YES_2x4: Feed motor has returned home. Engaging feed clamp.");
                extendFeedClamp();
                Serial.println("Feed clamp engaged (feed motor home).");
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor to complete return home and verify position
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //! ************************************************************************
                //! STEP: CUT MOTOR HOME COMPLETE - VERIFY POSITION WITH ERROR HANDLING
                //! ************************************************************************
                Serial.println("RETURNING_YES_2x4: Cut motor has returned home.");
                cutMotorInReturningYes2x4Return = false;

                // Wait 30ms then check cut motor homing switch
                delay(30);
                stateManager.getCutHomingSwitch()->update();
                bool sensorReading = stateManager.getCutHomingSwitch()->read();
                
                Serial.print("Cut position switch reading after 30ms delay: ");
                Serial.println(sensorReading ? "HIGH" : "LOW");
                
                if (sensorReading == HIGH) {
                    // Initial homing successful - start 20ms final verification
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                    Serial.println("Starting 20ms final verification before feed motor operations...");
                    
                    cutMotorFinalVerificationInProgress = true;
                    cutMotorFinalVerificationStartTime = millis();
                    // DO NOT proceed to feed motor operations yet - wait for final verification
                } else {
                    // Home switch not detected - start recovery attempt and BLOCK feed motor operations
                    Serial.println("CRITICAL: Cut motor home switch not detected. Feed motor operations BLOCKED until cut motor homes.");
                    Serial.println("Starting cut motor recovery homing attempt...");
                    cutMotorHomingAttemptInProgress = true;
                    cutMotorHomingAttemptStartTime = millis();
                    
                    // Move cut motor toward home switch
                    if (cutMotor) {
                        extern const float CUT_MOTOR_HOMING_SPEED;
                        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
                        cutMotor->moveTo(-10000); // Move toward home switch
                    }
                    Serial.println("Cut motor moving toward home switch for recovery. Feed operations suspended.");
                    // DO NOT advance returningYes2x4SubStep - stay in case 2 until cut motor homes
                }
            }
            
            // Handle cut motor homing recovery attempt - FEED MOTOR OPERATIONS REMAIN BLOCKED
            if (cutMotorHomingAttemptInProgress) {
                stateManager.getCutHomingSwitch()->update();
                bool sensorReading = stateManager.getCutHomingSwitch()->read();
                
                if (sensorReading == HIGH) {
                    // Recovery successful - start 20ms final verification
                    Serial.println("Cut motor home switch detected during recovery attempt.");
                    if (cutMotor) {
                        cutMotor->forceStop();
                        cutMotor->setCurrentPosition(0);
                    }
                    cutMotorHomingAttemptInProgress = false;
                    Serial.println("Cut motor recovery successful. Position set to 0.");
                    Serial.println("Starting 20ms final verification after recovery before feed motor operations...");
                    
                    cutMotorFinalVerificationInProgress = true;
                    cutMotorFinalVerificationStartTime = millis();
                    // DO NOT proceed to feed motor operations yet - wait for final verification
                } else if (millis() - cutMotorHomingAttemptStartTime > 2000) {
                    // Timeout exceeded - transition to error state
                    Serial.println("ERROR: Cut motor failed to reach home switch within 2 seconds.");
                    Serial.println("SAFETY: Feed motor operations prevented. Transitioning to CUT_MOTOR_ERROR state.");
                    
                    if (cutMotor) cutMotor->forceStop();
                    cutMotorHomingAttemptInProgress = false;
                    
                    stateManager.changeState(Cut_Motor_Homing_Error);
                    resetSteps();
                    return;
                }
            }
            
            // Handle 20ms final verification after successful cut motor homing
            if (cutMotorFinalVerificationInProgress) {
                if (millis() - cutMotorFinalVerificationStartTime >= 20) {
                    // 20ms has elapsed - perform final verification
                    stateManager.getCutHomingSwitch()->update();
                    bool finalSensorReading = stateManager.getCutHomingSwitch()->read();
                    
                    Serial.print("20ms final verification - Cut motor home switch reading: ");
                    Serial.println(finalSensorReading ? "HIGH" : "LOW");
                    
                    if (finalSensorReading == HIGH) {
                        // FINAL VERIFICATION SUCCESSFUL - NOW safe to proceed with feed motor
                        cutMotorFinalVerificationInProgress = false;
                        
                        //! ************************************************************************
                        //! STEP: 20MS VERIFICATION COMPLETE - FEED MOTOR OPERATIONS NOW AUTHORIZED
                        //! ************************************************************************
                        Serial.println("20ms verification PASSED. Cut motor confirmed stable at home. Feed motor operations authorized.");
                        
                        retract2x4SecureClamp();
                        Serial.println("2x4 secure clamp retracted after final cut motor verification.");

                        Serial.println("Cut motor FINAL verification complete. Proceeding to move feed motor to final travel position.");
                        configureFeedMotorForNormalOperation();
                        moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                        returningYes2x4SubStep = 3;
                    } else {
                        // FINAL VERIFICATION FAILED - cut motor not stable at home
                        Serial.println("CRITICAL ERROR: 20ms final verification FAILED. Cut motor home switch not stable.");
                        Serial.println("SAFETY: Cut motor position unstable. Transitioning to CUT_MOTOR_ERROR state.");
                        
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
                Serial.println("RETURNING_YES_2x4: Feed motor at final position. Starting end-of-cycle feed motor homing sequence.");
                
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Starting feed motor homing sequence...");
                
                returningYes2x4SubStep = 4;
                feedHomingSubStep = 0;
                Serial.println("Transitioning to feed motor homing sequence."); 
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
    extern const float FEED_MOTOR_STEPS_PER_INCH;
    
    switch (feedMotorReturnSubStep) {
        case 0: // Move feed motor back 0.1 inches
            //! ************************************************************************
            //! STEP 2: MOVE FEED MOTOR -0.1 INCHES
            //! ************************************************************************
            Serial.println("Feed Motor Return Step 2: Moving feed motor -0.1 inches.");
            configureFeedMotorForReturn();
            if (feedMotor) {
                feedMotor->move(-0.1 * FEED_MOTOR_STEPS_PER_INCH);
            }
            feedMotorReturnSubStep = 1;
            break;
            
        case 1: // Wait for move completion then adjust clamps
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 3: RETRACT FEED CLAMP AND EXTEND 2X4 SECURE CLAMP
                //! ************************************************************************
                Serial.println("Feed Motor Return Step 3: Retracting feed clamp and extending 2x4 secure clamp.");
                retractFeedClamp();
                extend2x4SecureClamp();
                Serial.println("Feed clamp retracted and 2x4 secure clamp extended.");
                feedMotorReturnSubStep = 2;
            }
            break;
            
        case 2: // Start feed motor return to home
            //! ************************************************************************
            //! STEP 4: RETURN FEED MOTOR TO HOME POSITION
            //! ************************************************************************
            Serial.println("Feed Motor Return Step 4: Starting feed motor return to home.");
            if (feedMotor) {
                moveFeedMotorToHome();
            }
            Serial.println("Feed motor returning to home position.");
            
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
    extern const float FEED_MOTOR_HOMING_SPEED;
    extern const float FEED_TRAVEL_DISTANCE;
    
    switch (feedHomingSubStep) {
        case 0: // Start homing movement toward switch
            //! ************************************************************************
            //! HOMING STEP 1: MOVE TOWARD HOME SWITCH
            //! ************************************************************************
            Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 0: Moving toward home switch.");
            extend2x4SecureClamp();
            
            Serial.println("2x4 secure clamp extended before final positioning.");
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH);
            }
            feedHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch trigger
            //! ************************************************************************
            //! HOMING STEP 2: DETECT HOME SWITCH ACTIVATION
            //! ************************************************************************
            stateManager.getFeedHomingSwitch()->update();
            if (stateManager.getFeedHomingSwitch()->read() == HIGH) {
                Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 1: Home switch triggered. Stopping motor.");
                if (feedMotor) {
                    feedMotor->forceStop();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("Feed motor hit home switch.");
                feedHomingSubStep = 2;
            }
            break;
            
        case 2: // Move to working position offset from switch
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! HOMING STEP 3: MOVE TO WORKING ZERO POSITION
                //! ************************************************************************
                Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 2: Moving to -0.2 inch from home switch to establish working zero.");
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 1.0 * FEED_MOTOR_STEPS_PER_INCH);
                feedHomingSubStep = 3;
            }
            break;
            
        case 3: // Set new zero position
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! HOMING STEP 4: ESTABLISH NEW WORKING ZERO
                //! ************************************************************************
                Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 3: Setting new working zero position.");
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                Serial.println("Feed motor homed: 0.2 inch from switch set as position 0.");
                
                configureFeedMotorForNormalOperation();
                feedHomingSubStep = 4;
            }
            break;
            
        case 4: // Complete homing and determine next state
            //! ************************************************************************
            //! HOMING COMPLETE: CHECK FOR CONTINUOUS OPERATION OR RETURN TO IDLE
            //! ************************************************************************
            Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 4: Homing sequence complete.");
            extend2x4SecureClamp(); 
            Serial.println("2x4 secure clamp engaged."); 
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Check for continuous operation mode
            if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                Serial.println("Start cycle switch is active - continuing with another cut cycle.");
                extendFeedClamp();
                configureCutMotorForCutting();
                turnYellowLedOn();
                stateManager.setCuttingCycleInProgress(true);
                stateManager.changeState(CUTTING);
                resetSteps();
                Serial.println("Transitioning to CUTTING state for continuous operation.");
            } else {
                Serial.println("Cycle complete. Transitioning to IDLE state.");
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