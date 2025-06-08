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
    
    // Increment consecutive yeswood counter
    stateManager.incrementConsecutiveYeswoodCount();
    Serial.print("DEBUG: Consecutive yeswood count: ");
    Serial.println(stateManager.getConsecutiveYeswoodCount());
    
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
    
    Serial.print("DEBUG: RETURNING_YES_2x4 SubStep: ");
    Serial.println(returningYes2x4SubStep);
    
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

        case 2: // Wait for cut motor to complete return home, then execute homing sequence
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //! ************************************************************************
                //! STEP: CUT MOTOR RETURN COMPLETE - START HOMING VERIFICATION SEQUENCE
                //! ************************************************************************
                cutMotorInReturningYes2x4Return = false;
                
                Serial.println("DEBUG: Cut motor stopped at -0.02, starting homing verification");
                bool sensorDetectedHome = false;
                
                // Execute homing verification sequence - 3-attempt verification
                for (int i = 0; i < 3; i++) {
                    delay(30);
                    stateManager.getCutHomingSwitch()->update();
                    bool sensorReading = stateManager.getCutHomingSwitch()->read();
                    Serial.print("DEBUG: Cut home verification attempt ");
                    Serial.print(i + 1);
                    Serial.print(" of 3: ");
                    Serial.println(sensorReading ? "HIGH" : "LOW");
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        Serial.println("DEBUG: Cut motor home position confirmed during homing sequence");
                        break;
                    }
                }
                
                if (sensorDetectedHome) {
                    //! ************************************************************************
                    //! STEP: HOMING VERIFIED - SET POSITION TO 0 AND PROCEED WITH FEED MOTOR
                    //! ************************************************************************
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    Serial.println("DEBUG: Cut motor position set to 0 after homing verification");
                    
                    retract2x4SecureClamp();
                    configureFeedMotorForNormalOperation();
                    moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                    returningYes2x4SubStep = 3;
                } else {
                    // Home switch not detected during homing sequence - start recovery attempt
                    Serial.println("DEBUG: Homing sequence failed - sensor LOW - starting recovery attempt");
                    cutMotorHomingAttemptInProgress = true;
                    cutMotorHomingAttemptStartTime = millis();
                    
                    // Move cut motor toward home switch
                    if (cutMotor) {
                        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
                        cutMotor->moveTo(-LARGE_POSITION_VALUE); // Move toward home switch
                    }
                }
            }
            
            // Handle cut motor homing recovery attempt
            if (cutMotorHomingAttemptInProgress) {
                stateManager.getCutHomingSwitch()->update();
                bool sensorReading = stateManager.getCutHomingSwitch()->read();
                
                if (sensorReading == HIGH) {
                    // Recovery successful - set position to 0 and proceed
                    if (cutMotor) {
                        cutMotor->forceStop();
                        cutMotor->setCurrentPosition(0);
                    }
                    cutMotorHomingAttemptInProgress = false;
                    
                    Serial.println("DEBUG: Recovery successful - position set to 0 - proceeding with feed motor");
                    retract2x4SecureClamp();
                    configureFeedMotorForNormalOperation();
                    moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                    returningYes2x4SubStep = 3;
                    
                } else if (millis() - cutMotorHomingAttemptStartTime > CUT_MOTOR_RECOVERY_TIMEOUT_MS) {
                    // Timeout exceeded - transition to error state
                    if (cutMotor) cutMotor->forceStop();
                    cutMotorHomingAttemptInProgress = false;
                    
                    stateManager.changeState(Cut_Motor_Homing_Error);
                    resetSteps();
                    return;
                }
            }
            break;
            
        case 3: // Wait for feed motor final positioning
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP: START FEED MOTOR HOMING SEQUENCE (RUNS EVERY TIME)
                //! ************************************************************************
                Serial.println("DEBUG: Starting feed motor homing sequence");
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
        case 0: // Start homing movement toward sensor
            //! ************************************************************************
            //! HOMING STEP 1: MOVE TOWARD HOME SENSOR USING CONTINUOUS FORWARD RUN
            //! ************************************************************************
            extend2x4SecureClamp();
            
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->runForward();  // Use runForward() like the working blocking function
            }
            feedHomingSubStep = 1;
            break;
            
        case 1: // Wait for home sensor trigger with continuous monitoring
            //! ************************************************************************
            //! HOMING STEP 2: DETECT HOME SENSOR ACTIVATION - CONTINUOUS MONITORING
            //! ************************************************************************
            // Check if motor stopped unexpectedly and restart it
            if (feedMotor && !feedMotor->isRunning()) {
                feedMotor->runForward();
            }
            
            // Continuous sensor monitoring - check multiple times per loop to prevent missing sensor activation
            for (int i = 0; i < 10; i++) {  // Check sensor 10 times per main loop iteration
                stateManager.getFeedHomingSwitch()->update();
                if (stateManager.getFeedHomingSwitch()->read() == LOW) {
                    if (feedMotor) {
                        feedMotor->forceStop();
                        feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                    }
                    feedHomingSubStep = 2;
                    return;  // Exit immediately when sensor detected
                }
                delayMicroseconds(100);  // Small delay between checks
            }
            break;
            
        case 2: // Move to working position offset from sensor
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! HOMING STEP 3: MOVE TO WORKING ZERO POSITION
                //! ************************************************************************
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - FEED_MOTOR_OFFSET_FROM_SENSOR * FEED_MOTOR_STEPS_PER_INCH);
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
            //! HOMING COMPLETE: RESET COUNTER IF AT 3 AND CHECK FOR CONTINUOUS OPERATION OR RETURN TO IDLE
            //! ************************************************************************
            extend2x4SecureClamp();
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Reset consecutive yeswood counter only when it reaches 3
            if (stateManager.getConsecutiveYeswoodCount() >= 3) {
                stateManager.resetConsecutiveYeswoodCount();
                Serial.println("DEBUG: Homing sequence complete - consecutive yeswood counter reset at 3");
            } else {
                Serial.print("DEBUG: Homing sequence complete - consecutive yeswood count: ");
                Serial.println(stateManager.getConsecutiveYeswoodCount());
            }
            
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
} 