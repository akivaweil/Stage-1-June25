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

// Static variables for returning yes 2x4 state tracking
static int returningYes2x4SubStep = 0;
static int feedMotorReturnSubStep = 0; // For initial feed motor return sequence

// Cut motor homing recovery timing
static unsigned long cutMotorHomingAttemptStartTime = 0;
static bool cutMotorHomingAttemptInProgress = false;

void executeReturningYes2x4State() {
    handleReturningYes2x4Sequence();
}

void onEnterReturningYes2x4State() {
    //! ************************************************************************
    //! STEP 1: START CUT MOTOR RETURN AND RETRACT 2X4 SECURE CLAMP
    //! ************************************************************************
    
    // Increment consecutive yeswood counter
    incrementConsecutiveYeswoodCount();
    Serial.print("DEBUG: Consecutive yeswood count: ");
    Serial.println(getConsecutiveYeswoodCount());
    
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
    
    Serial.print("DEBUG: RETURNING_YES_2x4 SubStep: ");
    Serial.println(returningYes2x4SubStep);
    
    switch (returningYes2x4SubStep) {
        case 0: // Execute feed motor return sequence
            handleFeedMotorReturnSequence();
            break;
            
        case 1: // Wait for feed motor to complete return home
            if (feedMotor && !feedMotor->isRunning()) {
                //! ************************************************************************
                //! STEP 2: FEED MOTOR HOME COMPLETE - ENGAGE FEED CLAMP
                //! ************************************************************************
                extendFeedClamp();
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor to complete return home, then execute homing sequence
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                //! ************************************************************************
                //! STEP 3: CUT MOTOR RETURN COMPLETE - START HOMING VERIFICATION SEQUENCE
                //! ************************************************************************
                cutMotorInReturningYes2x4Return = false;
                
                Serial.println("DEBUG: Cut motor stopped at -0.02, starting homing verification");
                bool sensorDetectedHome = false;
                
                // Execute homing verification sequence - 3-attempt verification
                for (int i = 0; i < 3; i++) {
                    delay(30);
                    getCutHomingSwitch()->update();
                    bool sensorReading = getCutHomingSwitch()->read();
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
                    //! STEP 4: HOMING VERIFIED - SET POSITION TO 0 AND PROCEED WITH FEED MOTOR
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
                getCutHomingSwitch()->update();
                bool sensorReading = getCutHomingSwitch()->read();
                
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
                    
                    changeState(Cut_Motor_Homing_Error);
                    resetReturningYes2x4Steps();
                    return;
                }
            }
            break;
            
        case 3: // Complete sequence without homing
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
                    Serial.println("DEBUG: Sequence complete - consecutive yeswood counter reset at 3");
                } else {
                    Serial.print("DEBUG: Sequence complete - consecutive yeswood count: ");
                    Serial.println(getConsecutiveYeswoodCount());
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
// Handles the multi-step feed motor return sequence during simultaneous operation

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
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetReturningYes2x4Steps() {
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
} 