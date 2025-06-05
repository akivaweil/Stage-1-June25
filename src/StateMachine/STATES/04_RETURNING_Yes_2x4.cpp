#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "../../../include/StateMachine/04_RETURNING_Yes_2x4.h"
#include "../../../include/StateMachine/StateManager.h"
#include "../../../include/StateMachine/FUNCTIONS/General_Functions.h"
#include "../../../include/Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ RETURNING YES 2X4 STATE **********************
//* ************************************************************************
// Handles the RETURNING_YES_2x4 cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

void ReturningYes2x4State::execute(StateManager& stateManager) {
    handleReturningYes2x4Sequence(stateManager);
}

void ReturningYes2x4State::onEnter(StateManager& stateManager) {
    Serial.println("Entering RETURNING_YES_2x4 state");
    
    // Initialize RETURNING_YES_2x4 sequence from CUTTING_state logic
    Serial.println("RETURNING_YES_2x4 state - Wood sensor reads LOW. Starting RETURNING_YES_2x4 Sequence (simultaneous return).");
    
    // Set flag to indicate we're in RETURNING_YES_2x4 return mode - enable homing sensor check
    extern bool cutMotorInReturningYes2x4Return; // From main.cpp
    cutMotorInReturningYes2x4Return = true;
    
    // Start cut motor return immediately
    moveCutMotorToHome();
    Serial.println("Cut motor started returning home.");
    
    // Start feed motor sequence - Step 1: Retract 2x4 secure clamp
    retract2x4SecureClamp();
    Serial.println("Step 1: 2x4 secure clamp retracted.");
    
    // Initialize step tracking - start with feed motor sequence
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
}

void ReturningYes2x4State::onExit(StateManager& stateManager) {
    Serial.println("Exiting RETURNING_YES_2x4 state");
    resetSteps();
}

void ReturningYes2x4State::handleReturningYes2x4Sequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern bool cutMotorInReturningYes2x4Return; // From main.cpp
    
    // This step handles the completion of the "RETURNING_YES_2x4 Sequence".
    switch (returningYes2x4SubStep) {
        case 0: // Handle initial feed motor return sequence
            handleFeedMotorReturnSequence(stateManager);
            break;
            
        case 1: // Wait for feed motor to home
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Step 1: Feed motor has returned home. Engaging feed clamp.");
                extendFeedClamp();
                Serial.println("Feed clamp engaged (feed motor home).");
                returningYes2x4SubStep = 2;
            }
            break;

        case 2: // Wait for cut motor to home, then proceed with original logic
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Step 2: Cut motor has returned home.");
                // Clear the RETURNING_YES_2x4 return flag since cut motor has stopped
                cutMotorInReturningYes2x4Return = false;

                // Check the cut motor homing switch. Enhanced diagnostics from RETURNING_No_2x4 approach.
                bool sensorDetectedHome = false;
                Serial.println("Checking cut motor position switch after simultaneous return.");
                for (int i = 0; i < 3; i++) { 
                    delay(30);  
                    stateManager.getCutHomingSwitch()->update();
                    bool sensorReading = stateManager.getCutHomingSwitch()->read();
                    Serial.print("Cut position switch read attempt "); 
                    Serial.print(i+1); 
                    Serial.print(" of 3: "); 
                    Serial.print(sensorReading ? "HIGH" : "LOW");
                    Serial.print(" (raw: ");
                    Serial.print(sensorReading);
                    Serial.println(")");
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                        break; 
                    }
                }

                // ENHANCED DIAGNOSTICS: Proceed but with warning if sensor not detected
                if (!sensorDetectedHome) {
                    Serial.println("WARNING: Cut motor home sensor did NOT detect HIGH during RETURNING_YES_2x4.");
                    Serial.println("DIAGNOSTIC: Proceeding with sequence - check sensor wiring/position if this persists.");
                    Serial.println("DIAGNOSTIC: If cut motor is physically at home, this may indicate a sensor issue.");
                    
                    // Still set position to 0 for operational continuity, but with warning
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    Serial.println("Cut motor position set to 0 despite sensor reading for operational continuity.");
                } else {
                    Serial.println("DIAGNOSTIC: Cut motor home sensor successfully detected HIGH.");
                }

                // Always proceed with next steps (removed error transition)
                retract2x4SecureClamp();
                Serial.println("2x4 secure clamp retracted after cut motor home sequence.");

                Serial.println("Cut motor return sequence complete. Proceeding to move feed motor to final travel position.");
                configureFeedMotorForNormalOperation();
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                returningYes2x4SubStep = 3; // Move to feed motor homing sequence
            }
            break;
            
        case 3: // Wait for feed motor to reach final position, then start homing sequence
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Step 3: Feed motor at final position. Skipping homing sequence for next cycle optimization.");
                
                //! ************************************************************************
                //! STEP: RETRACT FEED CLAMP - HOMING WILL HAPPEN DURING NEXT CUT CYCLE
                //! ************************************************************************
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Homing sequence will occur during next cutting cycle for optimization.");
                
                // Complete the cycle without homing
                extend2x4SecureClamp(); 
                Serial.println("2x4 secure clamp engaged."); 
                turnYellowLedOff();
                stateManager.setCuttingCycleInProgress(false);
                
                // Check if start cycle switch is active for continuous operation
                if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                    Serial.println("Start cycle switch is active - continuing with another cut cycle (feed motor homing will happen during cutting).");
                    // Prepare for next cycle - feed motor homing will happen during cutting
                    extendFeedClamp();
                    configureCutMotorForCutting(); // Ensure cut motor is set to proper cutting speed
                    turnYellowLedOn();
                    stateManager.setCuttingCycleInProgress(true);
                    stateManager.changeState(CUTTING);
                    resetSteps();
                    Serial.println("Transitioning to CUTTING state for continuous operation with concurrent feed motor homing.");
                } else {
                    Serial.println("Cycle complete. Transitioning to IDLE state (feed motor will home during next cycle start).");
                    stateManager.changeState(IDLE);
                    resetSteps();
                }
            }
            break;
    }
}

void ReturningYes2x4State::handleFeedMotorReturnSequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_MOTOR_STEPS_PER_INCH; // From General_Functions.h
    
    // Handle the initial feed motor return sequence (steps 2-4 from user requirements)
    switch (feedMotorReturnSubStep) {
        case 0: // Step 2: Move feed motor -0.1 inches
            Serial.println("Feed Motor Return Step 2: Moving feed motor -0.1 inches.");
            configureFeedMotorForReturn();
            if (feedMotor) {
                feedMotor->move(-0.1 * FEED_MOTOR_STEPS_PER_INCH); // Move -0.1 inches relative
            }
            feedMotorReturnSubStep = 1;
            break;
            
        case 1: // Wait for -0.1 inch move to complete
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("Feed Motor Return Step 3: Retracting feed clamp and extending 2x4 secure clamp.");
                // Step 3: Retract feed clamp and extend 2x4 secure clamp
                retractFeedClamp();
                extend2x4SecureClamp();
                Serial.println("Feed clamp retracted and 2x4 secure clamp extended.");
                feedMotorReturnSubStep = 2;
            }
            break;
            
        case 2: // Step 4: Finish feed motor return to home
            Serial.println("Feed Motor Return Step 4: Starting feed motor return to home.");
            if (feedMotor) {
                moveFeedMotorToHome();
            }
            Serial.println("Feed motor returning to home position.");
            
            // Move to main sequence - wait for motors to complete
            returningYes2x4SubStep = 1;
            break;
    }
}

void ReturningYes2x4State::resetSteps() {
    returningYes2x4SubStep = 0;
    feedMotorReturnSubStep = 0;
} 