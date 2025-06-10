#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

void ReturningNo2x4State::execute(StateManager& stateManager) {
    handleReturningNo2x4Sequence(stateManager); 
}

void ReturningNo2x4State::onEnter(StateManager& stateManager) {
    //serial.println("Entering RETURNING_NO_2x4 state");
    
    // Reset consecutive yeswood counter when nowood state occurs
    stateManager.resetConsecutiveYeswoodCount();
    Serial.println("DEBUG: RETURNING_No_2x4 state entered - consecutive yeswood counter reset");
    
    // Initialize RETURNING_NO_2x4 sequence from CUTTING_state logic
    //serial.println("RETURNING_NO_2x4 state - Wood sensor reads HIGH. Starting RETURNING_NO_2x4 Sequence.");
    configureCutMotorForReturn();
    moveCutMotorToHome();
    configureFeedMotorForNormalOperation();

    turnBlueLedOn();
    turnYellowLedOff();
    
    // Initialize step tracking
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
}

void ReturningNo2x4State::onExit(StateManager& stateManager) {
    //serial.println("Exiting RETURNING_NO_2x4 state");
    resetSteps();
}

void ReturningNo2x4State::handleReturningNo2x4Sequence(StateManager& stateManager) {
    // RETURNING_NO_2x4 sequence logic
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (returningNo2x4Step == 0) { // First time entering this specific RETURNING_NO_2x4 logic path
        //serial.println("RETURNING_NO_2x4 Step 0: Initiating feed motor to home & retracting 2x4 secure clamp.");
        retract2x4SecureClamp();
        if (feedMotor) {
            if (feedMotor->getCurrentPosition() != 0 || feedMotor->isRunning()) {
                feedMotor->moveTo(0);
                //serial.println("RETURNING_NO_2x4 Step 0: Feed motor commanded to home.");
            } else {
                //serial.println("RETURNING_NO_2x4 Step 0: Feed motor already at home.");
            }
        }
        returningNo2x4Step = 1;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        returningNo2x4Step++; 
    }
    
    if (!waitingForCylinder) {
        handleReturningNo2x4Step(stateManager, returningNo2x4Step);
    }
}

void ReturningNo2x4State::handleReturningNo2x4Step(StateManager& stateManager, int step) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then extend feed clamp
            if (cutMotor && !cutMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Step 1: Cut motor returned home. Extending feed clamp.");
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause returningNo2x4Step to increment to 2 after delay
            }
            break;
            
        case 2: // Was original returningNo2x4Step 1: wait for feed motor, then retract feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Step 2: Feed motor at home. Disengaging feed clamp."); 
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original returningNo2x4Step 2: move feed motor to 2.0 inches
            //serial.println("RETURNING_NO_2x4 Step 3: Moving feed motor to 2.0 inches."); 
            configureFeedMotorForNormalOperation(); // Ensure correct config
            moveFeedMotorToPosition(2.0);
            returningNo2x4Step = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original returningNo2x4Step 3: wait for feed motor at 2.0, extend feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Step 4: Feed motor at 2.0 inches. Extending feed clamp."); 
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 5
            }
            break;
            
        case 5: // Was original returningNo2x4Step 4: move feed motor to home
            //serial.println("RETURNING_NO_2x4 Step 5: Moving feed motor to home."); 
            configureFeedMotorForNormalOperation();
            moveFeedMotorToHome();
            returningNo2x4Step = 6; // Directly advance step
            break;
            
        case 6: // Was original returningNo2x4Step 5: wait for feed motor at home, retract feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Step 6: Feed motor at home. Disengaging feed clamp."); 
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 7
            }
            break;
            
        case 7: // Was original returningNo2x4Step 6: move feed motor to final position
            //serial.println("RETURNING_NO_2x4 Step 7: Moving feed motor to final position (FEED_TRAVEL_DISTANCE).");
            configureFeedMotorForNormalOperation();
            moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
            returningNo2x4Step = 8; // Directly advance step
            break;
            
        case 8: // Final step: check cut home and finish sequence
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Step 8: Feed motor at final position."); 
                bool sensorDetectedHome = false;
                //serial.println("RETURNING_NO_2x4 Step 8: Checking cut motor position switch."); 
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
                    //serial.println(")");
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        //serial.println("Cut motor position switch detected HIGH during RETURNING_NO_2x4 sequence completion.");
                        break;  
                    }
                }
                
                // TEMPORARY FIX: Always proceed regardless of sensor to identify the issue
                if (!sensorDetectedHome) {
                    //serial.println("DIAGNOSTIC: Cut motor home sensor did NOT detect HIGH.");
                    //serial.println("DIAGNOSTIC: Proceeding anyway to test if sensor logic is inverted.");
                    //serial.println("DIAGNOSTIC: If cut motor is physically at home, sensor logic may need inversion.");
                } else {
                    //serial.println("DIAGNOSTIC: Cut motor home sensor successfully detected HIGH.");
                }

                //serial.println("RETURNING_NO_2x4 Step 8: Completing sequence without homing.");
                
                // Complete sequence without homing
                retract2x4SecureClamp(); 
                //serial.println("2x4 secure clamp disengaged (final check in RETURNING_NO_2x4)."); 
                extend2x4SecureClamp(); 
                //serial.println("2x4 secure clamp engaged."); 
                turnYellowLedOff();
                turnBlueLedOn(); 

                resetSteps();
                stateManager.setCuttingCycleInProgress(false);
                stateManager.changeState(IDLE);
                
                // Check if cycle switch is currently ON - if yes, require cycling
                if (stateManager.getStartCycleSwitch()->read() == HIGH) {
                    stateManager.setStartSwitchSafe(false);
                    //serial.println("Cycle switch is still ON - must be cycled OFF then ON for next cycle.");
                } else {
                    //serial.println("Cycle switch is OFF - ready for next cycle.");
                }
                
                //serial.println("RETURNING_NO_2x4 sequence complete. Transitioning to IDLE state. Continuous mode OFF.");
            }
            break;
    }
}



void ReturningNo2x4State::resetSteps() {
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 