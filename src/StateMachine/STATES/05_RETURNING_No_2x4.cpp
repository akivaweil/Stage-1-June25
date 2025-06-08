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
    returningNo2x4HomingSubStep = 0;
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
            
        case 8: // Was original returningNo2x4Step 7: wait for motor, check cut home, start feed motor homing
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

                //serial.println("RETURNING_NO_2x4 Step 8: Starting feed motor homing sequence...");
                
                //! ************************************************************************
                //! START FEED MOTOR HOMING SEQUENCE FOR RETURNING_NO_2x4
                //! ************************************************************************
                retractFeedClamp();
                //serial.println("Feed clamp retracted. Starting RETURNING_NO_2x4 feed motor homing sequence...");
                
                // Transition to new RETURNING_NO_2x4 homing substeps
                returningNo2x4Step = 9;
                returningNo2x4HomingSubStep = 0; // Initialize homing substep
                //serial.println("Transitioning to RETURNING_NO_2x4 feed motor homing sequence (Step 9)."); 
            }
            break;
            
        case 9: // RETURNING_NO_2x4 Feed Motor Homing Sequence
            handleReturningNo2x4FeedMotorHoming(stateManager);
            break;
    }
}

void ReturningNo2x4State::handleReturningNo2x4FeedMotorHoming(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    // FEED_MOTOR_STEPS_PER_INCH is already declared in General_Functions.h
    
    // Non-blocking feed motor homing sequence for RETURNING_NO_2x4
    switch (returningNo2x4HomingSubStep) {
        case 0: // Start homing - move toward home switch
            //serial.println("RETURNING_NO_2x4 Feed Motor Homing Step 9.0: Moving toward home switch.");
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            returningNo2x4HomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getFeedHomingSwitch()->update();
            if (stateManager.getFeedHomingSwitch()->read() == HIGH) {
                //serial.println("RETURNING_NO_2x4 Feed Motor Homing Step 9.1: Home switch triggered. Stopping motor.");
                if (feedMotor) {
                    feedMotor->forceStop();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                //serial.println("RETURNING_NO_2x4: Feed motor hit home switch.");
                returningNo2x4HomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.1 inch from switch
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Feed Motor Homing Step 9.2: Moving to -0.1 inch from home switch to establish working zero.");
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                returningNo2x4HomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("RETURNING_NO_2x4 Feed Motor Homing Step 9.3: Setting new working zero position.");
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                //serial.println("RETURNING_NO_2x4: Feed motor homed: 0.1 inch from switch set as position 0.");
                
                configureFeedMotorForNormalOperation();
                returningNo2x4HomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - finish RETURNING_NO_2x4 sequence
            //serial.println("RETURNING_NO_2x4 Feed Motor Homing Step 9.4: Homing sequence complete.");
            
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
            
            //serial.println("RETURNING_NO_2x4 sequence with feed motor homing complete. Transitioning to IDLE state. Continuous mode OFF.");
            break;
    }
}

void ReturningNo2x4State::resetSteps() {
    returningNo2x4Step = 0;
    returningNo2x4HomingSubStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 