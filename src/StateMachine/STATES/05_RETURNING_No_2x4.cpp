#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

//! ************************************************************************
//! STEP 1: INITIALIZE SEQUENCE - MOVE CUT MOTOR HOME AND RETRACT 2X4 CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 2: WAIT FOR CUT MOTOR HOME AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 3: WAIT FOR FEED MOTOR HOME AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 4: MOVE FEED MOTOR TO 2.0 INCHES
//! ************************************************************************

//! ************************************************************************
//! STEP 5: WAIT FOR FEED MOTOR AT 2.0 AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 6: MOVE FEED MOTOR TO HOME
//! ************************************************************************

//! ************************************************************************
//! STEP 7: WAIT FOR FEED MOTOR HOME AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 8: MOVE FEED MOTOR TO FINAL POSITION
//! ************************************************************************

//! ************************************************************************
//! STEP 9: VERIFY CUT HOME POSITION AND COMPLETE SEQUENCE
//! ************************************************************************

// Static variables for returning no 2x4 state tracking
static int returningNo2x4Step = 0;
static unsigned long cylinderActionTime = 0;
static bool waitingForCylinder = false;

void executeReturningNo2x4State() {
    handleReturningNo2x4Sequence(); 
}

void onEnterReturningNo2x4State() {
    //! ************************************************************************
    //! STEP 1: INITIALIZE RETURNING NO 2X4 SEQUENCE
    //! ************************************************************************
    
    // Reset consecutive yeswood counter when nowood state occurs
    resetConsecutiveYeswoodCount();
    
    // Initialize RETURNING_NO_2x4 sequence from CUTTING_state logic
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

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

void handleReturningNo2x4Sequence() {
    // RETURNING_NO_2x4 sequence logic
    FastAccelStepper* feedMotor = getFeedMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (returningNo2x4Step == 0) { // First time entering this specific RETURNING_NO_2x4 logic path
        retract2x4SecureClamp();
        if (feedMotor) {
            if (feedMotor->getCurrentPosition() != 0 || feedMotor->isRunning()) {
                feedMotor->moveTo(0);
            }
        }
        returningNo2x4Step = 1;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        returningNo2x4Step++; 
    }
    
    if (!waitingForCylinder) {
        handleReturningNo2x4Step(returningNo2x4Step);
    }
}

void handleReturningNo2x4Step(int step) {
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then extend feed clamp
            if (cutMotor && !cutMotor->isRunning()) {
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause returningNo2x4Step to increment to 2 after delay
            }
            break;
            
        case 2: // Was original returningNo2x4Step 1: wait for feed motor, then retract feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original returningNo2x4Step 2: move feed motor to 0 inches
            configureFeedMotorForNormalOperation(); // Ensure correct config
            moveFeedMotorToPosition(0.0);
            returningNo2x4Step = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original returningNo2x4Step 3: wait for feed motor at 2.0, extend feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 5
            }
            break;
            
        case 5: // Was original returningNo2x4Step 4: move feed motor to home
            configureFeedMotorForNormalOperation();
            moveFeedMotorToHome();
            returningNo2x4Step = 6; // Directly advance step
            break;
            
        case 6: // Was original returningNo2x4Step 5: wait for feed motor at home, retract feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 7
            }
            break;
            
        case 7: // Was original returningNo2x4Step 6: move feed motor to final position
            configureFeedMotorForNormalOperation();
            moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
            returningNo2x4Step = 8; // Directly advance step
            break;
            
        case 8: // Final step: check cut home and finish sequence
            if (feedMotor && !feedMotor->isRunning()) {
                bool sensorDetectedHome = false;
                for (int i = 0; i < 3; i++) {
                    delay(30);  
                    getCutHomingSwitch()->update();
                    bool sensorReading = getCutHomingSwitch()->read();
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        break;  
                    }
                }
                
                // Complete sequence without homing
                retract2x4SecureClamp(); 
                extend2x4SecureClamp(); 
                turnYellowLedOff();
                turnBlueLedOn(); 

                resetReturningNo2x4Steps();
                setCuttingCycleInProgress(false);
                changeState(IDLE);
                
                // Check if cycle switch is currently ON - if yes, require cycling
                if (getStartCycleSwitch()->read() == HIGH) {
                    setStartSwitchSafe(false);
                } 
            }
            break;
    }
}

void resetReturningNo2x4Steps() {
    returningNo2x4Step = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 