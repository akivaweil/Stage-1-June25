#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ CONFIGURATION CONSTANTS *********************
//* ************************************************************************

// Movement Configuration
const float FEED_MOTOR_RETURN_DISTANCE = 3.0;             // inches - Distance to move feed motor away from home (within safe limits)
const float FEED_MOTOR_HOME_POSITION = 0.0;                // inches - home position

//* ************************************************************************
//* ************************ STATE VARIABLES ******************************
//* ************************************************************************
// No state variables needed - everything happens sequentially in one function

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// Simplified 3-step sequence for handling material that doesn't trigger the wood sensor.

//! ************************************************************************
//! STEP 1: RETURN CUT MOTOR HOME
//! ************************************************************************

//! ************************************************************************
//! STEP 2: MOVE FEED MOTOR 3.0 INCHES AWAY FROM HOME WITH CLAMPS SET
//! ************************************************************************

//! ************************************************************************
//! STEP 3: RETRACT BOTH CLAMPS AND SWITCH TO IDLE
//! ************************************************************************

//* ************************************************************************
//* ************************ PUBLIC INTERFACE *****************************
//* ************************************************************************

void executeReturningNo2x4State() {
    handleReturningNo2x4Sequence(); 
}

void onEnterReturningNo2x4State() {
    initializeReturningNo2x4Sequence();
}

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

//* ************************************************************************
//* ************************ INITIALIZATION *******************************
//* ************************************************************************

void initializeReturningNo2x4Sequence() {
    //! ************************************************************************
    //! STEP 1: INITIALIZE RETURNING NO 2X4 SEQUENCE
    //! ************************************************************************
    
    // Reset consecutive yeswood counter when nowood state occurs
    resetConsecutiveYeswoodCount();
    
    // Configure motors for return operation
    configureCutMotorForReturn();
    configureFeedMotorForNormalOperation();

    // Set up LED indicators
    turnBlueLedOn();
    turnYellowLedOff();
}

//* ************************************************************************
//* ************************ MAIN SEQUENCE HANDLER ************************
//* ************************************************************************

void handleReturningNo2x4Sequence() {
    //! ************************************************************************
    //! STEP 1: RETURN CUT MOTOR HOME AND RETRACT SECURE CLAMP
    //! ************************************************************************
    retract2x4SecureClamp();
    moveCutMotorToHome();
    
    // Wait for cut motor to finish homing
    FastAccelStepper* cutMotor = getCutMotor();
    while (cutMotor && cutMotor->isRunning()) {
        // Wait for cut motor to complete
    }
    
    //! ************************************************************************
    //! STEP 2: MOVE FEED MOTOR 3.0 INCHES WITH CLAMPS SET
    //! ************************************************************************
    // Set up clamps: extend feed clamp, retract secure clamp
    extendFeedClamp();
    retract2x4SecureClamp();
    
    // Move feed motor 3.0 inches away from home
    moveFeedMotorToPosition(FEED_MOTOR_RETURN_DISTANCE);
    
    // Wait for feed motor to finish moving
    FastAccelStepper* feedMotor = getFeedMotor();
    while (feedMotor && feedMotor->isRunning()) {
        // Wait for feed motor to complete
    }
    
    //! ************************************************************************
    //! STEP 3: RETRACT BOTH CLAMPS AND SWITCH TO IDLE
    //! ************************************************************************
    // Retract both clamps
    retractFeedClamp();
    retract2x4SecureClamp();
    
    // Complete sequence and switch to idle
    completeReturningNo2x4Sequence();
}




//* ************************************************************************
//* ************************ SEQUENCE COMPLETION **************************
//* ************************************************************************

void completeReturningNo2x4Sequence() {
    // Complete sequence and transition to idle
    turnYellowLedOff();
    turnBlueLedOn(); 

    resetReturningNo2x4Steps();
    setCuttingCycleInProgress(false);
    
    // Check if cycle switch is currently ON - if yes, require cycling
    if (getStartCycleSwitch()->read() == HIGH) {
        setStartSwitchSafe(false);
    }
    
    // Set flag to indicate we're coming from no-wood cycle
    setComingFromNoWoodWithSensorsClear(true);
    
    // Transition to IDLE state
    changeState(IDLE);
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetReturningNo2x4Steps() {
    // No state variables to reset - everything happens in one function
}