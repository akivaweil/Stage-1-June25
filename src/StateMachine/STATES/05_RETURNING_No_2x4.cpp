#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ CONFIGURATION CONSTANTS *********************
//* ************************************************************************

// Movement Configuration
const float FEED_MOTOR_RETURN_DISTANCE = 4.5;             // inches - Distance to move feed motor away from home
const float FEED_MOTOR_HOME_POSITION = 0.0;                // inches - home position

//* ************************************************************************
//* ************************ STATE VARIABLES ******************************
//* ************************************************************************

// Main sequence tracking
static int currentStep = 0;

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// Simplified 3-step sequence for handling material that doesn't trigger the wood sensor.

//! ************************************************************************
//! STEP 1: RETURN CUT MOTOR HOME
//! ************************************************************************

//! ************************************************************************
//! STEP 2: MOVE FEED MOTOR 4.5 INCHES AWAY FROM HOME WITH CLAMPS SET
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
    
    // Initialize step tracking
    currentStep = 0;
}

//* ************************************************************************
//* ************************ MAIN SEQUENCE HANDLER ************************
//* ************************************************************************

void handleReturningNo2x4Sequence() {
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    
    switch (currentStep) {
        case 0: // Step 1: Return cut motor home
            handleCutMotorReturnHome();
            break;
            
        case 1: // Step 2: Move feed motor 4.5 inches with clamps set
            handleFeedMotorMoveWithClamps();
            break;
            
        case 2: // Step 3: Retract both clamps and switch to idle
            handleRetractClampsAndComplete();
            break;
    }
}

//* ************************************************************************
//* ************************ STEP HANDLERS ********************************
//* ************************************************************************

void handleCutMotorReturnHome() {
    FastAccelStepper* cutMotor = getCutMotor();
    
    if (currentStep == 0) {
        // Start cut motor homing and retract secure clamp
        retract2x4SecureClamp();
        moveCutMotorToHome();
        currentStep = 1; // Move to next step
    }
}

void handleFeedMotorMoveWithClamps() {
    FastAccelStepper* cutMotor = getCutMotor();
    FastAccelStepper* feedMotor = getFeedMotor();
    
    // Wait for cut motor to finish homing
    if (cutMotor && !cutMotor->isRunning()) {
        // Set up clamps: extend feed clamp, retract secure clamp
        extendFeedClamp();
        retract2x4SecureClamp();
        
        // Move feed motor 4.5 inches away from home
        moveFeedMotorToPosition(FEED_MOTOR_RETURN_DISTANCE);
        currentStep = 2; // Move to final step
    }
}

void handleRetractClampsAndComplete() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    // Wait for feed motor to finish moving
    if (feedMotor && !feedMotor->isRunning()) {
        // Retract both clamps
        retractFeedClamp();
        retract2x4SecureClamp();
        
        // Complete sequence and switch to idle
        completeReturningNo2x4Sequence();
    }
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
    currentStep = 0;
}