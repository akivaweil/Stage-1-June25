#include "StateMachine/01_HOMING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** HOMING STATE ********************************
//* ************************************************************************
// Handles the homing sequence for all motors.

// Homing phase enumeration
enum HomingPhase {
    CUT_MOTOR_HOMING,
    FEED_MOTOR_HOMING,
    FEED_MOTOR_POSITIONING
};

//! ************************************************************************
//! STEP 1: BLINK BLUE LED TO INDICATE HOMING IN PROGRESS
//! ************************************************************************

//! ************************************************************************
//! STEP 2: HOME THE CUT MOTOR (BLOCKING) - RETRY ON FAILURE
//! ************************************************************************

//! ************************************************************************
//! STEP 3: HOME THE FEED MOTOR (BLOCKING) - RETRACT FEED CLAMP FIRST
//! ************************************************************************

//! ************************************************************************
//! STEP 4: MOVE FEED MOTOR TO FEED_TRAVEL_DISTANCE - RE-EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 5: SET ISHOMED FLAG TO TRUE WHEN ALL HOMING COMPLETE
//! ************************************************************************

//! ************************************************************************
//! STEP 6: TURN OFF BLUE LED, TURN ON GREEN LED
//! ************************************************************************

//! ************************************************************************
//! STEP 7: ENSURE SERVO IS AT 2 DEGREES
//! ************************************************************************

//! ************************************************************************
//! STEP 8: TRANSITION TO IDLE STATE
//! ************************************************************************

// Static variables for homing state tracking
static HomingPhase currentHomingPhase = CUT_MOTOR_HOMING;
static bool cutMotorHomed = false;
static bool feedMotorHomed = false;
static bool feedMotorMoved = false;
static bool feedHomingPhaseInitiated = false;
static bool isHomed = false;
static unsigned long blinkTimer = 0;

// Forward declarations for helper functions
void executeCutMotorHoming();
void executeFeedMotorHoming();
void executeFeedMotorPositioning();

void onEnterHomingState() {
    // Reset homing flags
    cutMotorHomed = false;
    feedMotorHomed = false;
    feedMotorMoved = false;
    feedHomingPhaseInitiated = false;
    
    // Start with cut motor homing
    currentHomingPhase = CUT_MOTOR_HOMING;
    
    // Turn on blue LED during homing
    turnBlueLedOn();
}

void executeHomingState() {
    // Check if both motors are homed
    if (cutMotorHomed && feedMotorHomed && feedMotorMoved) {
        // All homing complete, transition to IDLE
        changeState(IDLE);
        return;
    }
    
    // Execute homing steps based on current phase
    switch (currentHomingPhase) {
        case CUT_MOTOR_HOMING:
            executeCutMotorHoming();
            break;
            
        case FEED_MOTOR_HOMING:
            executeFeedMotorHoming();
            break;
            
        case FEED_MOTOR_POSITIONING:
            executeFeedMotorPositioning();
            break;
    }
}

void onExitHomingState() {
    // Homing complete
    isHomed = true;
    
    // Turn off blue LED, turn on green LED
    turnBlueLedOff();
    turnGreenLedOn();
    
    // Return servo to home position
    activateRotationServo();
}

void executeCutMotorHoming() {
    if (!cutMotorHomed) {
        // Home cut motor
        homeCutMotorBlocking(cutHomingSwitch, 30000);
        cutMotorHomed = true;
        currentHomingPhase = FEED_MOTOR_HOMING;
    }
}

void executeFeedMotorHoming() {
    if (!feedMotorHomed && !feedHomingPhaseInitiated) {
        // Retract feed clamp for homing
        retractFeedClamp();
        feedHomingPhaseInitiated = true;
        
        // Start feed motor homing
        homeFeedMotorBlocking(feedHomingSwitch);
        feedMotorHomed = true;
        currentHomingPhase = FEED_MOTOR_POSITIONING;
    }
}

void executeFeedMotorPositioning() {
    if (!feedMotorMoved) {
        // Move feed motor to travel distance
        FastAccelStepper* feedMotor = getFeedMotor();
        if (feedMotor) {
            feedMotor->moveTo((long)(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH));
            feedMotorMoved = true;
        }
        
        // Re-extend feed clamp
        extendFeedClamp();
    }
} 