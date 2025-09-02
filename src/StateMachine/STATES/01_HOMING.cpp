#include "StateMachine/01_HOMING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** HOMING STATE ********************************
//* ************************************************************************
// Handles the homing sequence for all motors.

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
static bool cutMotorHomed = false;
static bool feedMotorHomed = false;
static bool feedMotorMoved = false;
static bool feedHomingPhaseInitiated = false;
static unsigned long blinkTimer = 0;

void onEnterHomingState() {
    // Reset homing state variables when entering
    cutMotorHomed = false;
    feedMotorHomed = false;
    feedMotorMoved = false;
    feedHomingPhaseInitiated = false;
    blinkTimer = 0;
}

void executeHomingState() {
    // Blink blue LED to indicate homing in progress
    if (millis() - blinkTimer > 500) {
        bool blinkState = getBlinkState();
        blinkState = !blinkState;
        setBlinkState(blinkState);
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }

    // Debug output to track homing progress
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 2000) {
        Serial.print("HOMING STATE DEBUG - cutMotorHomed: ");
        Serial.print(cutMotorHomed);
        Serial.print(", feedMotorHomed: ");
        Serial.print(feedMotorHomed);
        Serial.print(", feedMotorMoved: ");
        Serial.print(feedMotorMoved);
        Serial.print(", feedHomingPhaseInitiated: ");
        //serial.println(feedHomingPhaseInitiated);
        lastDebugTime = millis();
    }

    if (!cutMotorHomed) {
        //serial.println("Starting cut motor homing phase (blocking)...");
        extern const unsigned long CUT_HOME_TIMEOUT; // This is in main.cpp
        homeCutMotorBlocking(*getCutHomingSwitch(), CUT_HOME_TIMEOUT);
        if (getCutMotor() && getCutMotor()->getCurrentPosition() == 0) { // Check if homing was successful
            cutMotorHomed = true;
            //serial.println("Cut motor homing marked as successful.");
        } else {
            //serial.println("Cut motor homing failed or timed out. Retrying or error.");
        }
    } else if (!feedMotorHomed) {
        if (!feedHomingPhaseInitiated) {
            //serial.println("Starting feed motor homing phase (blocking)..."); 
            retractFeedClamp(); 
            //serial.println("Feed clamp retracted for homing."); 
            feedHomingPhaseInitiated = true;
        }
        //serial.println("Calling homeFeedMotorBlocking...");
        homeFeedMotorBlocking(*getFeedHomingSwitch());
        feedMotorHomed = true; 
        feedHomingPhaseInitiated = false; // Reset for next potential homing cycle
        //serial.println("Feed motor homing marked as successful.");
    } else if (!feedMotorMoved) {
        //serial.println("Moving feed motor to travel distance...");
        extendFeedClamp();
        //serial.println("Feed clamp re-extended.");
        moveFeedMotorToTravel();
        while(getFeedMotor()->isRunning()){
            // Wait for feed motor to reach FEED_TRAVEL_DISTANCE
        }
        feedMotorMoved = true;
        //serial.println("Feed motor moved to FEED_TRAVEL_DISTANCE (blocking complete).");
    } else {
        //serial.println("All homing steps complete! Transitioning to IDLE..."); 
        cutMotorHomed = false; 
        feedMotorHomed = false;
        feedMotorMoved = false;
        
        extern bool isHomed; // This is in main.cpp
        isHomed = true; 
        //serial.println("isHomed flag set to true.");

        turnBlueLedOff();
        turnGreenLedOn();
        //serial.println("LEDs updated: Blue OFF, Green ON.");

        // SAFETY CHANGE: Do NOT automatically home the rotation servo on startup
        // This prevents ramming stuck wood pieces into the blade during emergency restart
        // The servo will only be homed when manually starting a cut cycle
        //serial.println("Servo homing skipped on startup for safety - will home when cut cycle starts.");
        
        //serial.println("Changing state to IDLE...");
        changeState(IDLE);
        //serial.println("State change to IDLE completed.");
    }
}

void onExitHomingState() {
    // No specific cleanup needed for HOMING state
} 