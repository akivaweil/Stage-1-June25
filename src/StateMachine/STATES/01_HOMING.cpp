#include "StateMachine/01_HOMING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** HOMING STATE ********************************
//* ************************************************************************
// Handles the homing sequence for all motors.
// Step 1: Blink blue LED to indicate homing in progress.
// Step 2: Home the cut motor (blocking). If fails, it might retry or transition to ERROR (currently retries).
// Step 3: If cut motor homed, home the feed motor (blocking). Retract feed clamp before homing.
// Step 4: If feed motor homed, move feed motor to FEED_TRAVEL_DISTANCE (blocking). Re-extend feed clamp.
// Step 5: If all homing and initial positioning are complete, set isHomed flag to true.
// Step 6: Turn off blue LED, turn on green LED.
// Step 7: Ensure servo is at 2 degrees.
// Step 8: Transition to IDLE state.

void HomingState::onEnter(StateManager& stateManager) {
    // Reset homing state variables when entering
    cutMotorHomed = false;
    feedMotorHomed = false;
    feedMotorMoved = false;
    feedHomingPhaseInitiated = false;
    blinkTimer = 0;
}

void HomingState::execute(StateManager& stateManager) {
    // Blink blue LED to indicate homing in progress
    if (millis() - blinkTimer > 500) {
        bool blinkState = stateManager.getBlinkState();
        blinkState = !blinkState;
        stateManager.setBlinkState(blinkState);
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
        homeCutMotorBlocking(*stateManager.getCutHomingSwitch(), CUT_HOME_TIMEOUT);
        if (stateManager.getCutMotor() && stateManager.getCutMotor()->getCurrentPosition() == 0) { // Check if homing was successful
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
        homeFeedMotorBlocking(*stateManager.getFeedHomingSwitch());
        feedMotorHomed = true; 
        feedHomingPhaseInitiated = false; // Reset for next potential homing cycle
        //serial.println("Feed motor homing marked as successful.");
    } else if (!feedMotorMoved) {
        //serial.println("Moving feed motor to travel distance...");
        extendFeedClamp();
        //serial.println("Feed clamp re-extended.");
        moveFeedMotorToTravel();
        while(stateManager.getFeedMotor()->isRunning()){
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

        // Set initial servo position via function call
        handleRotationServoReturn();
        //serial.println("Servo returned to home position.");
        
        //serial.println("Changing state to IDLE...");
        stateManager.changeState(IDLE);
        //serial.println("State change to IDLE completed.");
    }
} 