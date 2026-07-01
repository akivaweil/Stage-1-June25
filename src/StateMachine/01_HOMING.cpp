#include "StateMachine/01_HOMING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"

// HOMING STATE
// Handles the homing sequence for all motors.
// CUT_HOME_TIMEOUT now lives in Config.cpp along with the rest of the timings.

// STEP 1: BLINK BLUE LED TO INDICATE HOMING IN PROGRESS

// STEP 2: HOME THE CUT MOTOR (BLOCKING) - RETRY ON FAILURE

// STEP 3: HOME THE FEED MOTOR (BLOCKING) - RETRACT FEED CLAMP FIRST

// STEP 4: RE-EXTEND FEED CLAMP (FEED MOTOR STAYS AT HOME POSITION)

// STEP 5: SET ISHOMED FLAG TO TRUE WHEN ALL HOMING COMPLETE

// STEP 6: TURN OFF BLUE LED, TURN ON GREEN LED

// STEP 7: ENSURE SERVO IS AT 2 DEGREES

// STEP 8: TRANSITION TO IDLE STATE

// Blink interval for the blue "homing in progress" LED indicator.
static const unsigned long HOMING_LED_BLINK_INTERVAL_MS = 500;

// Static variables for homing state tracking
static bool cutMotorHomed = false;
static bool feedMotorHomed = false;
static bool feedHomingPhaseInitiated = false;
static unsigned long blinkTimer = 0;

void onEnterHomingState() {
    // Reset homing state variables when entering
    cutMotorHomed = false;
    feedMotorHomed = false;
    feedHomingPhaseInitiated = false;
    blinkTimer = 0;
}

void handleHomingState() {
    // Blink blue LED to indicate homing in progress
    if (millis() - blinkTimer > HOMING_LED_BLINK_INTERVAL_MS) {
        bool blinkState = getBlinkState();
        blinkState = !blinkState;
        setBlinkState(blinkState);
        if (blinkState) showBlueLed(); else turnBlueLedOff();
        blinkTimer = millis();
    }

    if (!cutMotorHomed) {
        homeCutMotorBlocking(*getCutHomingSwitch(), CUT_HOME_TIMEOUT);
        if (getCutMotor() && getCutMotor()->getCurrentPosition() == 0) { // Check if homing was successful
            cutMotorHomed = true;
        }
    } else if (!feedMotorHomed) {
        if (!feedHomingPhaseInitiated) {
            retractFeedClamp();
            feedHomingPhaseInitiated = true;
        }
        if (homeFeedMotorNonBlocking(*getFeedHomingSwitch())) {
            extendFeedClamp();
            feedMotorHomed = true;
            feedHomingPhaseInitiated = false; // Reset for next potential homing cycle
        }
    } else {
        cutMotorHomed = false;
        feedMotorHomed = false;

        extern bool isHomed; // This is in main.cpp
        isHomed = true;

        turnBlueLedOff();
        showGreenLed();

        // SAFETY CHANGE: Do NOT automatically home the rotation servo on startup
        // This prevents ramming stuck wood pieces into the blade during emergency restart
        // The servo will only be homed when manually starting a cut cycle

        changeState(STATE_IDLE);
    }
}

void onExitHomingState() {
    // No specific cleanup needed for HOMING state
} 