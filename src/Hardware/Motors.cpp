#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

// External motor object references from main.cpp
extern FastAccelStepper* cutMotor;
extern FastAccelStepper* feedMotor;
extern Bounce cutHomingSwitch;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 MOTOR CONTROL                                                     ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// All motor configuration, movement, and homing lives here. The cut motor
// (FastAccelStepper) and feed motor are driven directly; the rotation servo
// is in Servo.cpp.

const unsigned long FEED_HOME_TIMEOUT = 30000; // 30 seconds timeout

void configureCutMotorForCutting() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_NORMAL_SPEED * CUT_MOTOR_STEPS_PER_INCH));
        cutMotor->setAcceleration((uint32_t)(CUT_MOTOR_NORMAL_ACCELERATION * CUT_MOTOR_STEPS_PER_INCH));
    }
}

void configureCutMotorForCuttingSlow() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_NO_WOOD_SPEED * CUT_MOTOR_STEPS_PER_INCH));
        cutMotor->setAcceleration((uint32_t)(CUT_MOTOR_NORMAL_ACCELERATION * CUT_MOTOR_STEPS_PER_INCH));
    }
}

void configureCutMotorForReturn() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_RETURN_SPEED * CUT_MOTOR_STEPS_PER_INCH));
        cutMotor->setAcceleration((uint32_t)(CUT_MOTOR_NORMAL_ACCELERATION * CUT_MOTOR_STEPS_PER_INCH));
    }
}

void configureFeedMotorForNormalOperation() {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_NORMAL_SPEED);
        feedMotor->setAcceleration((uint32_t)FEED_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureFeedMotorForReturn() {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_RETURN_SPEED);
        feedMotor->setAcceleration((uint32_t)FEED_MOTOR_RETURN_ACCELERATION);
    }
}

void configureFeedMotorForSlowOperation(float speedMultiplier) {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)(FEED_MOTOR_NORMAL_SPEED * speedMultiplier));
        feedMotor->setAcceleration((uint32_t)(FEED_MOTOR_NORMAL_ACCELERATION * speedMultiplier));
    }
}

void moveCutMotorToCut() {
    if (cutMotor) {
        extern float getCutTravelDistance();
        cutMotor->moveTo(getCutTravelDistance() * CUT_MOTOR_STEPS_PER_INCH);
    }
}

void moveCutMotorToHome() {
    if (cutMotor) {
        cutMotor->moveTo(-0.02 * CUT_MOTOR_STEPS_PER_INCH); // Minimal overshoot
    }
}

void moveFeedMotorToTravel() {
    if (feedMotor) {
        feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    }
}

// NOTE: feed-motor "home" (the physical home sensor / post-homing rest pose)
// is at coordinate FEED_TRAVEL_DISTANCE — NOT at position 0. Position 0 is
// the fully pulled-back / load end, opposite the home sensor. The helpers
// below move to position 0, which is NOT home.
void moveFeedMotorToZero() {
    if (feedMotor) {
        feedMotor->moveTo(0);
    }
}

void moveFeedMotorToPosition(float targetPositionInches) {
    if (feedMotor) {
        feedMotor->moveTo(targetPositionInches * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void moveFeedMotorToPostCutZero() {
    if (feedMotor) {
        feedMotor->moveTo(0);
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->stopMove();
    }
}

void stopFeedMotor() {
    if (feedMotor) {
        feedMotor->stopMove();
    }
}

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🏠 HOMING                                                            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

// Basic blocking homing function for Cut Motor - can be expanded
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) {
        return;
    }

    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_HOMING_SPEED * CUT_MOTOR_STEPS_PER_INCH));
    cutMotor->moveTo(-40000);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();

        if (millis() - startTime > timeout) {
            cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            return;
        }
    }

    // Use forceStopAndNewPosition for immediate stopping and set position to 0
    cutMotor->forceStopAndNewPosition(0);

    // Add a small delay to ensure motor has fully stopped
    delay(50);

    // Verify the switch is still pressed after stopping
    homingSwitch.update();
}

// Basic blocking homing function for Feed Motor - can be expanded
void homeFeedMotorBlocking(Bounce& homingSwitch) {
    if (!feedMotor) {
        return;
    }

    // Step 1: Move toward home sensor until it triggers
    feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);

    // Try using runForward() instead of moveTo() for more reliable operation
    feedMotor->runForward();

    // Verify motor started
    delay(100); // Small delay to let motor start

    // Add timeout for feed motor homing
    unsigned long startTime = millis();

    while (homingSwitch.read() != LOW) {
        homingSwitch.update();

        // If motor stopped running unexpectedly, restart it
        static unsigned long lastRestartCheck = 0;
        if (millis() - lastRestartCheck >= 1000) {
            if (!feedMotor->isRunning()) {
                feedMotor->runForward();
            }
            lastRestartCheck = millis();
        }

        // Check for timeout
        if (millis() - startTime > FEED_HOME_TIMEOUT) {
            feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
            return;
        }
    }

    feedMotor->forceStopAndNewPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);

    // Step 2: Move to working position (offset from sensor)
    feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - FEED_MOTOR_OFFSET_FROM_SENSOR * FEED_MOTOR_STEPS_PER_INCH);

    // Wait for move to complete with timeout
    unsigned long moveStartTime = millis();
    while (feedMotor->isRunning()) {
        if (millis() - moveStartTime > 10000) { // 10 second timeout for positioning
            feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
            break;
        }
    }

    // Step 3: Set this position as the new zero
    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);

    configureFeedMotorForNormalOperation();
}

// Non-blocking feed motor homing state variables
static bool feedMotorHomingInProgress = false;
static int feedMotorHomingStep = 0;
static unsigned long feedMotorHomingStartTime = 0;
static unsigned long feedMotorHomingLastRestartCheck = 0;

// Non-blocking feed motor homing function
bool homeFeedMotorNonBlocking(Bounce& homingSwitch) {
    if (!feedMotor) {
        return false;
    }

    // Initialize homing if not already in progress
    if (!feedMotorHomingInProgress) {
        feedMotorHomingInProgress = true;
        feedMotorHomingStep = 0;
        feedMotorHomingStartTime = millis();
        feedMotorHomingLastRestartCheck = millis();

        // Step 1: Start moving toward home sensor
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
        feedMotor->runForward();
        feedMotorHomingStep = 1;
        return false; // Not complete yet
    }

    // Check for timeout
    if (millis() - feedMotorHomingStartTime > FEED_HOME_TIMEOUT) {
        feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
        feedMotorHomingInProgress = false;
        return false; // Failed due to timeout
    }

    // Step 1: Wait for home sensor to trigger
    if (feedMotorHomingStep == 1) {
        homingSwitch.update();

        // If motor stopped running unexpectedly, restart it
        if (millis() - feedMotorHomingLastRestartCheck >= 1000) {
            if (!feedMotor->isRunning()) {
                feedMotor->runForward();
            }
            feedMotorHomingLastRestartCheck = millis();
        }

        if (homingSwitch.read() == LOW) {
            // Home sensor detected - stop motor and move to step 2
            feedMotor->forceStopAndNewPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            feedMotorHomingStep = 2;
            return false; // Not complete yet
        }
    }

    // Step 2: Move to working position
    if (feedMotorHomingStep == 2) {
        if (!feedMotor->isRunning()) {
            // Move to working position
            feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - FEED_MOTOR_OFFSET_FROM_SENSOR * FEED_MOTOR_STEPS_PER_INCH);
            feedMotorHomingStep = 3;
        }
        return false; // Not complete yet
    }

    // Step 3: Wait for positioning to complete
    if (feedMotorHomingStep == 3) {
        if (!feedMotor->isRunning()) {
            // Set working position as zero
            feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            configureFeedMotorForNormalOperation();
            feedMotorHomingInProgress = false;
            return true; // Complete!
        }

        // Check for positioning timeout
        if (millis() - feedMotorHomingStartTime > FEED_HOME_TIMEOUT) {
            feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
            feedMotorHomingInProgress = false;
            return false; // Failed due to timeout
        }
    }

    return false; // Still in progress
}

void moveFeedMotorToInitialAfterHoming() {
    if (feedMotor) {
        configureFeedMotorForNormalOperation();
        moveFeedMotorToZero();
    }
}

// Checks the cut motor homing switch multiple times and recalibrates if detected.
// Returns true if home detected and recalibrated, false otherwise.
bool checkAndRecalibrateCutMotorHome(int attempts) {
    if (!cutMotor) return false;

    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); Serial.print(i + 1); Serial.print(": ");
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0);
            break;
        }
    }
    return sensorDetectedHome;
}
