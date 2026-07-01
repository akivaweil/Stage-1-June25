#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include <esp_task_wdt.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

// External motor object references from main.cpp
extern FastAccelStepper* cutMotor;
extern FastAccelStepper* feedMotor;
extern Bounce cutHomingSwitch;

// MOTOR CONTROL
// All motor configuration, movement, and homing lives here. The cut motor
// (FastAccelStepper) and feed motor are driven directly; the rotation servo
// is in Servo.cpp.

const unsigned long FEED_HOME_TIMEOUT = 30000; // 30 seconds timeout

// Minimal negative overshoot past home so the cut motor firmly seats the home switch.
const float CUT_MOTOR_HOME_OVERSHOOT_INCHES = -0.02;
// Large negative seek target for blocking cut homing: drives the motor toward the
// home switch; the switch (not this target) actually stops the move.
const long CUT_MOTOR_HOME_SEEK_TARGET_STEPS = -40000;
// Settle delay after the cut motor force-stops at home, before re-reading the switch.
const unsigned long CUT_MOTOR_HOME_SETTLE_DELAY_MS = 50;

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
        cutMotor->moveTo(CUT_MOTOR_HOME_OVERSHOOT_INCHES * CUT_MOTOR_STEPS_PER_INCH); // Minimal overshoot
    }
}

// NOTE: feed-motor "home" (the physical home sensor / post-homing rest pose)
// is at coordinate FEED_TRAVEL_DISTANCE — NOT at position 0. Position 0 is
// the fully pulled-back / load end, opposite the home sensor. The helper
// below moves to position 0, which is NOT home.
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

// HOMING

// Basic blocking homing function for Cut Motor - can be expanded
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) {
        return;
    }

    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_HOMING_SPEED * CUT_MOTOR_STEPS_PER_INCH));
    cutMotor->moveTo(CUT_MOTOR_HOME_SEEK_TARGET_STEPS);

    while (homingSwitch.read() != HIGH) {
        esp_task_wdt_reset();  // blocking wait — keep the task watchdog fed
        homingSwitch.update();

        if (millis() - startTime > timeout) {
            cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            return;
        }
    }

    // Use forceStopAndNewPosition for immediate stopping and set position to 0
    cutMotor->forceStopAndNewPosition(0);

    // Add a small delay to ensure motor has fully stopped
    delay(CUT_MOTOR_HOME_SETTLE_DELAY_MS);

    // Verify the switch is still pressed after stopping
    homingSwitch.update();
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
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0);
            break;
        }
    }
    return sensorDetectedHome;
}
