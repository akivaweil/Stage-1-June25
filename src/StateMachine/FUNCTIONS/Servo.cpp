#include <Arduino.h>
#include <ESP32Servo.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

// ROTATION SERVO
// The rotation servo swings the suction assembly between HOME and ACTIVE
// positions. There is no position feedback from the servo, so the caller
// has to time the movement.

void activateRotationServo() {
    // Activate rotation servo without sending TA signal
    if (!rotationServoActive) {
        Servo* servo = getRotationServo();
        if (servo) {
            // Force servo write with robust control - no attach checks, just send the command
            servo->write(ROTATION_SERVO_HOME_POSITION + ROTATION_SERVO_ACTIVE_OFFSET);
        }

        rotationServoActiveStartTime = millis();
        rotationServoActive = true;
        rotationServoKnownHome = false;      // servo is moving to ACTIVE — no longer at home
        rotationServoHomePending = false;    // cancel any in-flight home buffer
        // Reset the return completed flag for new activation cycle
        rotationServoReturnCompleted = false;
    }
}

void returnRotationServoHome() {
    // Move rotation servo to home position with fast return - send command multiple times rapidly
    Servo* servo = getRotationServo();
    if (servo) {
        // Send multiple rapid commands to ensure fast return to home
        for (int i = 0; i < 5; i++) {
            servo->write(ROTATION_SERVO_HOME_POSITION);
            delayMicroseconds(100); // Small delay between rapid writes
        }
    }
    // Start the travel buffer — rotationServoKnownHome will flip true once
    // ROTATION_SERVO_HOME_WAIT_DURATION_MS has elapsed, via
    // updateRotationServoHomeStatus() on the main loop tick.
    rotationServoHomePending = true;
    rotationServoHomeCommandTime = millis();
    rotationServoKnownHome = false; // becomes true only after the buffer
}

void updateRotationServoHomeStatus() {
    if (rotationServoHomePending &&
        millis() - rotationServoHomeCommandTime >= ROTATION_SERVO_HOME_WAIT_DURATION_MS) {
        rotationServoKnownHome = true;
        rotationServoHomePending = false;
    }
}
