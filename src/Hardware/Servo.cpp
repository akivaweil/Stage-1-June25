#include <Arduino.h>
#include <ESP32Servo.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ ROTATION SERVO                                                    ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// The rotation servo swings the suction assembly between HOME and ACTIVE
// positions. There is no position feedback from the servo, so the caller
// has to time the movement.

void activateRotationServo() {
    // Activate rotation servo without sending TA signal
    if (!rotationServoIsActiveAndTiming) {
        Servo* servo = getRotationServo();
        if (servo) {
            // Force servo write with robust control - no attach checks, just send the command
            servo->write(ROTATION_SERVO_ACTIVE_POSITION);
        }

        rotationServoActiveStartTime = millis();
        rotationServoIsActiveAndTiming = true;
        // Reset the return completed flag for new activation cycle
        rotationServoReturnCompleted = false;
    }
}

void handleRotationServoReturn() {
    // Move rotation servo to home position with fast return - send command multiple times rapidly
    Servo* servo = getRotationServo();
    if (servo) {
        // Send multiple rapid commands to ensure fast return to home
        for (int i = 0; i < 5; i++) {
            servo->write(ROTATION_SERVO_HOME_POSITION);
            delayMicroseconds(100); // Small delay between rapid writes
        }
    }
}
