#include <Arduino.h>
#include "StateMachine/General_Functions.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 CLAMP CONTROL                                                     ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Feed/2x4-secure clamps: LOW = extended, HIGH = retracted (inverted logic).
// Rotation clamp: HIGH = extended, LOW = retracted.

void extendFeedClamp() {
    digitalWrite(FEED_CLAMP, LOW); // Extended
}

void retractFeedClamp() {
    digitalWrite(FEED_CLAMP, HIGH); // Retracted
}

void extend2x4SecureClamp() {
    digitalWrite(_2x4_SECURE_CLAMP, LOW); // Extended
}

void retract2x4SecureClamp() {
    digitalWrite(_2x4_SECURE_CLAMP, HIGH); // Retracted
}

void extendRotationClamp() {
    digitalWrite(ROTATION_CLAMP, HIGH); // Extended
    rotationClampExtendTime = millis();
    rotationClampIsExtended = true;
}

void retractRotationClamp() {
    digitalWrite(ROTATION_CLAMP, LOW); // Retracted
    rotationClampIsExtended = false;
}
