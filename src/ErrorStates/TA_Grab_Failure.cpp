#include "ErrorStates/TA_Grab_Failure.h"
#include "ErrorStates/Error_Reset.h"
#include "StateMachine/StateManager.h"
#include "Config/Pins_Definitions.h"
#include <Bounce2.h>

//* ************************************************************************
//* ********************* TA GRAB FAILURE **********************************
//* ************************************************************************
// Entered from CUTTING when WOOD_SUCTION_CONFIRM_SENSOR reads LOW (no wood
// grabbed by the transfer arm):
//   - Step 0 (start of cut, before any motion)
//   - Step 1 (at SUCTION_SENSOR_CHECK_DISTANCE of cut travel)
//
// On entry: forces TA signal LOW, homes the cut motor.
// While held: blinks red+blue LEDs. Fires a second TA retry pulse once after
//             TA_RETRY_DELAY_MS in case the arm missed the first pickup.
// Resume:    when the suction sensor reads HIGH continuously for
//             GRAB_CLEAR_HOLD_MS, transitions back to CUTTING. CUTTING's
//             onEnter resets the step counter so Step 0 runs fresh.

static const unsigned long TA_RETRY_DELAY_MS = 8000;
static const unsigned long GRAB_CLEAR_HOLD_MS = 2000;

extern void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
extern Bounce cutHomingSwitch;
extern void sendSignalToTA();
extern bool signalTAActive;

void handleTaGrabFailureState() {
    static bool hasEntered = false;
    static unsigned long enterTime = 0;
    static bool taRetryPulseSent = false;
    static unsigned long sensorHighSince = 0;
    static unsigned long lastBlinkTime = 0;
    static bool blinkOn = false;

    if (!hasEntered) {
        digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
        signalTAActive = false;

        FastAccelStepper* feedMotor = getFeedMotor();
        if (feedMotor && feedMotor->isRunning()) {
            feedMotor->stopMove();
        }

        homeCutMotorBlocking(cutHomingSwitch, 10000);

        enterTime = millis();
        taRetryPulseSent = false;
        sensorHighSince = 0;
        lastBlinkTime = 0;
        blinkOn = false;
        hasEntered = true;
    }

    if (millis() - lastBlinkTime >= SUCTION_ERROR_BLINK_INTERVAL) {
        lastBlinkTime = millis();
        blinkOn = !blinkOn;
        if (blinkOn) { turnRedLedOn();  turnBlueLedOn();  }
        else         { turnRedLedOff(); turnBlueLedOff(); }
    }
    turnYellowLedOff();
    turnGreenLedOff();

    if (!taRetryPulseSent && (millis() - enterTime >= TA_RETRY_DELAY_MS)) {
        sendSignalToTA();
        taRetryPulseSent = true;
    }

    Bounce* suctionSensor = getSuctionSensorBounce();
    bool sensorHigh = (suctionSensor && suctionSensor->read() == HIGH);

    if (!sensorHigh) {
        sensorHighSince = 0;
        return;
    }

    if (sensorHighSince == 0) {
        sensorHighSince = millis();
        return;
    }

    if (millis() - sensorHighSince >= GRAB_CLEAR_HOLD_MS) {
        hasEntered = false;
        turnRedLedOff();
        turnBlueLedOff();
        turnYellowLedOn();
        changeState(CUTTING);
    }
}
