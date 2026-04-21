#include "ErrorStates/Suction_Error.h"
#include "ErrorStates/Error_Reset.h"  // For error timing constants
#include "StateMachine/StateManager.h"
#include "Config/Pins_Definitions.h"
#include <Bounce2.h>

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ SUCTION ERROR CONFIG ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Delay after entering SUCTION_ERROR before firing a second TA pulse to
// re-trigger the transfer arm (in case it missed the first pickup).
static const unsigned long SUCTION_ERROR_TA_RETRY_DELAY_MS = 8000;

// External references to functions from main.cpp (LED functions only)
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOff();
extern void turnGreenLedOff();
extern void turnBlueLedOff();

// External references for cut motor homing
extern void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
extern Bounce cutHomingSwitch;

// External references for TA signal handling
extern void sendSignalToTA();
extern bool signalTAActive;
extern unsigned long signalTAStartTime;

//* ************************************************************************
//* ********************* SUCTION ERROR ************************************
//* ************************************************************************
// Handles wood suction error detection and recovery.
// This state is entered from CUTTING (Step 1) if the WOOD_SUCTION_CONFIRM_SENSOR indicates an error (LOW = no suction detected).
// Step 1: Automatically home the cut motor upon entering this state for safety.
// Step 2: Slowly blink the red LED using defined suction error timing interval.
// Step 3: Ensure yellow, green, and blue LEDs are off.
// Step 4: Monitor the start cycle switch.
// Step 5: If the start cycle switch shows a rising edge (OFF to ON transition):
//          - Print a message about resetting from suction error.
//          - Turn off the red LED.
//          - Set continuousModeActive to false.
//          - Set startSwitchSafe to false (requires user to cycle switch again for a new start).
//          - Transition to HOMING state to re-initialize the system.
void handleSuctionErrorState() {
    static bool hasHomedCutMotor = false;
    static unsigned long lastSuctionErrorBlinkTime = 0;
    static bool suctionErrorBlinkState = false;
    static unsigned long suctionErrorEnterTime = 0;
    static bool taRetryPulseSent = false;

    //! ************************************************************************
    //! STEP 1: HOME CUT MOTOR AND FORCE TA SIGNAL LOW ON FIRST ENTRY
    //! ************************************************************************
    if (!hasHomedCutMotor) {
        // Kill any stuck TA signal that could be left HIGH because the blocking
        // homing call below prevents handleCommonOperations() from running.
        digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
        signalTAActive = false;

        suctionErrorEnterTime = millis();
        taRetryPulseSent = false;

        homeCutMotorBlocking(cutHomingSwitch, 10000); // 10 second timeout
        hasHomedCutMotor = true;
    }

    //! ************************************************************************
    //! STEP 2: BLINK RED LED AT SUCTION ERROR INTERVAL
    //! ************************************************************************
    if (millis() - lastSuctionErrorBlinkTime >= SUCTION_ERROR_BLINK_INTERVAL) {
        lastSuctionErrorBlinkTime = millis();
        suctionErrorBlinkState = !suctionErrorBlinkState;
        if(suctionErrorBlinkState) turnRedLedOn(); else turnRedLedOff();
    }
    
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();

    //! ************************************************************************
    //! STEP 3: FIRE SECOND 500ms TA PULSE AFTER RETRY DELAY (ONCE)
    //! ************************************************************************
    if (!taRetryPulseSent &&
        (millis() - suctionErrorEnterTime >= SUCTION_ERROR_TA_RETRY_DELAY_MS)) {
        sendSignalToTA();
        taRetryPulseSent = true;
    }

    //! ************************************************************************
    //! STEP 4: WAIT FOR START SWITCH RISING EDGE TO RESET
    //! ************************************************************************
    if (getStartCycleSwitch()->rose()) {
        turnRedLedOff();
        
        setContinuousModeActive(false);
        
        hasHomedCutMotor = false;
        taRetryPulseSent = false;
        suctionErrorEnterTime = 0;
        
        changeState(HOMING);
    }
} 