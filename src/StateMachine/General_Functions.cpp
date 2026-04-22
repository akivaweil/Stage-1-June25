// Switch-handling and cycle-start helpers used across state handlers.
// Hardware-level helpers (clamps, LEDs, motors, servo, TA signaling) now live
// under src/Hardware/. Keep this file for pure state/switch logic.
#include <Arduino.h>
#include <Bounce2.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🎚️ SWITCH LOGIC                                                     ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

void handleErrorAcknowledgement() {
    // This handles the general error acknowledgement via reloadSwitch
    // It was present in the main loop and also within the CUTTING state's homePositionErrorDetected block.
    if (reloadSwitch.rose() && (currentState == ERROR || currentState == CUTTING)) {
        if (currentState == ERROR) {
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Set flag, main loop will see this for ERROR state
        }
        // If in CUTTING, setting errorAcknowledged might be used by the CUTTING state to proceed.
        // The original CUTTING state logic directly transitioned. For now, we set the flag.
    }
}

void handleStartSwitchSafety() {
    // startSwitchSafe is gated until the start switch has been observed OFF at least once.
    if (!startSwitchSafe && startCycleSwitch.fell()) {
        startSwitchSafe = true;
    }
}

void handleStartSwitchContinuousMode(){
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
    }
}

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔁 CYCLE-START GATE                                                  ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

bool shouldStartCycle() {
    // Condition from IDLE state to start a cycle
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🚩 SHARED FLAG ACCESSORS                                             ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

bool getComingFromNoWoodWithSensorsClear() {
    return comingFromNoWoodWithSensorsClear;
}

void setComingFromNoWoodWithSensorsClear(bool value) {
    comingFromNoWoodWithSensorsClear = value;
}
