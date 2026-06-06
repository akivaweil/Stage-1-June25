// Switch-handling and cycle-start helpers used across state handlers.
// Hardware-level helpers (clamps, LEDs, motors, servo, TA signaling) now live
// under src/Hardware/. Keep this file for pure state/switch logic.
#include <Arduino.h>
#include <Bounce2.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

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
