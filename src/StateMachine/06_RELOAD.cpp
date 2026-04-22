#include "StateMachine/06_RELOAD.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔄 RELOAD STATE                                                      ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Handles reload mode operations when the reload switch is active.
// Provides safe conditions for loading new wood pieces into the system.
// Maintains clamps in retracted position for safe wood loading.

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 1: MAINTAIN BLUE LED TO INDICATE RELOAD MODE IS ACTIVE         ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ STEP 2: MONITOR RELOAD SWITCH STATE FOR EXIT CONDITION              ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// When reload switch is turned OFF, exit reload mode and return to IDLE

void executeReloadState() {
    // Check if reload switch is turned off - if so, exit reload mode
    bool reloadSwitchOn = getReloadSwitch()->read() == HIGH;
    if (!reloadSwitchOn) {
        // Reload switch turned off - exit reload mode
        changeState(IDLE);
    }
}

void onEnterReloadState() {
    // Set flag
    setIsReloadMode(true);

    // Ensure clamps are properly positioned for reload mode
    retractFeedClamp();
    retract2x4SecureClamp();
    showBlueLed();

    // Reset any state flags that might be set
    setComingFromNoWoodWithSensorsClear(false);
}

void onExitReloadState() {
    // Cleanup when exiting reload mode
    setIsReloadMode(false);
    turnBlueLedOff();
}
