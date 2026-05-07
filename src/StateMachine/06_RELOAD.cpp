#include <Arduino.h>
#include "StateMachine/06_RELOAD.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "WebSocketDashboard/websocket_dashboard.h"

// Stagger timing for reload-mode clamp transitions
static const unsigned long RELOAD_TOP_CLAMP_RETRACT_LEAD_MS = 100; // On entry: top clamp retracts this many ms before feed clamp
static const unsigned long RELOAD_FEED_CLAMP_EXTEND_LEAD_MS = 100; // On exit:  feed clamp extends this many ms before top clamp (matches top clamp extension duration so feed is fully seated throughout)

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

    // Stagger clamp retraction: top clamp first, then feed clamp 100ms later
    retractTopClamp();
    delay(RELOAD_TOP_CLAMP_RETRACT_LEAD_MS);
    retractFeedClamp();
    showBlueLed();

    // Reset any state flags that might be set
    setComingFromNoWoodWithSensorsClear(false);
}

void onExitReloadState() {
    // Stagger clamp extension on exit: feed clamp first, then top clamp after the lead time
    // (lead matches top clamp's ~100ms extension stroke so feed is fully seated throughout)
    extendFeedClamp();
    delay(RELOAD_FEED_CLAMP_EXTEND_LEAD_MS);
    extendTopClamp();

    // Cleanup when exiting reload mode
    setIsReloadMode(false);
    turnBlueLedOff();
}
