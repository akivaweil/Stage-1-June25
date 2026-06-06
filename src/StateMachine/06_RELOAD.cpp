#include <Arduino.h>
#include "StateMachine/06_RELOAD.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "WebSocketDashboard/websocket_dashboard.h"

// Stagger timing for reload-mode clamp transitions
static const unsigned long RELOAD_TOP_CLAMP_RETRACT_LEAD_MS = 100; // On entry: top clamp retracts this many ms before feed clamp
static const unsigned long RELOAD_FEED_CLAMP_SETTLE_MS      = 150; // On exit:  feed clamp must already be extended this long before top clamp begins extending
static const unsigned long RELOAD_TOP_CLAMP_EXTEND_MS       = 100; // On exit:  approximate time top clamp takes to fully extend; feed clamp stays in until this elapses

// RELOAD STATE
// Handles reload mode operations when the reload switch is active.
// Provides safe conditions for loading new wood pieces into the system.
// Maintains clamps in retracted position for safe wood loading.

// STEP 1: MAINTAIN BLUE LED TO INDICATE RELOAD MODE IS ACTIVE

// STEP 2: MONITOR RELOAD SWITCH STATE FOR EXIT CONDITION
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
    // Exit sequence:
    //   1. Extend feed clamp and let it settle for RELOAD_FEED_CLAMP_SETTLE_MS
    //      so the wood is held before the top clamp lands.
    //   2. Extend top clamp; wait RELOAD_TOP_CLAMP_EXTEND_MS for it to fully seat.
    //   3. Retract feed clamp now that the top clamp is holding the wood.
    extendFeedClamp();
    delay(RELOAD_FEED_CLAMP_SETTLE_MS);
    extendTopClamp();
    delay(RELOAD_TOP_CLAMP_EXTEND_MS);
    retractFeedClamp();

    // Cleanup when exiting reload mode
    setIsReloadMode(false);
    turnBlueLedOff();
}
