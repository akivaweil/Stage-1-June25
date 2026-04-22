#include <Arduino.h>
#include "StateMachine/General_Functions.h"
#include "Config/Config.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🤖 TRANSFER ARM SIGNALING                                            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Pulses the digital line that tells the Transfer Arm (separate machine) to
// grab the cut diamond. In Minis mode, a 500 ms pre-delay is applied so the
// TA is holding position before the pulse arrives.

// Global non-blocking delay variables for TA signal
unsigned long taSignalDelayStartTime = 0;
bool taSignalDelayPending = false;

void sendSignalToTA() {
  // Instead of setting HIGH immediately, start a non-blocking delay
  // Check config mode: if Minis mode (1), add 500ms delay. Otherwise no delay.
  int mode = 0;

  // Since we cannot include websocket_dashboard.h here due to conflicts,
  // we will simply assume mode 0 if we can't access it, or better yet,
  // we can declare the function signature manually
  extern int getCurrentConfigMode();
  mode = getCurrentConfigMode();

  if (mode == 1) {
      // Minis mode: Start 500ms non-blocking delay
      if (!taSignalDelayPending) {
          taSignalDelayStartTime = millis();
          taSignalDelayPending = true;
      }
  } else {
      // 3 Inch mode: Execute immediately
      digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
      signalTAStartTime = millis();
      taSignalActive = true;
      taSignalDelayPending = false;
  }
}

// Function to handle Transfer Arm signal timing (including start delay)
void handleTASignalTiming() {
  // Handle start delay if active
  if (taSignalDelayPending) {
      if (millis() - taSignalDelayStartTime >= 500) {
          digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
          signalTAStartTime = millis();
          taSignalActive = true;
          taSignalDelayPending = false;
      }
  }

  // Handle signal duration
  if (taSignalActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
    taSignalActive = false;
  }
}
