#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

//* ************************************************************************
//* ************************ GENERAL FUNCTIONS ***************************
//* ************************************************************************
// General utility functions used across multiple state sequences
// Note: moveToPosition and Wait functions are implemented in Utils.cpp

// Get state string for debugging (placeholder - specific implementations in each state)
const char* getStateString(int state) {
  switch (state) {
    case 0: return "IDLE";
    case 1: return "PICKUP_SEQUENCE";
    case 2: return "TRANSPORT_SEQUENCE";
    case 3: return "DROPOFF_SEQUENCE";
    case 4: return "COMPLETION_SEQUENCE";
    default: return "UNKNOWN";
  }
}

// Initialize all state sequences
void initializeAllStateSequences() {
  // Initialize Stage 2 signal pin
  pinMode(STAGE2_SIGNAL_PIN, OUTPUT);
  digitalWrite(STAGE2_SIGNAL_PIN, LOW);
  
  smartLog("All state sequences initialized");
}

// Reset all sequence timers and flags
void resetAllSequenceVariables() {
  smartLog("All sequence variables reset");
}

// Emergency stop function for all sequences
void emergencyStopAllSequences() {
  // Stop all steppers
  transferArm.getXStepper()->forceStop();
  transferArm.getZStepper()->forceStop();
  
  // Turn off vacuum
  digitalWrite(SOLENOID_RELAY_PIN, LOW);
  
  // Turn off Stage 2 signal
  digitalWrite(STAGE2_SIGNAL_PIN, LOW);
  
  smartLog("EMERGENCY STOP - All sequences halted");
} 