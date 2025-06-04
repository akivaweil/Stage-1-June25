#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

// External variable to track vacuum activation
extern bool vacuumActivatedDuringDescent;

//* ************************************************************************
//* ************************ PICKUP SEQUENCE FUNCTIONS ***************************
//* ************************************************************************
// Functions for pickup sequence operations

// Setup Z-axis for pickup operations
void setupZAxisForPickup() {
  // Initialize Z-axis to normal speed and acceleration
  transferArm.getZStepper()->setSpeedInHz(Z_MAX_SPEED);
  transferArm.getZStepper()->setAcceleration(Z_ACCELERATION);
  smartLog("Z-axis configured for pickup operations");
  delay(100);
}

// Activate vacuum during Z descent when reaching the suction start position
void activateVacuumDuringDescent() {
  if (!vacuumActivatedDuringDescent &&
      transferArm.getZStepper()->getCurrentPosition() >= Z_SUCTION_START_POS) {
    digitalWrite(SOLENOID_RELAY_PIN, HIGH);
    vacuumActivatedDuringDescent = true;
    smartLog("Vacuum activated during descent at Z: " +
             String(transferArm.getZStepper()->getCurrentPosition()));
  }
}

// Reset pickup sequence variables
void resetPickupSequence() {
  vacuumActivatedDuringDescent = false;
  smartLog("Pickup sequence variables reset");
} 