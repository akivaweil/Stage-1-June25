#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

//* ************************************************************************
//* ************************ DROPOFF SEQUENCE FUNCTIONS ***************************
//* ************************************************************************
// Functions for dropoff sequence operations

// Setup Z-axis for slower dropoff operations
void setupZAxisForDropoff() {
  transferArm.getZStepper()->setSpeedInHz(Z_DROPOFF_MAX_SPEED);
  transferArm.getZStepper()->setAcceleration(Z_DROPOFF_ACCELERATION);
  smartLog("Z-axis configured for slower dropoff movement");
  smartLog("Max Speed: " + String(Z_DROPOFF_MAX_SPEED) + 
           ", Acceleration: " + String(Z_DROPOFF_ACCELERATION));
}

// Release the object by turning off vacuum
void releaseObject() {
  digitalWrite(SOLENOID_RELAY_PIN, LOW);
  smartLog("Vacuum solenoid turned OFF - object released");
}

// Restore Z-axis to normal speed and acceleration for upward movement
void restoreZAxisToNormalSpeed() {
  transferArm.getZStepper()->setSpeedInHz(Z_MAX_SPEED);
  transferArm.getZStepper()->setAcceleration(Z_ACCELERATION);
  smartLog("Z-axis speed restored to normal for upward movement");
  smartLog("Max Speed: " + String(Z_MAX_SPEED) + 
           ", Acceleration: " + String(Z_ACCELERATION));
}

// Get current Z position for debugging
float getCurrentZPosition() {
  return transferArm.getZStepper()->getCurrentPosition();
}

// Check if vacuum is currently active
bool isVacuumActive() {
  return digitalRead(SOLENOID_RELAY_PIN) == HIGH;
} 