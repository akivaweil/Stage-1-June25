#include "../../../include/Config/Config.h"
#include "../../../include/Config/Pins_Definitions.h"
#include "../../../include/TransferArm.h"
#include "../../../include/Utils.h"

//* ************************************************************************
//* ************************ IDLE FUNCTIONS ***************************
//* ************************************************************************
// Functions for the idle state operations

// Check if pick cycle should be triggered
bool checkPickCycleTrigger() {
  // Check start button or Stage 1 signal
  if (transferArm.getStartButton().read() == HIGH ||
      transferArm.getStage1Signal().read() == HIGH) {
    return true;
  }
  return false;
} 