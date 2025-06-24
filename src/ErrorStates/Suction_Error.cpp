#include "ErrorStates/Suction_Error.h"
#include "ErrorStates/Error_Reset.h"  // For error timing constants
#include "StateMachine/StateManager.h"
#include <Bounce2.h>

// External references to functions from main.cpp (LED functions only)
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOff();
extern void turnGreenLedOff();
extern void turnBlueLedOff();

// External references for cut motor homing
extern void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
extern Bounce cutHomingSwitch;

//* ************************************************************************
//* ********************* SUCTION ERROR ************************************
//* ************************************************************************
// Handles wood suction error detection and recovery.
// This state is entered from CUTTING (Step 1) if the WOOD_SUCTION_CONFIRM_SENSOR indicates an error (LOW).
// Step 1: Automatically home the cut motor upon entering this state for safety.
// Step 2: Slowly blink the red LED using defined suction error timing interval.
// Step 3: Ensure yellow, green, and blue LEDs are off.
// Step 4: Monitor the start cycle switch.
// Step 5: If the start cycle switch shows a rising edge (OFF to ON transition):
//          - Print a message about resetting from suction error.
//          - Turn off the red LED.
//          - Set continuousModeActive to false.
//          - Set startSwitchSafe to false (requires user to cycle switch again for a new start).
//          - Transition to HOMING state to re-initialize the system.
void handleSuctionErrorState() {
    static bool hasHomedCutMotor = false;
    static unsigned long lastSuctionErrorBlinkTime = 0;
    static bool suctionErrorBlinkState = false;

    // Step 1: Home cut motor immediately when entering this state (only once)
    if (!hasHomedCutMotor) {
        //serial.println("SUCTION ERROR: Automatically homing cut motor for safety...");
        homeCutMotorBlocking(cutHomingSwitch, 10000); // 10 second timeout
        hasHomedCutMotor = true;
        //serial.println("Cut motor homing complete. Now monitoring for user reset.");
    }

    // Step 2: Blink STATUS_LED_RED using defined suction error timing interval
    if (millis() - lastSuctionErrorBlinkTime >= SUCTION_ERROR_BLINK_INTERVAL) {
        lastSuctionErrorBlinkTime = millis();
        suctionErrorBlinkState = !suctionErrorBlinkState;
        if(suctionErrorBlinkState) turnRedLedOn(); else turnRedLedOff();
    }
    
    // Step 3: Ensure other LEDs are off
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();

    // Step 4 & 5: Use StateManager to access switches instead of global variables
    if (getStartCycleSwitch()->rose()) { // Check for start switch OFF to ON transition
        //serial.println("Start cycle switch toggled ON. Resetting from suction error. Transitioning to HOMING.");
        turnRedLedOff();   // Turn off error LED explicitly before changing state
        
        setContinuousModeActive(false); // Ensure continuous mode is off
        setStartSwitchSafe(false);      // Require user to cycle switch OFF then ON for a new actual start
        
        // Reset the homing flag for next time this state is entered
        hasHomedCutMotor = false;
        
        changeState(HOMING);        // Go to HOMING to re-initialize using proper StateManager method
    }
} 