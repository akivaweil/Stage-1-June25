#include "StateMachine/09_SUCTION_ERROR.h"
#include "StateMachine/11_ERROR_RESET.h"  // For error timing constants
#include "StateMachine/StateManager.h"
#include <Bounce2.h>

// External references to functions from main.cpp (LED functions only)
extern void showRedLed();
extern void turnRedLedOff();
extern void turnYellowLedOff();
extern void turnGreenLedOff();
extern void turnBlueLedOff();

// External references for cut motor homing
extern void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
extern Bounce cutHomingSwitch;

// SUCTION ERROR
// Handles wood suction error detection and recovery.
// This state is entered from CUTTING (Step 1) if the WOOD_SUCTION_CONFIRM_SENSOR indicates an error (LOW = no suction detected).
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
    static bool waitingForSensorClear = true;
    static bool sensorCleared = false;
    static unsigned long sensorClearedTime = 0;
    static const unsigned long WAIT_AFTER_CLEAR_MS = 3000; // 3 second wait

    // Step 1: Wait for sensor to clear, then wait 3 seconds before homing cut motor
    if (!hasHomedCutMotor) {
        Bounce* suctionSensor = getSuctionSensorBounce();
        
        if (waitingForSensorClear) {
            // Update sensor and check if it has cleared (HIGH)
            if (suctionSensor) {
                suctionSensor->update();
                if (suctionSensor->read() == HIGH) {
                    // Sensor cleared - start 3 second timer
                    sensorCleared = true;
                    sensorClearedTime = millis();
                    waitingForSensorClear = false;
                }
            }
        } else if (sensorCleared && (millis() - sensorClearedTime >= WAIT_AFTER_CLEAR_MS)) {
            // 3 seconds have passed since sensor cleared - now home the motor
            homeCutMotorBlocking(cutHomingSwitch, 10000); // 10 second timeout
            hasHomedCutMotor = true;
            
            // Reset flags and transition back to CUTTING state
            hasHomedCutMotor = false;
            waitingForSensorClear = true;
            sensorCleared = false;
            sensorClearedTime = 0;
            
            turnRedLedOff(); // Turn off error LED
            changeState(STATE_CUTTING); // Return to cutting state
        }
    }

    // Step 2: Blink STATUS_LED_RED using defined suction error timing interval
    if (millis() - lastSuctionErrorBlinkTime >= SUCTION_ERROR_BLINK_INTERVAL) {
        lastSuctionErrorBlinkTime = millis();
        suctionErrorBlinkState = !suctionErrorBlinkState;
        if(suctionErrorBlinkState) showRedLed(); else turnRedLedOff();
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
        
        // Reset all flags for next time this state is entered
        hasHomedCutMotor = false;
        waitingForSensorClear = true;
        sensorCleared = false;
        sensorClearedTime = 0;
        
        changeState(STATE_HOMING);        // Go to HOMING to re-initialize using proper StateManager method
    }
} 