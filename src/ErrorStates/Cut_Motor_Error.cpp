#include "ErrorStates/Cut_Motor_Error.h"
#include "ErrorStates/Error_Reset.h"  // For error timing constants
#include "StateMachine/StateManager.h"

// External references to functions from main.cpp (LED and motor functions only)
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOn();
extern void turnYellowLedOff();
extern void stopCutMotor();
extern void stopFeedMotor();

//* ************************************************************************
//* *********************** CUT MOTOR ERROR ********************************
//* ************************************************************************
// Handles cut motor specific error states.
// Step 1: Blink red and yellow LEDs to indicate a cut motor error at standard rate.
// Step 2: Ensure cut and feed motors are stopped.
// Step 3: Wait for the reload switch to be pressed (rising edge) to acknowledge the error.
// Step 4: Once error is acknowledged, transition to ERROR_RESET state.
void handleCutMotorErrorState() {
    // Blink error LEDs using StateManager timing
    if (millis() - getLastErrorBlinkTime() > STANDARD_ERROR_BLINK_INTERVAL) {
        bool newBlinkState = !getErrorBlinkState();
        setErrorBlinkState(newBlinkState);
        
        if(newBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!newBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        
        setLastErrorBlinkTime(millis());
    }
    
    // Keep motors stopped
    stopCutMotor();
    stopFeedMotor();
    
    // Wait for reload switch to acknowledge error using StateManager
    if (getErrorAcknowledged()) {
        changeState(ERROR_RESET);
        //serial.println("Cut motor error acknowledged. Transitioning to ERROR_RESET.");
    }
} 