#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ CONFIGURATION CONSTANTS *********************
//* ************************************************************************

// Timing Configuration
const unsigned long ATTENTION_SEQUENCE_DELAY_MS = 125;    // Delay between feed clamp movements in attention sequence
const unsigned long CYLINDER_ACTION_DELAY_MS = 150;       // Delay for cylinder actions
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 30;    // Delay between sensor verification attempts
const unsigned long SENSOR_READING_DELAY_MS = 10;         // Delay between sensor readings for stability

// Movement Configuration
const float FEED_MOTOR_MOVEMENT_1_DISTANCE = 1.0;          // inches - Intermediate position for no-wood sequence (safe distance)
const float FEED_MOTOR_MOVEMENT_2_DISTANCE = 1.0;          // inches - Final position (safe distance within travel limits)

// Sequence Configuration
const int ATTENTION_SEQUENCE_TOTAL_MOVEMENTS = 5;         // Total number of movements in attention sequence
const int SENSOR_VERIFICATION_ATTEMPTS = 3;               // Number of attempts for sensor verification

//* ************************************************************************
//* ************************ STATE VARIABLES ******************************
//* ************************************************************************

// Main sequence tracking
static int currentStep = 0;
static unsigned long cylinderActionTime = 0;
static bool waitingForCylinder = false;

// Attention sequence tracking
static int attentionStep = 0;
static unsigned long attentionStartTime = 0;

//* ************************************************************************
//* ************************ RETURNING NO 2X4 STATE ***********************
//* ************************************************************************
// Handles the RETURNING_NO_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.
// Latest working code - committed: 2025-09-02 14:56:20

//! ************************************************************************
//! STEP 1: INITIALIZE SEQUENCE - MOVE CUT MOTOR HOME AND RETRACT 2X4 CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 2: WAIT FOR CUT MOTOR HOME AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 3: WAIT FOR FEED MOTOR HOME AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 4: MOVE FEED MOTOR TO MOVEMENT 1 POSITION
//! ************************************************************************

//! ************************************************************************
//! STEP 5: WAIT FOR FEED MOTOR AT MOVEMENT 1 POSITION AND EXTEND FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 6: MOVE FEED MOTOR BACK TO HOME POSITION
//! ************************************************************************

//! ************************************************************************
//! STEP 7: WAIT FOR FEED MOTOR AT HOME AND START ATTENTION SEQUENCE
//! ************************************************************************

//! ************************************************************************
//! STEP 8: ATTENTION GETTING SEQUENCE - INTENSE FEED CLAMP EXTENSION/RETRACTION
//! ************************************************************************

//! ************************************************************************
//! STEP 9: MOVE FEED MOTOR TO HOME (AFTER ATTENTION SEQUENCE)
//! ************************************************************************

//! ************************************************************************
//! STEP 10: WAIT FOR FEED MOTOR HOME AND RETRACT FEED CLAMP
//! ************************************************************************

//! ************************************************************************
//! STEP 11: MOVE FEED MOTOR TO FINAL POSITION (4.75 INCHES)
//! ************************************************************************

//! ************************************************************************
//! STEP 12: MOVE FEED MOTOR BACK TO HOME AFTER FINAL POSITION
//! ************************************************************************

//! ************************************************************************
//! STEP 13: WAIT FOR FEED MOTOR AT HOME AFTER FINAL POSITION
//! ************************************************************************

//! ************************************************************************
//! STEP 14: VERIFY CUT HOME POSITION AND COMPLETE SEQUENCE
//! ************************************************************************

//* ************************************************************************
//* ************************ PUBLIC INTERFACE *****************************
//* ************************************************************************

void executeReturningNo2x4State() {
    handleReturningNo2x4Sequence(); 
}

void onEnterReturningNo2x4State() {
    initializeReturningNo2x4Sequence();
}

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

//* ************************************************************************
//* ************************ INITIALIZATION *******************************
//* ************************************************************************

void initializeReturningNo2x4Sequence() {
    //! ************************************************************************
    //! STEP 1: INITIALIZE RETURNING NO 2X4 SEQUENCE
    //! ************************************************************************
    
    // Reset consecutive yeswood counter when nowood state occurs
    resetConsecutiveYeswoodCount();
    
    // Initialize RETURNING_NO_2x4 sequence from CUTTING_state logic
    configureCutMotorForReturn();
    moveCutMotorToHome();
    configureFeedMotorForNormalOperation();

    turnBlueLedOn();
    turnYellowLedOff();
    
    // Initialize step tracking
    currentStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
    attentionStep = 0;
    attentionStartTime = 0;
}

//* ************************************************************************
//* ************************ MAIN SEQUENCE HANDLER ************************
//* ************************************************************************

void handleReturningNo2x4Sequence() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    // First time entering this specific RETURNING_NO_2x4 logic path
    if (currentStep == 0) {
        retract2x4SecureClamp();
        if (feedMotor && (feedMotor->getCurrentPosition() != 0 || feedMotor->isRunning())) {
            feedMotor->moveTo(0);
        }
        currentStep = 1;
    }

    // Handle cylinder action delays
    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        currentStep++; 
    }
    
    // Process current step if not waiting for cylinder
    if (!waitingForCylinder) {
        processCurrentStep(currentStep);
    }
}

//* ************************************************************************
//* ************************ STEP PROCESSING ******************************
//* ************************************************************************

void processCurrentStep(int step) {
    switch (step) { 
        case 1: // Wait for cut motor, then extend feed clamp
            handleCutMotorWaitAndExtendFeedClamp();
            break;
            
        case 2: // Wait for feed motor, then retract feed clamp
            handleFeedMotorWaitAndRetractFeedClamp();
            break;
            
        case 3: // Move feed motor to Movement 1 position
            handleFeedMotorMoveToPosition1();
            break;
            
        case 4: // Wait for feed motor at Movement 1 position, extend feed clamp
            handleFeedMotorWaitAtPosition1AndExtendClamp();
            break;
            
        case 5: // Move feed motor back to home position
            handleFeedMotorMoveBackToHome();
            break;
            
        case 6: // Wait for feed motor at home, then start attention sequence
            handleFeedMotorWaitAtHomeForAttention();
            break;
            
        case 7: // Attention-getting sequence
            handleAttentionSequence();
            break;
            
        case 8: // Move feed motor to home (after attention sequence)
            handleFeedMotorMoveToHome();
            break;
            
        case 9: // Wait for feed motor at home, retract feed clamp
            handleFeedMotorWaitAtHomeAndRetractClamp();
            break;
            
        case 10: // Move feed motor to Movement 2 position
            handleFeedMotorMoveToPosition2();
            break;
            
        case 11: // Move feed motor back to home after Movement 2
            handleFeedMotorMoveBackToHomeAfterPosition2();
            break;
            
        case 12: // Wait for feed motor at home after Movement 2
            handleFeedMotorWaitAtHomeAfterPosition2();
            break;
            
        case 13: // Final step: verify sensors and complete sequence
            handleFinalVerificationAndCompletion();
            break;
    }
}

//* ************************************************************************
//* ************************ STEP HANDLERS ********************************
//* ************************************************************************

void handleCutMotorWaitAndExtendFeedClamp() {
    FastAccelStepper* cutMotor = getCutMotor();
    
    if (cutMotor && !cutMotor->isRunning()) {
        extendFeedClamp();
        cylinderActionTime = millis();
        waitingForCylinder = true; // Will cause currentStep to increment to 2 after delay
    }
}

void handleFeedMotorWaitAndRetractFeedClamp() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    if (feedMotor && !feedMotor->isRunning()) {
        retractFeedClamp();
        cylinderActionTime = millis();
        waitingForCylinder = true; // Increments to 3
    }
}

void handleFeedMotorMoveToPosition1() {
    configureFeedMotorForNormalOperation(); // Ensure correct config
    moveFeedMotorToPosition(FEED_MOTOR_MOVEMENT_1_DISTANCE);
    currentStep = 4; // Directly advance step here as it's a command
}

void handleFeedMotorWaitAtPosition1AndExtendClamp() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    if (feedMotor && !feedMotor->isRunning()) {
        extendFeedClamp();
        currentStep = 5; // Move to move back to home
    }
}

void handleFeedMotorMoveBackToHome() {
    configureFeedMotorForNormalOperation();
    moveFeedMotorToHome();
    currentStep = 6; // Directly advance step
}

void handleFeedMotorWaitAtHomeForAttention() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    if (feedMotor && !feedMotor->isRunning()) {
        currentStep = 7; // Move to attention sequence
    }
}

void handleAttentionSequence() {
    if (attentionStep == 0) {
        // First step: retract feed clamp
        retractFeedClamp();
        attentionStep = 1;
        attentionStartTime = millis();
    } else if (attentionStep < ATTENTION_SEQUENCE_TOTAL_MOVEMENTS && 
               millis() - attentionStartTime >= ATTENTION_SEQUENCE_DELAY_MS) {
        
        // Alternate between extend and retract
        if (attentionStep % 2 == 1) {
            extendFeedClamp();
        } else {
            retractFeedClamp();
        }
        
        attentionStep++;
        attentionStartTime = millis();
        
        // Check if sequence is complete
        if (attentionStep >= ATTENTION_SEQUENCE_TOTAL_MOVEMENTS) {
            // Ensure clamp is extended at end of attention sequence
            extendFeedClamp();
            attentionStep = 0; // Reset for next time
            currentStep = 8; // Move to next step
        }
    }
}

void handleFeedMotorMoveToHome() {
    configureFeedMotorForNormalOperation();
    moveFeedMotorToHome();
    currentStep = 9; // Directly advance step
}

void handleFeedMotorWaitAtHomeAndRetractClamp() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    if (feedMotor && !feedMotor->isRunning()) {
        retractFeedClamp();
        cylinderActionTime = millis();
        waitingForCylinder = true; // Increments to 10
    }
}

void handleFeedMotorMoveToPosition2() {
    configureFeedMotorForNormalOperation();
    moveFeedMotorToPosition(FEED_MOTOR_MOVEMENT_2_DISTANCE);
    currentStep = 11; // Directly advance step
}

void handleFeedMotorMoveBackToHomeAfterPosition2() {
    configureFeedMotorForNormalOperation();
    moveFeedMotorToHome();
    currentStep = 12; // Directly advance step
}

void handleFeedMotorWaitAtHomeAfterPosition2() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    if (feedMotor && !feedMotor->isRunning()) {
        currentStep = 13; // Move to final verification
    }
}

void handleFinalVerificationAndCompletion() {
    FastAccelStepper* feedMotor = getFeedMotor();
    FastAccelStepper* cutMotor = getCutMotor();
    
    if (feedMotor && !feedMotor->isRunning()) {
        // Verify cut home position
        verifyCutHomePosition(cutMotor);
        
        // Wait until both sensors are not active before proceeding
        if (checkBothSensorsNotActive()) {
            completeReturningNo2x4Sequence();
        } else {
            // At least one sensor is still active - keep waiting
            retractFeedClamp();
            retract2x4SecureClamp();
        }
    }
}

//* ************************************************************************
//* ************************ SENSOR VERIFICATION **************************
//* ************************************************************************

void verifyCutHomePosition(FastAccelStepper* cutMotor) {
    bool sensorDetectedHome = false;
    
    for (int i = 0; i < SENSOR_VERIFICATION_ATTEMPTS; i++) {
        delay(SENSOR_VERIFICATION_DELAY_MS);  
        getCutHomingSwitch()->update();
        bool sensorReading = getCutHomingSwitch()->read();
        
        if (sensorReading == HIGH) {
            sensorDetectedHome = true;
            if (cutMotor) cutMotor->setCurrentPosition(0); 
            break;  
        }
    }
}

bool checkBothSensorsNotActive() {
    // Read both sensors with multiple samples for reliability
    bool firstCutSensorNotActive = true;
    bool woodPresentSensorNotActive = true;
    
    // Take multiple readings for stability
    for (int i = 0; i < SENSOR_VERIFICATION_ATTEMPTS; i++) {
        delay(SENSOR_READING_DELAY_MS); // Small delay between readings
        
        // Check FIRST_CUT_OR_WOOD_FWD_ONE sensor (HIGH = not active)
        if (digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == LOW) {
            firstCutSensorNotActive = false;
        }
        
        // Check _2x4_PRESENT_SENSOR (HIGH = not active)  
        if (digitalRead(_2x4_PRESENT_SENSOR) == LOW) {
            woodPresentSensorNotActive = false;
        }
    }
    
    // Return true only if both sensors are not active (both HIGH)
    return (firstCutSensorNotActive && woodPresentSensorNotActive);
}

//* ************************************************************************
//* ************************ SEQUENCE COMPLETION **************************
//* ************************************************************************

void completeReturningNo2x4Sequence() {
    // Both sensors not active - extend secure wood clamp and complete sequence
    extend2x4SecureClamp();
    
    turnYellowLedOff();
    turnBlueLedOn(); 

    resetReturningNo2x4Steps();
    setCuttingCycleInProgress(false);
    
    // Check if cycle switch is currently ON - if yes, require cycling
    if (getStartCycleSwitch()->read() == HIGH) {
        setStartSwitchSafe(false);
    }
    
    // Set flag to indicate we're coming from no-wood cycle with sensors clear
    setComingFromNoWoodWithSensorsClear(true);
    
    // Transition to IDLE state - secure clamp will remain extended
    changeState(IDLE);
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ****************************
//* ************************************************************************

void resetReturningNo2x4Steps() {
    currentStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
    attentionStep = 0;
    attentionStartTime = 0;
}