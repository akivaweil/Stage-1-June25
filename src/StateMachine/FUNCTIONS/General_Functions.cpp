// IMPORTANT NOTE: This file contains general helper functions used throughout the system.
// It relies on the main file for pin definitions and global variable declarations (via extern).
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "StateMachine/StateManager.h"

// External motor object references from main.cpp
extern FastAccelStepper* cutMotor;
extern FastAccelStepper* feedMotor;
extern Bounce cutHomingSwitch;

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
// Contains functions related to signaling other stages or components.

void sendSignalToTA() {
  // Set the signal pin HIGH to trigger Transfer Arm (active HIGH)
  digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
  signalTAStartTime = millis();
  signalTAActive = true;
  //serial.println("TA Signal activated (HIGH).");

  // Only activate servo if it hasn't been activated early
  if (!rotationServoIsActiveAndTiming) {
    Servo* servo = getRotationServo();
    if (servo) {
      // Force servo write with robust control - no attach checks, just send the command
      servo->write(ROTATION_SERVO_ACTIVE_POSITION);
      Serial.printf("FORCED Servo command sent: %d degrees (attach status ignored)\n", ROTATION_SERVO_ACTIVE_POSITION);
    }
    
    rotationServoActiveStartTime = millis();
    rotationServoIsActiveAndTiming = true;
    Serial.print("Rotation servo moved to ");
    Serial.print(ROTATION_SERVO_ACTIVE_POSITION);
    Serial.println(" degrees with TA signal.");
  } else {
    Serial.println("Rotation servo already activated early - skipping normal activation.");
  }
}

//* ************************************************************************
//* ************************* CLAMP FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling various clamps.
// Clamp Logic: LOW = extended, HIGH = retracted
// Rotation Clamp Logic: HIGH = extended, LOW = retracted

void extendFeedClamp() {
    // Feed clamp extends when LOW (inversed logic)
    digitalWrite(FEED_CLAMP, LOW); // Extended
    //serial.println("Feed Clamp Extended");
}

void retractFeedClamp() {
    // Feed clamp retracts when HIGH (inversed logic)
    digitalWrite(FEED_CLAMP, HIGH); // Retracted
    //serial.println("Feed Clamp Retracted");
}

void extend2x4SecureClamp() {
    // 2x4 secure clamp extends when LOW (inversed logic)
    digitalWrite(_2x4_SECURE_CLAMP, LOW); // Extended
    //serial.println("2x4 Secure Clamp Extended");
}

void retract2x4SecureClamp() {
    // 2x4 secure clamp retracts when HIGH (inversed logic)
    digitalWrite(_2x4_SECURE_CLAMP, HIGH); // Retracted
    //serial.println("2x4 Secure Clamp Retracted");
}

void extendRotationClamp() {
    // Rotation clamp extends when HIGH
    digitalWrite(ROTATION_CLAMP, HIGH); // Extended 
    rotationClampExtendTime = millis();
    rotationClampIsExtended = true;
    //serial.println("Rotation Clamp Extended");
}

void retractRotationClamp() {
    // Rotation clamp retracts when LOW
    digitalWrite(ROTATION_CLAMP, LOW); // Retracted 
    rotationClampIsExtended = false; // Assuming we want to clear the flag when explicitly retracting
    //serial.println("Rotation Clamp Retracted");
}

//* ************************************************************************
//* *************************** LED FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling LEDs.

void turnRedLedOn() {
  static bool lastRedLedState = false;
  digitalWrite(STATUS_LED_RED, HIGH);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastRedLedState) {
    //serial.println("Red LED ON");
    lastRedLedState = true;
  }
}

void turnRedLedOff() {
  static bool lastRedLedState = true;
  digitalWrite(STATUS_LED_RED, LOW);
  if (lastRedLedState) {
    //serial.println("Red LED OFF");
    lastRedLedState = false;
  }
}

void turnYellowLedOn() {
  static bool lastYellowLedState = false;
  digitalWrite(STATUS_LED_YELLOW, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastYellowLedState) {
    //serial.println("Yellow LED ON");
    lastYellowLedState = true;
  }
}

void turnYellowLedOff() {
  static bool lastYellowLedState = true;
  digitalWrite(STATUS_LED_YELLOW, LOW);
  if (lastYellowLedState) {
    //serial.println("Yellow LED OFF");
    lastYellowLedState = false;
  }
}

void turnGreenLedOn() {
  static bool lastGreenLedState = false;
  digitalWrite(STATUS_LED_GREEN, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastGreenLedState) {
    //serial.println("Green LED ON");
    lastGreenLedState = true;
  }
}

void turnGreenLedOff() {
  static bool lastGreenLedState = true;
  digitalWrite(STATUS_LED_GREEN, LOW);
  if (lastGreenLedState) {
    //serial.println("Green LED OFF");
    lastGreenLedState = false;
  }
}

void turnBlueLedOn() {
  static bool lastBlueLedState = false;
  digitalWrite(STATUS_LED_BLUE, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  if (!lastBlueLedState) {
    //serial.println("Blue LED ON");
    lastBlueLedState = true;
  }
}

void turnBlueLedOff() {
  static bool lastBlueLedState = true;
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (lastBlueLedState) {
    //serial.println("Blue LED OFF");
    lastBlueLedState = false;
  }
}

void allLedsOff() {
    turnRedLedOff();
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

void handleHomingLedBlink() {
    static unsigned long blinkTimer = 0;
    if (millis() - blinkTimer > 500) {
        blinkState = !blinkState;
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************

void configureCutMotorForCutting() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_NORMAL_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureCutMotorForReturn() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_RETURN_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureFeedMotorForNormalOperation() {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_NORMAL_SPEED);
        feedMotor->setAcceleration((uint32_t)FEED_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureFeedMotorForReturn() {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_RETURN_SPEED);
        feedMotor->setAcceleration((uint32_t)FEED_MOTOR_RETURN_ACCELERATION);
    }
}

void moveCutMotorToCut() {
    if (cutMotor) {
        cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
    }
}

void moveCutMotorToHome() {
    if (cutMotor) {
        cutMotor->moveTo(-0.02 * CUT_MOTOR_STEPS_PER_INCH); // Minimal overshoot
    }
}

void moveCutMotorToCutWithReverseAcceleration() {
    // Reverse acceleration curve: Fast speed for first and last 2.5 inches, slow speed for middle section
    if (cutMotor) {
        // Start with fast speed for first 2.5 inches and move to the FULL distance
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_FAST_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
        cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
        
        // Speed transitions will be handled in the cutting state logic while motor is running
        Serial.println("// Starting continuous cut motion with reverse acceleration curve");
    }
}

void moveFeedMotorToTravel() {
    if (feedMotor) {
        feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void moveFeedMotorToHome() {
    if (feedMotor) {
        feedMotor->moveTo(0);
    }
}

void moveFeedMotorToPosition(float targetPositionInches) {
    if (feedMotor) {
        feedMotor->moveTo(targetPositionInches * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->stopMove();
    }
}

void stopFeedMotor() {
    if (feedMotor) {
        feedMotor->stopMove();
    }
}

// Basic blocking homing function for Cut Motor - can be expanded
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) {
        //serial.println("ERROR: cutMotor is NULL in homeCutMotorBlocking!");
        return;
    }
    
    //serial.println("Starting cut motor homing sequence...");
    Serial.print("Initial switch state: ");
    //serial.println(homingSwitch.read() == HIGH ? "HIGH" : "LOW");
    
    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
    cutMotor->moveTo(-40000);
    
    Serial.print("Cut motor homing speed set to: ");
    //serial.println(CUT_MOTOR_HOMING_SPEED);
    //serial.println("Cut motor moving to -40000 steps...");

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        
        // Add periodic status updates
        static unsigned long lastStatusTime = 0;
        if (millis() - lastStatusTime >= 500) {
            Serial.print("Homing in progress... Switch: ");
            Serial.print(homingSwitch.read() == HIGH ? "HIGH" : "LOW");
            Serial.print(", Position: ");
            Serial.print(cutMotor->getCurrentPosition());
            Serial.print(", Running: ");
            //serial.println(cutMotor->isRunning() ? "YES" : "NO");
            lastStatusTime = millis();
        }
        
        if (millis() - startTime > timeout) {
            //serial.println("Cut motor homing timeout!");
            cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            return;
        }
    }
    
    //serial.println("HOME SWITCH DETECTED! Stopping motor immediately...");
    // Use forceStopAndNewPosition for immediate stopping and set position to 0
    cutMotor->forceStopAndNewPosition(0);
    
    // Add a small delay to ensure motor has fully stopped
    delay(50);
    
    // Verify the switch is still pressed after stopping
    homingSwitch.update();
    if (homingSwitch.read() == HIGH) {
        //serial.println("Cut motor homed successfully - switch confirmed HIGH after stop.");
    } else {
        //serial.println("WARNING: Switch not HIGH after homing - possible contact issue.");
    }
}

// Basic blocking homing function for Feed Motor - can be expanded
void homeFeedMotorBlocking(Bounce& homingSwitch) {
    if (!feedMotor) {
        //serial.println("ERROR: feedMotor is NULL in homeFeedMotorBlocking!");
        return;
    }
    
    //serial.println("Starting feed motor homing sequence...");
    Serial.print("Initial feed sensor state: ");
    //serial.println(homingSwitch.read() == LOW ? "ACTIVE" : "INACTIVE");
    
    // Debug motor setup
    Serial.print("FEED_MOTOR_STEPS_PER_INCH value: ");
    //serial.println(FEED_MOTOR_STEPS_PER_INCH);
    Serial.print("FEED_MOTOR_HOMING_SPEED value: ");
    //serial.println(FEED_MOTOR_HOMING_SPEED);
    
    // Step 1: Move toward home sensor until it triggers
    feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
    
    // Try using runForward() instead of moveTo() for more reliable operation
    //serial.println("Starting feed motor forward run...");
    feedMotor->runForward();
    
    // Verify motor started
    delay(100); // Small delay to let motor start
    Serial.print("Motor started - Running: ");
    Serial.print(feedMotor->isRunning() ? "YES" : "NO");
    Serial.print(", Position: ");
    //serial.println(feedMotor->getCurrentPosition());

    // Add timeout for feed motor homing
    unsigned long startTime = millis();
    const unsigned long FEED_HOME_TIMEOUT = 30000; // 30 seconds timeout

    while (homingSwitch.read() != LOW) {
        homingSwitch.update();
        
        // Add periodic status updates
        static unsigned long lastStatusTime = 0;
        if (millis() - lastStatusTime >= 1000) {
            Serial.print("Feed homing in progress... Sensor: ");
            Serial.print(homingSwitch.read() == LOW ? "ACTIVE" : "INACTIVE");
            Serial.print(", Position: ");
            Serial.print(feedMotor->getCurrentPosition());
            Serial.print(", Running: ");
            //serial.println(feedMotor->isRunning() ? "YES" : "NO");
            lastStatusTime = millis();
            
            // If motor stopped running unexpectedly, restart it
            if (!feedMotor->isRunning()) {
                //serial.println("Motor stopped unexpectedly! Restarting...");
                feedMotor->runForward();
            }
        }
        
        // Check for timeout
        if (millis() - startTime > FEED_HOME_TIMEOUT) {
            //serial.println("Feed motor homing timeout!");
            feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
            return;
        }
    }
    
    //serial.println("FEED HOME SENSOR DETECTED! Stopping motor...");
    feedMotor->forceStopAndNewPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    //serial.println("Feed motor hit home sensor.");
    
    // Step 2: Move to -0.3 inch from home sensor to establish working zero
    //serial.println("Moving feed motor to -0.3 inch from home sensor...");
    feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - FEED_MOTOR_OFFSET_FROM_SENSOR * FEED_MOTOR_STEPS_PER_INCH);
    
    // Wait for move to complete with timeout
    unsigned long moveStartTime = millis();
    while (feedMotor->isRunning()) {
        if (millis() - moveStartTime > 10000) { // 10 second timeout for positioning
            //serial.println("Feed motor positioning timeout!");
            feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
            break;
        }
    }
    
    // Step 3: Set this position (-0.3 inch from sensor) as the new zero
    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    //serial.println("Feed motor homed: 0.3 inch from sensor set as working zero.");
    
    configureFeedMotorForNormalOperation();
    //serial.println("Feed motor homed successfully.");
}

void moveFeedMotorToInitialAfterHoming() {
    if (feedMotor) {
        configureFeedMotorForNormalOperation();
        moveFeedMotorToHome();
        while(feedMotor->isRunning()){
        }
    }
}

// Complex conditional logic
// Checks the cut motor homing switch multiple times and recalibrates if detected.
// Returns true if home detected and recalibrated, false otherwise.
bool checkAndRecalibrateCutMotorHome(int attempts) {
    if (!cutMotor) return false;

    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); Serial.print(i + 1); Serial.print(": "); //serial.println(cutHomingSwitch.read());
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0);
            //serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
            break;
        }
    }
    return sensorDetectedHome;
}

//* ************************************************************************
//* ************************* SWITCH LOGIC FUNCTIONS ***********************
//* ************************************************************************

void handleReloadMode() {
    if (currentState == IDLE) {
        bool reloadSwitchOn = reloadSwitch.read() == HIGH;
        if (reloadSwitchOn && !isReloadMode) {
            isReloadMode = true;
            retractFeedClamp();
            retract2x4SecureClamp();
            turnYellowLedOn();
            //serial.println("Entered reload mode");
        } else if (!reloadSwitchOn && isReloadMode) {
            isReloadMode = false;
            extendFeedClamp();
            extend2x4SecureClamp();
            turnYellowLedOff();
            //serial.println("Exited reload mode, ready for operation");
        }
    }
}

void handleErrorAcknowledgement() {
    // This handles the general error acknowledgement via reloadSwitch
    // It was present in the main loop and also within the CUTTING state's homePositionErrorDetected block.
    if (reloadSwitch.rose() && (currentState == ERROR || currentState == CUTTING)) { // Check if in ERROR or if a cutting error is active
        // For CUTTING state, the homePositionErrorDetected flag logic needs to remain there,
        // but the transition to ERROR_RESET can be centralized if errorAcknowledged is set.
        if (currentState == ERROR) {
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Set flag, main loop will see this for ERROR state
            //serial.println("Error acknowledged by reload switch (from ERROR state). Transitioning to ERROR_RESET.");
        }
        // If in CUTTING, setting errorAcknowledged might be used by the CUTTING state to proceed.
        // The original CUTTING state logic directly transitioned. For now, we set the flag.
        // The calling code in CUTTING will need to check this flag if it relies on it.
        // For direct transition from specific cutting error, that logic is better kept in cutting stage.
        // This function primarily handles the generic ERROR state reset.
    }
}

void handleStartSwitchSafety() {
    // Original logic from setup() and main loop for startSwitchSafe
    // Call this once in setup() after startCycleSwitch.update()
    // And continuously in the main loop before checking shouldStartCycle()
    if (!startSwitchSafe && startCycleSwitch.fell()) {
        startSwitchSafe = true;
        //serial.println("Start switch is now safe to use (cycled OFF).");
    }
    // Initial check (typically for setup)
    // This part might be better directly in setup, but included here for completeness if called from there.
    // If called repeatedly from loop, this `else if` might be redundant if startSwitchSafe is managed correctly.
    /* else if (startCycleSwitch.read() == HIGH && !startSwitchSafe) {
        //serial.println("WARNING: Start switch is ON. Turn it OFF before operation.");
    }*/
}

void handleStartSwitchContinuousMode(){
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
        if (continuousModeActive) {
            //serial.println("Continuous operation mode activated");
        } else {
            //serial.println("Continuous operation mode deactivated");
        }
    }
}

//* ************************************************************************
//* ************************* STATE LOGIC HELPERS **************************
//* ************************************************************************

bool shouldStartCycle() {
    // Condition from IDLE state to start a cycle
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

// Rotation Servo Timing
void activateRotationServo() {
    // Activate rotation servo without sending TA signal
    if (!rotationServoIsActiveAndTiming) {
        Servo* servo = getRotationServo();
        if (servo) {
            // Force servo write with robust control - no attach checks, just send the command
            servo->write(ROTATION_SERVO_ACTIVE_POSITION);
            Serial.printf("FORCED Servo command sent: %d degrees (attach status ignored)\n", ROTATION_SERVO_ACTIVE_POSITION);
        }
        
        rotationServoActiveStartTime = millis();
        rotationServoIsActiveAndTiming = true;
        // Reset safety delay flag for new activation cycle
        setRotationServoSafetyDelayActive(false);
        Serial.print("Rotation servo activated to ");
        Serial.print(ROTATION_SERVO_ACTIVE_POSITION);
        Serial.println(" degrees.");
    } else {
        Serial.println("Rotation servo already active - skipping activation.");
    }
}

void handleRotationServoReturn() {
    // Move rotation servo to home position
    Servo* servo = getRotationServo();
    if (servo) {
        // Force servo write with robust control - no attach checks, just send the command
        servo->write(ROTATION_SERVO_HOME_POSITION);
        Serial.printf("FORCED Servo command sent: %d degrees (attach status ignored)\n", ROTATION_SERVO_HOME_POSITION);
    }
    
    Serial.print("Rotation servo returned to home position (");
    Serial.print(ROTATION_SERVO_HOME_POSITION);
    Serial.println(" degrees).");
}

void handleTASignalTiming() { 
  if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
    signalTAActive = false;
    //serial.println("Signal to Transfer Arm (TA) completed"); 
  }
}

void handleRotationClampRetract() {
    if (rotationClampIsExtended && (millis() - rotationClampExtendTime >= ROTATION_CLAMP_EXTEND_DURATION_MS)) {
        retractRotationClamp();
        //serial.println("Rotation Clamp retracted after 1 second.");
    }
}

void moveFeedMotorToPostCutHome() {
    if (feedMotor) {
        feedMotor->moveTo(0);
        //serial.println("Feed motor moving to post-cut home position (0 inches)");
    }
} 