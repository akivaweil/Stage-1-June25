// IMPORTANT NOTE: This file contains general helper functions used throughout the system.
// It relies on the main file for pin definitions and global variable declarations (via extern).
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "StateMachine/General_Functions.h"
#include "StateMachine/StateManager.h"
#include "Config/Config.h"

// External motor object references from main.cpp
extern FastAccelStepper* cutMotor;
extern FastAccelStepper* feedMotor;
extern Bounce cutHomingSwitch;

// Configuration Constants
const unsigned long FEED_HOME_TIMEOUT = 30000; // 30 seconds timeout

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
// Contains functions related to signaling other stages or components.

// Global non-blocking delay variables for TA signal
unsigned long taSignalDelayStartTime = 0;
bool taSignalDelayActive = false;

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
      if (!taSignalDelayActive) {
          taSignalDelayStartTime = millis();
          taSignalDelayActive = true;
      }
  } else {
      // 3 Inch mode: Execute immediately
      digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
      signalTAStartTime = millis();
      signalTAActive = true;
      taSignalDelayActive = false;
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

void showRedLed() {
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

void showYellowLed() {
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

void showGreenLed() {
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

void showBlueLed() {
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

// LED wave pattern timing constants (adjust these to change animation speed)
const unsigned long LED_WAVE_INTERVAL_MS = 200;    // Time between each LED starting (256ms = 1/4 second)
const unsigned long LED_ON_DURATION_MS = 250;      // How long each LED stays on (256ms = 1/4 second)

// Static variables for LED wave pattern (shared between states)
static unsigned long ledWaveOnTime[4] = {0, 0, 0, 0}; // When each LED turned on (0 = off)
static unsigned long ledWaveTurnOnTime[4] = {0, 0, 0, 0}; // When each LED should next turn on
static bool ledWaveInitialized = false;

void handleNoWoodLedWavePattern(float speedMultiplier) {
    // Scale timing constants by speedMultiplier (>1 = slower)
    unsigned long interval = (unsigned long)(LED_WAVE_INTERVAL_MS * speedMultiplier);
    unsigned long onDuration = (unsigned long)(LED_ON_DURATION_MS * speedMultiplier);

    // Initialize wave pattern if not already done
    if (!ledWaveInitialized) {
        unsigned long now = millis();
        ledWaveTurnOnTime[0] = now; // Red starts immediately
        ledWaveTurnOnTime[1] = now + interval; // Yellow starts after red
        ledWaveTurnOnTime[2] = now + (interval * 2); // Green starts after yellow
        ledWaveTurnOnTime[3] = now + (interval * 3); // Blue starts after green
        ledWaveOnTime[0] = 0;
        ledWaveOnTime[1] = 0;
        ledWaveOnTime[2] = 0;
        ledWaveOnTime[3] = 0;
        ledWaveInitialized = true;
    }

    // Handle LED wave pattern: sequential red -> yellow -> green -> blue, max 2 on at once
    unsigned long now = millis();
    for (int i = 0; i < 4; i++) {
        // Check if LED should turn on
        if (now >= ledWaveTurnOnTime[i] && ledWaveOnTime[i] == 0) {
            // Turn on this LED
            switch (i) {
                case 0: digitalWrite(STATUS_LED_RED, HIGH); break;
                case 1: digitalWrite(STATUS_LED_YELLOW, HIGH); break;
                case 2: digitalWrite(STATUS_LED_GREEN, HIGH); break;
                case 3: digitalWrite(STATUS_LED_BLUE, HIGH); break;
            }
            // Record when this LED turned on
            ledWaveOnTime[i] = now;
            // Schedule next turn on (continuous wave)
            ledWaveTurnOnTime[i] = now + onDuration + (interval * 3);
        }

        // Check if LED should turn off (after being on for onDuration)
        if (ledWaveOnTime[i] > 0 && now - ledWaveOnTime[i] >= onDuration) {
            // Turn off this LED
            switch (i) {
                case 0: digitalWrite(STATUS_LED_RED, LOW); break;
                case 1: digitalWrite(STATUS_LED_YELLOW, LOW); break;
                case 2: digitalWrite(STATUS_LED_GREEN, LOW); break;
                case 3: digitalWrite(STATUS_LED_BLUE, LOW); break;
            }
            // Reset on time (waiting for next turn on)
            ledWaveOnTime[i] = 0;
        }
    }
}

void resetNoWoodLedWavePattern(bool preserveYellowLed) {
    ledWaveInitialized = false;
    // Turn off all LEDs
    digitalWrite(STATUS_LED_RED, LOW);
    if (!preserveYellowLed) {
        digitalWrite(STATUS_LED_YELLOW, LOW);
    }
    digitalWrite(STATUS_LED_GREEN, LOW);
    digitalWrite(STATUS_LED_BLUE, LOW);
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
        if (blinkState) showBlueLed(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************

void configureCutMotorForCutting() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_NORMAL_SPEED * CUT_MOTOR_STEPS_PER_INCH));
        cutMotor->setAcceleration((uint32_t)(CUT_MOTOR_NORMAL_ACCELERATION * CUT_MOTOR_STEPS_PER_INCH));
    }
}

void configureCutMotorForCuttingSlow() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_NO_WOOD_SPEED * CUT_MOTOR_STEPS_PER_INCH));
        cutMotor->setAcceleration((uint32_t)(CUT_MOTOR_NORMAL_ACCELERATION * CUT_MOTOR_STEPS_PER_INCH));
    }
}

void configureCutMotorForReturn() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_RETURN_SPEED * CUT_MOTOR_STEPS_PER_INCH));
        cutMotor->setAcceleration((uint32_t)(CUT_MOTOR_NORMAL_ACCELERATION * CUT_MOTOR_STEPS_PER_INCH));
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

void configureFeedMotorForSlowOperation(float speedMultiplier) {
    if (feedMotor) {
        // Apply speed multiplier to normal speed and acceleration
        feedMotor->setSpeedInHz((uint32_t)(FEED_MOTOR_NORMAL_SPEED * speedMultiplier));
        feedMotor->setAcceleration((uint32_t)(FEED_MOTOR_NORMAL_ACCELERATION * speedMultiplier));
    }
}

void moveCutMotorToCut() {
    if (cutMotor) {
        extern float getCutTravelDistance();
        cutMotor->moveTo(getCutTravelDistance() * CUT_MOTOR_STEPS_PER_INCH);
    }
}

void moveCutMotorToHome() {
    if (cutMotor) {
        cutMotor->moveTo(-0.02 * CUT_MOTOR_STEPS_PER_INCH); // Minimal overshoot
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
    //serial.print("Initial switch state: ");
    //serial.println(homingSwitch.read() == HIGH ? "HIGH" : "LOW");
    
    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)(CUT_MOTOR_HOMING_SPEED * CUT_MOTOR_STEPS_PER_INCH));
    cutMotor->moveTo(-40000);
    
    //serial.print("Cut motor homing speed set to: ");
    //serial.println(CUT_MOTOR_HOMING_SPEED);
    //serial.println("Cut motor moving to -40000 steps...");

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        
        // Periodic status updates removed to reduce serial output
        
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
    //serial.print("Initial feed sensor state: ");
    //serial.println(homingSwitch.read() == LOW ? "ACTIVE" : "INACTIVE");
    
    // Debug motor setup
    //serial.print("FEED_MOTOR_STEPS_PER_INCH value: ");
    //serial.println(FEED_MOTOR_STEPS_PER_INCH);
    //serial.print("FEED_MOTOR_HOMING_SPEED value: ");
    //serial.println(FEED_MOTOR_HOMING_SPEED);
    
    // Step 1: Move toward home sensor until it triggers
    feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
    
    // Try using runForward() instead of moveTo() for more reliable operation
    //serial.println("Starting feed motor forward run...");
    feedMotor->runForward();
    
    // Verify motor started
    delay(100); // Small delay to let motor start
    //serial.print("Motor started - Running: ");
    //serial.print(feedMotor->isRunning() ? "YES" : "NO");
    //serial.print(", Position: ");
    //serial.println(feedMotor->getCurrentPosition());

    // Add timeout for feed motor homing
    unsigned long startTime = millis();

    while (homingSwitch.read() != LOW) {
        homingSwitch.update();
        
        // Periodic status updates removed to reduce serial output
        
        // If motor stopped running unexpectedly, restart it
        static unsigned long lastRestartCheck = 0;
        if (millis() - lastRestartCheck >= 1000) {
            if (!feedMotor->isRunning()) {
                //serial.println("Motor stopped unexpectedly! Restarting...");
                feedMotor->runForward();
            }
            lastRestartCheck = millis();
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

// Non-blocking feed motor homing state variables
static bool feedMotorHomingInProgress = false;
static int feedMotorHomingStep = 0;
static unsigned long feedMotorHomingStartTime = 0;
static unsigned long feedMotorHomingLastRestartCheck = 0;

// Non-blocking feed motor homing function
bool homeFeedMotorNonBlocking(Bounce& homingSwitch) {
    if (!feedMotor) {
        return false;
    }
    
    // Initialize homing if not already in progress
    if (!feedMotorHomingInProgress) {
        feedMotorHomingInProgress = true;
        feedMotorHomingStep = 0;
        feedMotorHomingStartTime = millis();
        feedMotorHomingLastRestartCheck = millis();
        
        // Step 1: Start moving toward home sensor
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
        feedMotor->runForward();
        feedMotorHomingStep = 1;
        return false; // Not complete yet
    }
    
    // Check for timeout
    if (millis() - feedMotorHomingStartTime > FEED_HOME_TIMEOUT) {
        feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
        feedMotorHomingInProgress = false;
        return false; // Failed due to timeout
    }
    
    // Step 1: Wait for home sensor to trigger
    if (feedMotorHomingStep == 1) {
        homingSwitch.update();
        
        // If motor stopped running unexpectedly, restart it
        if (millis() - feedMotorHomingLastRestartCheck >= 1000) {
            if (!feedMotor->isRunning()) {
                feedMotor->runForward();
            }
            feedMotorHomingLastRestartCheck = millis();
        }
        
        if (homingSwitch.read() == LOW) {
            // Home sensor detected - stop motor and move to step 2
            feedMotor->forceStopAndNewPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            feedMotorHomingStep = 2;
            return false; // Not complete yet
        }
    }
    
    // Step 2: Move to working position
    if (feedMotorHomingStep == 2) {
        if (!feedMotor->isRunning()) {
            // Move to working position (-0.5 inch from sensor)
            feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - FEED_MOTOR_OFFSET_FROM_SENSOR * FEED_MOTOR_STEPS_PER_INCH);
            feedMotorHomingStep = 3;
        }
        return false; // Not complete yet
    }
    
    // Step 3: Wait for positioning to complete
    if (feedMotorHomingStep == 3) {
        if (!feedMotor->isRunning()) {
            // Set working position as zero
            feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
            configureFeedMotorForNormalOperation();
            feedMotorHomingInProgress = false;
            return true; // Complete!
        }
        
        // Check for positioning timeout
        if (millis() - feedMotorHomingStartTime > FEED_HOME_TIMEOUT) {
            feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
            feedMotorHomingInProgress = false;
            return false; // Failed due to timeout
        }
    }
    
    return false; // Still in progress
}

void moveFeedMotorToInitialAfterHoming() {
    if (feedMotor) {
        configureFeedMotorForNormalOperation();
        moveFeedMotorToHome();
        // Removed blocking while loop - let the state machine handle this
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
            //Serial.printf("FORCED Servo command sent: %d degrees (attach status ignored)\n", ROTATION_SERVO_ACTIVE_POSITION);
        }
        
        rotationServoActiveStartTime = millis();
        rotationServoIsActiveAndTiming = true;
        // Reset the return completed flag for new activation cycle
        rotationServoReturnCompleted = false;
        //Serial.print("Rotation servo activated to ");
        //Serial.print(ROTATION_SERVO_ACTIVE_POSITION);
        //Serial.println(" degrees.");
    } else {
        //Serial.println("Rotation servo already active - skipping activation.");
    }
}

void handleRotationServoReturn() {
    // Move rotation servo to home position with fast return - send command multiple times rapidly
    Servo* servo = getRotationServo();
    if (servo) {
        // Send multiple rapid commands to ensure fast return to home
        for (int i = 0; i < 5; i++) {
            servo->write(ROTATION_SERVO_HOME_POSITION);
            delayMicroseconds(100); // Small delay between rapid writes
        }
        //Serial.printf("FORCED Servo command sent: %d degrees (attach status ignored)\n", ROTATION_SERVO_HOME_POSITION);
    }
    
    //Serial.print("Rotation servo returned to home position (");
    //Serial.print(ROTATION_SERVO_HOME_POSITION);
    //Serial.println(" degrees).");
}

// Function to handle Transfer Arm signal timing (including start delay)
void handleTASignalTiming() { 
  // Handle start delay if active
  if (taSignalDelayActive) {
      if (millis() - taSignalDelayStartTime >= 500) {
          digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
          signalTAStartTime = millis();
          signalTAActive = true;
          taSignalDelayActive = false;
      }
  }

  // Handle signal duration
  if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
    signalTAActive = false;
    //serial.println("Signal to Transfer Arm (TA) completed"); 
  }
}


void moveFeedMotorToPostCutHome() {
    if (feedMotor) {
        feedMotor->moveTo(0);
        //serial.println("Feed motor moving to post-cut home position (0 inches)");
    }
}

//* ************************************************************************
//* ************************* FLAG MANAGEMENT FUNCTIONS ********************
//* ************************************************************************

bool getComingFromNoWoodWithSensorsClear() {
    return comingFromNoWoodWithSensorsClear;
}

void setComingFromNoWoodWithSensorsClear(bool value) {
    comingFromNoWoodWithSensorsClear = value;
} 