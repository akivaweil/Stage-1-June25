#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include "Config/Pins_Definitions.h"
#include "Config/Config.h"
#include "OTAUpdater/ota_updater.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "ErrorStates/Errors_Functions.h"
#include "StateMachine/StateManager.h"

//* ************************************************************************
//* ************************ AUTOMATED TABLE SAW **************************
//* ************************************************************************
// Main control system for Stage 1 of the automated table saw.
// Handles state machine logic, motor control, sensor monitoring, and safety systems.

// Pin definitions and configuration constants are now in Config/ header files

// Timing variables (constants moved to Config/system_config.h)
unsigned long rotationServoActiveStartTime = 0;
bool rotationServoIsActiveAndTiming = false;

// Rotation servo safety delay variables
bool rotationServoSafetyDelayActive = false;
unsigned long rotationServoSafetyDelayStartTime = 0;

unsigned long rotationClampExtendTime = 0;
bool rotationClampIsExtended = false;

// SystemStates Enum is now in Functions.h
SystemState currentState = STARTUP;
SystemState previousState = ERROR_RESET; // Initialize to a different state to ensure first print

// Motor configuration constants moved to Config/system_config.h

// Speed and acceleration settings moved to Config/system_config.h

// Create motor objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *cutMotor = NULL;
FastAccelStepper *feedMotor = NULL;

// Create servo object
Servo rotationServo;

// Bounce objects for debouncing switches
Bounce cutHomingSwitch = Bounce();
Bounce feedHomingSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce startCycleSwitch = Bounce();
Bounce pushwoodForwardSwitch = Bounce();
Bounce suctionSensorBounce = Bounce();

// System flags
bool isHomed = false;
bool isReloadMode = false;
bool _2x4Present = false;
bool woodSuctionError = false;
bool errorAcknowledged = false;
bool cuttingCycleInProgress = false;
bool continuousModeActive = false;  // New flag for continuous operation
bool startSwitchSafe = false;       // New flag to track if start switch is safe

// Timers for various operations
unsigned long lastBlinkTime = 0;
unsigned long lastErrorBlinkTime = 0;
unsigned long errorStartTime = 0;
unsigned long feedMoveStartTime = 0;

// LED states
bool blinkState = false;
bool errorBlinkState = false;

// Global variables for signal handling
unsigned long signalTAStartTime = 0; // For Transfer Arm signal
bool signalTAActive = false;      // For Transfer Arm signal

// New flag to track cut motor return during RETURNING_YES_2x4 mode
bool cutMotorInReturningYes2x4Return = false;

// WiFi credentials are defined in ota_updater.cpp
extern const char* ssid;
extern const char* password;

// Additional variables needed by states - declarations moved to above

// FIX_POSITION state steps now defined in fix_position.cpp

// StateManager instance is created in StateManager.cpp

// Forward declarations for functions
void initOTA();

// Motor initialization functions
void initCutMotor() {
    // Initialize cut motor - minimal implementation
    if (cutMotor) {
        cutMotor->setSpeedInHz(1000);
        cutMotor->setAcceleration(1000);
    }
}

void initFeedMotor() {
    // Initialize feed motor - minimal implementation
    if (feedMotor) {
        feedMotor->setSpeedInHz(1000);
        feedMotor->setAcceleration(1000);
    }
}

void initStateMachine() {
    // Initialize state machine - minimal implementation
    // State machine will start in STARTUP state
}

void executeCurrentState() {
    // Execute current state using StateManager
    executeStateMachine();
}

void setup() {
  // Initialize WiFi
  WiFi.begin(ssid, password);
  
  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
  }
  
  // Initialize OTA updater
  initOTA();
  
  // Initialize motors
  initCutMotor();
  initFeedMotor();
  
  // Initialize servo
  //rotationServo.attach(ROTATION_SERVO_PIN);
  //rotationServo.write(ROTATION_SERVO_HOME_POSITION);
  
  // Initialize state machine
  initStateMachine();
  
  // Set initial state
  changeState(STARTUP);
}

void loop() {
  // Execute current state
  executeCurrentState();
  
  // Handle OTA updates
  handleOTA();
  
  // Small delay to prevent watchdog issues
  delay(10);
}
