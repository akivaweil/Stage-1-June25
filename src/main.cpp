#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h>
#include <ESP32Servo.h>
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

// Additional variables needed by states - declarations moved to above

// FIX_POSITION state steps now defined in fix_position.cpp

// StateManager instance is created in StateManager.cpp

void setup() {
  Serial.begin(115200);
  //serial.println("Automated Table Saw Control System - Stage 1");
  
  setupOTA();

  //! Configure pin modes
  pinMode(CUT_MOTOR_STEP_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(FEED_MOTOR_STEP_PIN, OUTPUT);
  pinMode(FEED_MOTOR_DIR_PIN, OUTPUT);
  
  pinMode(CUT_MOTOR_HOME_SWITCH, INPUT_PULLDOWN);
  pinMode(FEED_MOTOR_HOME_SENSOR, INPUT_PULLUP);
  pinMode(RELOAD_SWITCH, INPUT_PULLDOWN);
  pinMode(START_CYCLE_SWITCH, INPUT_PULLDOWN);
  pinMode(MANUAL_FEED_SWITCH, INPUT_PULLDOWN);
  
  pinMode(_2x4_PRESENT_SENSOR, INPUT_PULLUP);
  pinMode(WOOD_SUCTION_CONFIRM_SENSOR, INPUT_PULLUP);
  
  pinMode(ROTATION_CLAMP, OUTPUT);
  pinMode(FEED_CLAMP, OUTPUT);
  pinMode(_2x4_SECURE_CLAMP, OUTPUT);
  
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_YELLOW, OUTPUT);
  pinMode(STATUS_LED_GREEN, OUTPUT);
  pinMode(STATUS_LED_BLUE, OUTPUT);
  
  pinMode(TRANSFER_ARM_SIGNAL_PIN, OUTPUT);
  digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
  
  //! Initialize clamps and LEDs
  extendFeedClamp();
  extend2x4SecureClamp();
  retractRotationClamp();
  allLedsOff();
  turnBlueLedOn();
  
  //! Configure switch debouncing
  cutHomingSwitch.attach(CUT_MOTOR_HOME_SWITCH);
  cutHomingSwitch.interval(3);
  
  feedHomingSwitch.attach(FEED_MOTOR_HOME_SENSOR);
  feedHomingSwitch.interval(5);
  
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(10);
  
  startCycleSwitch.attach(START_CYCLE_SWITCH);
  startCycleSwitch.interval(20);
  
  pushwoodForwardSwitch.attach(MANUAL_FEED_SWITCH);
  pushwoodForwardSwitch.interval(20);
  
  suctionSensorBounce.attach(WOOD_SUCTION_CONFIRM_SENSOR);
  suctionSensorBounce.interval(15);
  
  //! Initialize motors
  engine.init();

  cutMotor = engine.stepperConnectToPin(CUT_MOTOR_STEP_PIN);
  if (cutMotor) {
    cutMotor->setDirectionPin(CUT_MOTOR_DIR_PIN);
    configureCutMotorForCutting();
    cutMotor->setCurrentPosition(0);
  } else {
    //serial.println("Failed to init cutMotor");
  }

  feedMotor = engine.stepperConnectToPin(FEED_MOTOR_STEP_PIN);
  if (feedMotor) {
    feedMotor->setDirectionPin(FEED_MOTOR_DIR_PIN);
    configureFeedMotorForNormalOperation();
    feedMotor->setCurrentPosition(0);
  } else {
    //serial.println("Failed to init feedMotor");
  }
  
  //! Initialize servo with robust attachment
  //Serial.printf("Initializing servo on pin %d with robust attachment\n", ROTATION_SERVO_PIN);
  
  // Force servo attachment using multiple methods to ensure proper initialization
  rotationServo.attach(ROTATION_SERVO_PIN);
  rotationServo.attach(ROTATION_SERVO_PIN, 500, 2500);
  rotationServo.attach(ROTATION_SERVO_PIN, 1000, 2000);
  rotationServo.attach(ROTATION_SERVO_PIN, 544, 2400);  // Standard servo range
  
  // Final forced attach
  rotationServo.attach(ROTATION_SERVO_PIN);
  
  //Serial.println("✓ Servo attachment completed - Commands will be sent regardless of attach status");
  
  // Set initial servo position to home with forced write
  rotationServo.write(ROTATION_SERVO_HOME_POSITION);
  //Serial.printf("Servo initialized and set to home position: %d degrees\n", ROTATION_SERVO_HOME_POSITION);
  
  //! Configure initial state
  currentState = STARTUP;
  
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
  } else {
    startSwitchSafe = true;
  }
  
  delay(10);
}

void loop() {
  handleOTA(); // Handle OTA requests

  // Execute the state machine - all the logic below has been moved to function-based state management
  executeStateMachine();
}
