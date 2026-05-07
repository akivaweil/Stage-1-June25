#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include <esp_system.h>
#include <esp_attr.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Config/Pins.h"
#include "Config/Config.h"
#include "OTAUpdater/ota_updater.h"
#include "StateMachine/General_Functions.h"
#include "StateMachine/ErrorHandlers.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/03_CUTTING.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//* ************************************************************************
//* ************************ AUTOMATED TABLE SAW **************************
//* ************************************************************************
// Main control system for Stage 1 of the automated table saw.
// Handles state machine logic, motor control, sensor monitoring, and safety systems.

// Pin definitions and configuration constants are now in Config/ header files

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 💥 CRASH DIAGNOSTICS (RTC BREADCRUMBS)                              ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// RTC_NOINIT_ATTR variables persist across software resets / panics / WDTs
// (but not across power cycles or hard resets). On boot, esp_reset_reason() +
// these breadcrumbs tell us exactly what the firmware was doing when it died.

#define CRASH_BREADCRUMB_MAGIC 0xC0FFEE42

RTC_NOINIT_ATTR uint32_t crashBreadcrumbMagic;
RTC_NOINIT_ATTR uint32_t crashBreadcrumbState;
RTC_NOINIT_ATTR uint32_t crashBreadcrumbCuttingStep;
RTC_NOINIT_ATTR uint32_t crashBreadcrumbLastAliveMs;
RTC_NOINIT_ATTR uint32_t crashBreadcrumbCount;

// Captured-at-boot snapshot of last-run breadcrumbs, exposed to dashboard.
String lastResetReasonStr = "UNKNOWN";
String lastCrashStateStr = "UNKNOWN";
int lastCrashCuttingStep = -1;
unsigned long lastCrashUptimeMs = 0;
unsigned long crashCountSincePower = 0;
bool lastResetWasAbnormal = false;

// Timing variables (constants moved to Config/system_config.h)
unsigned long rotationServoActiveStartTime = 0;
bool rotationServoActive = false;
// Starts false on boot so the first cut always waits for the servo to reach home.
// Flips true only after ROTATION_SERVO_HOME_WAIT_DURATION_MS has elapsed since
// the most recent returnRotationServoHome() command — never set instantly.
// activateRotationServo() clears it when the servo moves back to ACTIVE.
bool rotationServoKnownHome = false;
// True from the moment a home command is issued until the travel buffer elapses
// and the known-home flag flips true. Ensures any flag-true transition always
// carries the full physical-travel buffer.
bool rotationServoHomePending = false;
unsigned long rotationServoHomeCommandTime = 0;

// Rotation servo return delay variables
bool rotationServoReturnDelayActive = false;
unsigned long rotationServoReturnDelayStartTime = 0;
bool rotationServoReturnCompleted = false; // Flag to prevent multiple calls to returnRotationServoHome()

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
Bounce woodPresentSensorBounce = Bounce();

// System flags
bool isHomed = false;
bool isReloadMode = false;
bool _2x4Present = false;
bool woodSuctionError = false;
bool errorAcknowledged = false;
bool cuttingCycleInProgress = false;
bool continuousModeActive = false;  // New flag for continuous operation
bool startSwitchSafe = false;       // New flag to track if start switch is safe
bool comingFromNoWoodWithSensorsClear = false; // Flag to track when coming from no-wood cycle with sensors clear
bool dashboardStartCycleTrigger = false; // Flag to trigger a start cycle from the dashboard

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
bool taSignalActive = false;      // For Transfer Arm signal
unsigned long taSignalOffTime = 0; // millis() when TA signal last went LOW (used to gate OTA)

// New flag to track cut motor return during YESWOOD mode
bool cutMotorInYeswoodReturn = false;

// Additional variables needed by states - declarations moved to above

// FIX_POSITION state steps now defined in fix_position.cpp

// StateManager instance is created in StateManager.cpp

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 💥 CRASH DIAGNOSTIC HELPERS                                          ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

static String resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWER_ON";
    case ESP_RST_EXT:       return "EXT_PIN";
    case ESP_RST_SW:        return "SW_RESTART";
    case ESP_RST_PANIC:     return "PANIC (exception)";
    case ESP_RST_INT_WDT:   return "INT_WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
    case ESP_RST_WDT:       return "OTHER_WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP_WAKE";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static String stateIdToName(uint32_t s) {
  switch ((SystemState)s) {
    case STARTUP:                return "STARTUP";
    case HOMING:                 return "HOMING";
    case IDLE:                   return "IDLE";
    case FEED_FIRST_CUT:         return "FEED_FIRST_CUT";
    case FEED_WOOD_FWD_ONE:      return "FEED_WOOD_FWD_ONE";
    case CUTTING:                return "CUTTING";
    case YESWOOD:      return "YESWOOD";
    case NOWOOD:       return "NOWOOD";
    case RELOAD:                 return "RELOAD";
    case ERROR:                  return "ERROR";
    case ERROR_RESET:            return "ERROR_RESET";
    case SUCTION_ERROR:          return "SUCTION_ERROR";
    case Cut_Motor_Homing_Error: return "CUT_MOTOR_HOMING_ERROR";
    default:                     return "UNKNOWN";
  }
}

static void captureCrashDiagnostics() {
  esp_reset_reason_t reason = esp_reset_reason();
  lastResetReasonStr = resetReasonToString(reason);

  bool magicValid = (crashBreadcrumbMagic == CRASH_BREADCRUMB_MAGIC);
  bool abnormal = (reason == ESP_RST_PANIC ||
                   reason == ESP_RST_INT_WDT ||
                   reason == ESP_RST_TASK_WDT ||
                   reason == ESP_RST_WDT ||
                   reason == ESP_RST_BROWNOUT);

  if (magicValid) {
    lastCrashStateStr     = stateIdToName(crashBreadcrumbState);
    lastCrashCuttingStep  = (int)crashBreadcrumbCuttingStep;
    lastCrashUptimeMs     = crashBreadcrumbLastAliveMs;
    crashCountSincePower  = crashBreadcrumbCount;
  } else {
    lastCrashStateStr     = "N/A";
    lastCrashCuttingStep  = -1;
    lastCrashUptimeMs     = 0;
    crashCountSincePower  = 0;
    crashBreadcrumbCount  = 0;
  }

  lastResetWasAbnormal = abnormal && magicValid;
  if (lastResetWasAbnormal) {
    crashBreadcrumbCount++;
    crashCountSincePower = crashBreadcrumbCount;
  }

  crashBreadcrumbMagic       = CRASH_BREADCRUMB_MAGIC;
  crashBreadcrumbState       = (uint32_t)STARTUP;
  crashBreadcrumbCuttingStep = 0xFFFFFFFF;
  crashBreadcrumbLastAliveMs = 0;

  Serial.print("Reset reason: ");
  Serial.println(lastResetReasonStr);
  if (lastResetWasAbnormal) {
    Serial.print("Last alive in state: ");
    Serial.print(lastCrashStateStr);
    Serial.print(" (cutting step ");
    Serial.print(lastCrashCuttingStep);
    Serial.print(") at uptime ");
    Serial.print(lastCrashUptimeMs);
    Serial.println(" ms");
  }
}

static inline void updateCrashBreadcrumbs() {
  crashBreadcrumbState       = (uint32_t)currentState;
  crashBreadcrumbCuttingStep = (currentState == CUTTING) ? (uint32_t)getCuttingStateStep() : 0xFFFFFFFF;
  crashBreadcrumbLastAliveMs = millis();
}

void setup() {
  // Disable brownout detector. TA signal trigger causes a brief 3.3V dip
  // from the shared external load that was rebooting the ESP. Hardware fix
  // (separate regulator / bulk cap) still recommended.
  // Note: ESP32-S3 brownout threshold is set at build time via sdkconfig,
  // not exposed at runtime in Arduino SDK, so it's effectively all-or-nothing
  // here. To re-enable at lowest sensitivity, set CONFIG_ESP_BROWNOUT_DET_LVL=7
  // (lowest voltage threshold) in build_flags and remove this WRITE_PERI_REG.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("Automated Table Saw Control System - Stage 1");

  captureCrashDiagnostics();
  
  setupOTA();
  
  //! Setup websocket dashboard
  setupWebSocketDashboard();

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
  pinMode(TOP_CLAMP, OUTPUT);
  
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_YELLOW, OUTPUT);
  pinMode(STATUS_LED_GREEN, OUTPUT);
  pinMode(STATUS_LED_BLUE, OUTPUT);
  
  pinMode(TRANSFER_ARM_SIGNAL_PIN, OUTPUT);
  digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
  
  //! Initialize clamps and LEDs
  extendFeedClamp();
  extendTopClamp();
  retractRotationClamp();
  allLedsOff();
  showBlueLed();
  
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
  
  woodPresentSensorBounce.attach(_2x4_PRESENT_SENSOR);
  woodPresentSensorBounce.interval(10);
  
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
  
  // SAFETY: Do NOT set initial servo position during startup
  // This prevents the servo from moving and potentially ramming stuck wood into the blade
  // The servo will only be positioned when manually starting a cut cycle
  //Serial.println("Servo initialization complete - no initial position set for safety");
  
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
  updateCrashBreadcrumbs();

  // Handle OTA requests when in IDLE, HOMING, RELOAD states, or at the beginning of CUTTING state (step 0)
  bool allowOTA = (currentState == IDLE || currentState == HOMING || currentState == RELOAD);

  // Also allow OTA at the beginning of cutting state (step 0 only)
  if (currentState == CUTTING) {
    allowOTA = isCuttingStateStep0();
  }

  // Require the TA signal to have been LOW for at least 500ms before allowing OTA.
  // OTA stalls block the TransferArm timing handler and can leave the signal stuck HIGH.
  static const unsigned long OTA_TA_SIGNAL_OFF_GUARD_MS = 500;
  if (taSignalActive || millis() - taSignalOffTime < OTA_TA_SIGNAL_OFF_GUARD_MS) {
    allowOTA = false;
  }

  if (allowOTA) {
    handleOTA();
  }

  // Execute the state machine - all the logic below has been moved to function-based state management
  executeStateMachine();
  
  // Update dashboard status periodically (only when not cutting to avoid timing interference)
  static unsigned long lastDashboardUpdate = 0;
  if (millis() - lastDashboardUpdate > 1000) { // Update every second
    updateDashboardStatus();
    lastDashboardUpdate = millis();
  }
}
