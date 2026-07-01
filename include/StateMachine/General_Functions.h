#pragma once

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include "Config/Config.h"
// #include <ESP32Servo.h> // Removed - using function-based PWM control instead

// Forward declarations and external variable references
extern bool blinkState;
extern bool errorBlinkState;
extern unsigned long lastErrorBlinkTime;
extern bool taSignalActive;
extern unsigned long signalTAStartTime;
extern unsigned long taSignalDelayStartTime; // For TA signal delay
extern bool taSignalDelayPending;             // For TA signal delay
extern bool rotationServoActive;
extern bool rotationServoKnownHome;          // true once a home command has had time to complete
extern bool rotationServoHomePending;        // return commanded, waiting for travel buffer to elapse
extern unsigned long rotationServoHomeCommandTime;
extern unsigned long rotationServoActiveStartTime;
extern bool rotationServoReturnCompleted; // Flag to prevent multiple calls to returnRotationServoHome()
void updateRotationServoHomeStatus();        // flips rotationServoKnownHome once the pending buffer elapses
extern bool rotationClampIsExtended;
extern unsigned long rotationClampExtendTime;
extern bool isReloadMode;
extern bool errorAcknowledged;
extern bool startSwitchSafe;
extern bool continuousModeActive;
extern bool cuttingCycleInProgress;
extern bool woodSuctionError;
extern bool comingFromNoWoodWithSensorsClear;
extern bool dashboardStartCycleTrigger; // Flag to trigger a start cycle from the dashboard

// System state enum
enum SystemState {
    STATE_STARTUP,
    STATE_HOMING,
    STATE_IDLE,
    STATE_CUTTING,
    STATE_ERROR,
    STATE_ERROR_RESET,
    STATE_SUCTION_ERROR,
    STATE_CUT_MOTOR_HOMING_ERROR,
    STATE_YESWOOD,
    STATE_NOWOOD,
    STATE_FEED_FIRST_CUT,
    STATE_FEED_WOOD_FWD_ONE,
    STATE_RELOAD
};

extern SystemState currentState;

// Motor objects (servo now uses function-based PWM control)
extern const int ROTATION_SERVO_PWM_CHANNEL;
extern FastAccelStepper* cutMotor;
extern FastAccelStepper* feedMotor;

// Switch objects
extern Bounce reloadSwitch;
extern Bounce startCycleSwitch;
extern Bounce cutHomingSwitch;
extern Bounce feedHomingSwitch;
extern Bounce pushwoodForwardSwitch;

// Additional system flags
extern bool _2x4Present;
extern unsigned long lastBlinkTime;
extern unsigned long errorStartTime;

// Pin definitions and constants
extern const int TRANSFER_ARM_SIGNAL_PIN;
extern const int FEED_CLAMP;
extern const int TOP_CLAMP;
extern const int ROTATION_CLAMP;
extern const int STATUS_LED_RED;
extern const int STATUS_LED_YELLOW;
extern const int STATUS_LED_GREEN;
extern const int STATUS_LED_BLUE;
// Configuration constants live in Config.h

// Signaling Functions
void sendSignalToTA();

// Cut-Rate Stats (rolling cuts-per-minute over 1/3/5/15 min windows)
void recordCut();
void getCutRates(float& perMin1, float& perMin3, float& perMin5, float& perMin15);

// Clamp Functions
void extendFeedClamp();
void retractFeedClamp();
void extendTopClamp();
void retractTopClamp();
void extendRotationClamp();
void retractRotationClamp();

// LED Functions
void showRedLed();
void turnRedLedOff();
void showYellowLed();
void turnYellowLedOff();
void showGreenLed();
void turnGreenLedOff();
void showBlueLed();
void turnBlueLedOff();
void allLedsOff();
void handleHomingLedBlink();
void handleNoWoodLedWavePattern(float speedMultiplier = 1.0f);
void resetNoWoodLedWavePattern(bool preserveYellowLed = false);

// Motor Control Functions
void configureCutMotorForCutting();
void configureCutMotorForCuttingSlow();
void configureCutMotorForReturn();
void configureFeedMotorForNormalOperation();
void configureFeedMotorForReturn();
void configureFeedMotorForSlowOperation(float speedMultiplier);
void moveCutMotorToCut();
void moveCutMotorToHome();
// NOTE: feed-motor coordinate 0 is the pulled-back / load end. Physical
// "home" (the home sensor) is at coordinate FEED_TRAVEL_DISTANCE.
void moveFeedMotorToZero();
void moveFeedMotorToPosition(float targetPositionInches);
void stopCutMotor();
void stopFeedMotor();
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
bool homeFeedMotorNonBlocking(Bounce& homingSwitch);
void moveFeedMotorToInitialAfterHoming();
bool checkAndRecalibrateCutMotorHome(int attempts);

// State Logic Helpers
bool shouldStartCycle();
void activateRotationServo();
void returnRotationServoHome();
void handleTASignalTiming();

// Flag Management Functions
bool getComingFromNoWoodWithSensorsClear();
void setComingFromNoWoodWithSensorsClear(bool value);
