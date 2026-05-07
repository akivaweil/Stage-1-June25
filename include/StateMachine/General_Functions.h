#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H

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
    STARTUP,
    HOMING,
    IDLE,
    CUTTING,
    ERROR,
    ERROR_RESET,
    SUCTION_ERROR,
    Cut_Motor_Homing_Error,
    YESWOOD,
    NOWOOD,
    FEED_FIRST_CUT,
    FEED_WOOD_FWD_ONE,
    RELOAD
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

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
void sendSignalToTA();

//* ************************************************************************
//* ************************* CLAMP FUNCTIONS ******************************
//* ************************************************************************
void extendFeedClamp();
void retractFeedClamp();
void extendTopClamp();
void retractTopClamp();
void extendRotationClamp();
void retractRotationClamp();

//* ************************************************************************
//* *************************** LED FUNCTIONS ******************************
//* ************************************************************************
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

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************
void configureCutMotorForCutting();
void configureCutMotorForCuttingSlow();
void configureCutMotorForReturn();
void configureFeedMotorForNormalOperation();
void configureFeedMotorForReturn();
void configureFeedMotorForSlowOperation(float speedMultiplier);
void moveCutMotorToCut();
void moveCutMotorToHome();
void moveFeedMotorToTravel();
// NOTE: feed-motor coordinate 0 is the pulled-back / load end. Physical
// "home" (the home sensor) is at coordinate -FEED_TRAVEL_DISTANCE.
// Convention: NEGATIVE coordinates are toward the home sensor; POSITIVE
// coordinates are away from it (toward the load end).
void moveFeedMotorToZero();
void moveFeedMotorToPosition(float targetPositionInches);
void stopCutMotor();
void stopFeedMotor();
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
void homeFeedMotorBlocking(Bounce& homingSwitch);
bool homeFeedMotorNonBlocking(Bounce& homingSwitch);
void moveFeedMotorToInitialAfterHoming();
bool checkAndRecalibrateCutMotorHome(int attempts);

//* ************************************************************************
//* ************************* SWITCH LOGIC FUNCTIONS ***********************
//* ************************************************************************
void handleErrorAcknowledgement();
void handleStartSwitchSafety();
void handleStartSwitchContinuousMode();

//* ************************************************************************
//* ************************* STATE LOGIC HELPERS **************************
//* ************************************************************************
bool shouldStartCycle();
void activateRotationServo();
void returnRotationServoHome();
void handleTASignalTiming();
void moveFeedMotorToPostCutZero();

//* ************************************************************************
//* ************************* FLAG MANAGEMENT FUNCTIONS ********************
//* ************************************************************************
bool getComingFromNoWoodWithSensorsClear();
void setComingFromNoWoodWithSensorsClear(bool value);

#endif // GENERAL_FUNCTIONS_H 