#ifndef ERRORS_FUNCTIONS_H
#define ERRORS_FUNCTIONS_H

#include <Arduino.h>
#include <Bounce2.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>

// Include the SystemState enum definition
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* *********************** ERROR FUNCTION DECLARATIONS ********************
//* ************************************************************************

// Signaling Functions
void sendSignalToTA();

// Clamp Control Functions
void extendFeedClamp();
void retractFeedClamp();
void extend2x4SecureClamp();
void retract2x4SecureClamp();
void extendRotationClamp();
void retractRotationClamp();

// LED Control Functions
void turnRedLedOn();
void turnRedLedOff();
void turnYellowLedOn();
void turnYellowLedOff();
void turnGreenLedOn();
void turnGreenLedOff();
void turnBlueLedOn();
void turnBlueLedOff();
void allLedsOff();
void handleHomingLedBlink();
void handleErrorLedBlink();
void handleSuctionErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef);

// Motor Control Functions
void configureCutMotorForCutting();
void configureCutMotorForReturn();
void configureFeedMotorForNormalOperation();
void configureFeedMotorForReturn();
void moveCutMotorToCut();
void moveCutMotorToHome();
void moveFeedMotorToTravel();
void moveFeedMotorToHome();
void moveFeedMotorToPosition(float targetPositionInches);
void stopCutMotor();
void stopFeedMotor();
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout);
void homeFeedMotorBlocking(Bounce& homingSwitch);
void moveFeedMotorToInitialAfterHoming();
bool checkAndRecalibrateCutMotorHome(int attempts);
void moveFeedMotorToPostCutHome();

// Switch Logic Functions
void handleReloadMode();
void handleErrorAcknowledgement();
void handleStartSwitchSafety();
void handleStartSwitchContinuousMode();

// State Logic Helpers
bool shouldStartCycle();
void activateRotationServo();
void handleRotationServoReturn();
void handleTASignalTiming();
void handleRotationClampRetract();

// Cut Motor Error Functions
struct CutMotorHomeErrorResult {
    bool wasHomeDetected;
    bool shouldTransitionToError;
    bool shouldAttemptSlowRecovery;
    bool shouldContinueWithWarning;
    String errorMessage;
};

void performCutMotorRealTimeHomeSensorCheck(FastAccelStepper* cutMotor, Bounce& cutHomingSwitch, bool& cutMotorInYes2x4Return);
CutMotorHomeErrorResult handleCutMotorHomeError(Bounce& cutHomingSwitch, FastAccelStepper* cutMotor, const String& contextDescription, bool allowSlowRecovery);
void executeCutMotorErrorStateTransition(FastAccelStepper* cutMotor, FastAccelStepper* positionMotor, SystemState& currentState, int& cuttingStep, int& cuttingSubStep7, int& fixPositionStep, int& fixPositionSubStep2, unsigned long& errorStartTime, bool shouldExtend2x4SecureClamp);
void logCutMotorHomeErrorResult(const CutMotorHomeErrorResult& result);

// Result creation helpers
CutMotorHomeErrorResult createSuccessResult();
CutMotorHomeErrorResult createErrorTransitionResult(const String& message);
CutMotorHomeErrorResult createWarningOnlyResult(const String& message);

// External variable declarations (defined in main.cpp)
extern bool blinkState;
extern bool errorBlinkState;
extern unsigned long lastErrorBlinkTime;
extern bool signalTAActive;
extern unsigned long signalTAStartTime;
extern bool rotationServoIsActiveAndTiming;
extern unsigned long rotationServoActiveStartTime;
extern bool rotationClampIsExtended;
extern unsigned long rotationClampExtendTime;
extern bool isReloadMode;
extern bool errorAcknowledged;
extern bool startSwitchSafe;
extern bool continuousModeActive;
extern bool cuttingCycleInProgress;
extern bool woodSuctionError;

// Servo objects
extern Servo rotationServo;

// Switch objects
extern Bounce reloadSwitch;
extern Bounce startCycleSwitch;
extern Bounce cutHomingSwitch;

// Motor objects
extern FastAccelStepper* cutMotor;
extern FastAccelStepper* feedMotor;

// Pin definitions and constants (defined in main.cpp)
extern const int TRANSFER_ARM_SIGNAL_PIN;
extern const int FEED_CLAMP;
extern const int _2x4_SECURE_CLAMP;
extern const int ROTATION_CLAMP;
extern const int STATUS_LED_RED;
extern const int STATUS_LED_YELLOW;
extern const int STATUS_LED_GREEN;
extern const int STATUS_LED_BLUE;
extern const int ROTATION_SERVO_ACTIVE_POSITION;
extern const int ROTATION_SERVO_HOME_POSITION;
extern const unsigned long TA_SIGNAL_DURATION;
extern const unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS;
extern const float CUT_TRAVEL_DISTANCE;
extern const float FEED_TRAVEL_DISTANCE;
// CUT_MOTOR_STEPS_PER_INCH and FEED_MOTOR_STEPS_PER_INCH are declared in General_Functions.h
extern const float CUT_MOTOR_NORMAL_SPEED;
extern const float CUT_MOTOR_RETURN_SPEED;
extern const float CUT_MOTOR_HOMING_SPEED;
extern const float CUT_MOTOR_NORMAL_ACCELERATION;
extern const float FEED_MOTOR_NORMAL_SPEED;
extern const float FEED_MOTOR_RETURN_SPEED;
extern const float FEED_MOTOR_HOMING_SPEED;
extern const float FEED_MOTOR_NORMAL_ACCELERATION;
extern const float FEED_MOTOR_RETURN_ACCELERATION;



// currentState is declared in General_Functions.h

#endif 