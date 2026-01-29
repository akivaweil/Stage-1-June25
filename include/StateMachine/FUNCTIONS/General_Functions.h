#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
// #include <ESP32Servo.h> // Removed - using function-based PWM control instead

// Forward declarations and external variable references
extern bool blinkState;
extern bool errorBlinkState;
extern unsigned long lastErrorBlinkTime;
extern bool signalTAActive;
extern unsigned long signalTAStartTime;
extern unsigned long taSignalDelayStartTime; // For TA signal delay
extern bool taSignalDelayActive;             // For TA signal delay
extern bool rotationServoIsActiveAndTiming;
extern unsigned long rotationServoActiveStartTime;
extern bool rotationServoReturnCompleted; // Flag to prevent multiple calls to handleRotationServoReturn()
extern bool rotationClampIsExtended;
extern unsigned long rotationClampExtendTime;
extern bool isReloadMode;
extern bool errorAcknowledged;
extern bool startSwitchSafe;
extern bool continuousModeActive;
extern bool cuttingCycleInProgress;
extern bool woodSuctionError;
extern bool comingFromNoWoodWithSensorsClear;

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
    RETURNING_YES_2x4,
    RETURNING_NO_2x4,
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
extern const int _2x4_SECURE_CLAMP;
extern const int ROTATION_CLAMP;
extern const int STATUS_LED_RED;
extern const int STATUS_LED_YELLOW;
extern const int STATUS_LED_GREEN;
extern const int STATUS_LED_BLUE;
// Configuration constants moved to States_Config.h

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
void sendSignalToTA();

//* ************************************************************************
//* ************************* CLAMP FUNCTIONS ******************************
//* ************************************************************************
void extendFeedClamp();
void retractFeedClamp();
void extend2x4SecureClamp();
void retract2x4SecureClamp();
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
void handleNoWoodLedWavePattern();
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
void moveFeedMotorToHome();
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
void handleRotationServoReturn();
void handleTASignalTiming();
void moveFeedMotorToPostCutHome();

//* ************************************************************************
//* ************************* FLAG MANAGEMENT FUNCTIONS ********************
//* ************************************************************************
bool getComingFromNoWoodWithSensorsClear();
void setComingFromNoWoodWithSensorsClear(bool value);

#endif // GENERAL_FUNCTIONS_H 