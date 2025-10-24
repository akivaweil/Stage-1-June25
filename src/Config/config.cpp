#include "StateMachine/STATES/States_Config.h"
#include "Config/Motor_Config.h"

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION *************************
//* ************************************************************************

//* ************************************************************************
//* ************************ EEPROM CONFIGURATION *************************
//* ************************************************************************
const int CONFIG_EEPROM_SIZE = 2048; // Increase EEPROM size for configuration
const int CONFIG_OFFSET = 0; // Configuration starts at beginning of extended EEPROM

//* ************************************************************************
//* ************************ TIMING CONSTANTS ******************************
//* ************************************************************************
unsigned long SENSOR_STABILIZATION_DELAY_MS = 30;
const unsigned long CLAMP_FEED_MOTOR_DELAY_MS = 100; // Delay between clamp extending/retracting and feed motor movement
const unsigned long FEED_CLAMP_DELAY_MS = 300; // Delay after extending feed clamp and retracting secure clamp
const unsigned long CYLINDER_ACTION_DELAY_MS = 150; // Delay for cylinder actions
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 30; // 30ms sensor stabilization delay

//* ************************************************************************
//* ************************ [⚙️] ROTATION SERVO **************************
//* ************************************************************************
int ROTATION_SERVO_HOME_POSITION = 27;
int ROTATION_SERVO_ACTIVE_POSITION = 120;
unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2000;
unsigned long ROTATION_SERVO_RETURN_DELAY_MS = 150;
unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS = 500;
float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = 0.2;

//* ************************************************************************
//* ************************ [🔧] ROTATION CLAMP **************************
//* ************************************************************************
unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 2200;
float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 2.7;

//* ************************************************************************
//* ************************ [🔄] CUT MOTOR TIMING *************************
//* ************************************************************************
unsigned long CUT_HOME_TIMEOUT = 5000;
unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000;
unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20;

//* ************************************************************************
//* ************************ [🤖] TRANSFER ARM *****************************
//* ************************************************************************
unsigned long TA_SIGNAL_DURATION = 1000;
float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = 0.01;

//* ************************************************************************
//* ************************ ERROR RECOVERY CONSTANTS *********************
//* ************************************************************************
const unsigned long CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS = 5000; // 5 second maximum recovery time
const float CUT_MOTOR_HOME_RECOVERY_SPEED = 1000; // Recovery speed (same as homing speed)
const float DECELERATION_DISTANCE_INCHES = 0.2; // Maximum 0.2 inch deceleration distance

//* ************************************************************************
//* ************************ STATE OPERATION CONSTANTS ********************
//* ************************************************************************
const float FEED_MOTOR_SPEED_MULTIPLIER = 0.6; // Speed reduction for NO_2x4 returning sequence
const float FEED_MOTOR_2ND_POSITION = -1.2; // Position for 2nd position movement
const float FEED_MOTOR_HOME_POSITION = 1.0; // Home position
const float FEED_MOTOR_FINAL_POSITION = -1.2; // Final position

const float FEED_MOTOR_FIRST_RUN_START_POSITION = -1.2; // inches - absolute position for first run start
const float FEED_MOTOR_FIRST_RUN_END_POSITION = 3.4; // inches - absolute position for first run end
const float FEED_MOTOR_SECOND_RUN_START_POSITION = -1.2; // inches - absolute position for second run start
const float FEED_MOTOR_SECOND_RUN_END_POSITION = 2.1; // inches - absolute position for second run end

const unsigned long FEED_HOME_TIMEOUT = 30000; // 30 seconds timeout

//* ************************************************************************
//* ************************ STATES CONFIGURATION ************************
//* ************************************************************************

