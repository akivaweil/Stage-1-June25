#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Single source of truth for all tunable timing/position/speed constants.
// Motor-specific constants live in MotorConfig.h.

#include "Config/MotorConfig.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ ROTATION SERVO                                                   ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern int ROTATION_SERVO_HOME_POSITION;
extern const int ROTATION_SERVO_ACTIVE_OFFSET; // Active position = HOME + this offset (degrees)
extern unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
extern unsigned long ROTATION_SERVO_RETURN_DELAY_MS;
extern unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS;
extern unsigned long ROTATION_SERVO_SUCTION_HOLD_DURATION_MS;
extern float ROTATION_SERVO_ACTIVATION_DISTANCE;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 ROTATION CLAMP                                                   ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS;
extern float ROTATION_CLAMP_ACTIVATION_DISTANCE;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🤖 TRANSFER ARM                                                     ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern unsigned long TA_SIGNAL_DURATION;
extern float TA_SIGNAL_OFFSET_FROM_END;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🧲 SUCTION SENSOR & RETRY                                           ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern unsigned long SUCTION_WAIT_TIMEOUT_MS;
extern unsigned long SUCTION_RETRY_PHASE1_WAIT_MS;
extern unsigned long SUCTION_RETRY_PHASE2_WAIT_MS;
extern unsigned long SUCTION_RETRY_SUCCESS_WAIT_MS;
extern unsigned long SUCTION_RETRY_INTER_PULSE_GAP_MS;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⏪ FEED PULLBACK (YESWOOD entry — wood-present path only)           ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern float FEED_PULLBACK_DISTANCE;             // inches — how far feed motor retreats with feed clamp gripping wood
extern float FEED_PULLBACK_FEED_COMPENSATION;    // inches — added to FEED_TRAVEL_DISTANCE on next forward stroke

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔄 CUT MOTOR TIMING                                                 ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern unsigned long CUT_HOME_TIMEOUT;
extern unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS;
extern unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⏱️ GENERAL TIMING                                                   ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
extern unsigned long SENSOR_STABILIZATION_DELAY_MS;

#endif // SYSTEM_CONFIG_H
