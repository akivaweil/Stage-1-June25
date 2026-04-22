#include "Config/Config.h"

//═══════════════════════════════════════════════════════════════════════════
//║ 📋  SYSTEM CONFIGURATION - STAGE 1                                       ║
//═══════════════════════════════════════════════════════════════════════════
// Definitions for every tunable constant declared in Config.h.
// Motor-specific constants are defined in Motor_Config.cpp.

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 💾 EEPROM STORAGE                                                    ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
const int CONFIG_EEPROM_SIZE = 2048;
const int CONFIG_OFFSET = 0;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ ROTATION SERVO                                                    ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// (HOME/ACTIVE position values live in websocket_dashboard.cpp — dashboard-managed)
unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2200;
unsigned long ROTATION_SERVO_RETURN_DELAY_MS = 150;
unsigned long ROTATION_SERVO_HOME_WAIT_DURATION_MS = 300;
unsigned long ROTATION_SERVO_SUCTION_HOLD_DURATION_MS = 300;
float ROTATION_SERVO_ACTIVATION_DISTANCE = 8.2;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔧 ROTATION CLAMP                                                    ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 2200;
float ROTATION_CLAMP_ACTIVATION_DISTANCE = 5.75;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🤖 TRANSFER ARM                                                      ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long TA_SIGNAL_DURATION = 5000;
float TA_SIGNAL_OFFSET_FROM_END = 0.2;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🧲 SUCTION SENSOR & RETRY                                            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long SUCTION_WAIT_TIMEOUT_MS = 1500;
unsigned long SUCTION_RETRY_PHASE1_WAIT_MS = 3000;
unsigned long SUCTION_RETRY_PHASE2_WAIT_MS = 3000;
unsigned long SUCTION_RETRY_SUCCESS_WAIT_MS = 1000;
unsigned long SUCTION_RETRY_INTER_PULSE_GAP_MS = 200;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🔄 CUT MOTOR TIMING                                                  ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long CUT_HOME_TIMEOUT = 5000;
unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS = 2000;
unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS = 20;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⏱️ GENERAL TIMING                                                    ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
unsigned long SENSOR_STABILIZATION_DELAY_MS = 20;
const unsigned long CLAMP_FEED_MOTOR_DELAY_MS = 100;
const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 100;
