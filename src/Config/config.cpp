#include "StateMachine/STATES/States_Config.h"
#include "Config/Motor_Config.h"

//═══════════════════════════════════════════════════════════════════════════
//║ ═══════════════════════════════════════════════════════════════════════ ║
//║          📋  SYSTEM CONFIGURATION FILE - STAGE 1 JUNE 25                ║
//║ ═══════════════════════════════════════════════════════════════════════ ║
//═══════════════════════════════════════════════════════════════════════════

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ 💾 EEPROM STORAGE CONFIGURATION                              ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
const int CONFIG_EEPROM_SIZE = 2048;  // EEPROM size for configuration storage
const int CONFIG_OFFSET = 0;          // Configuration starts at beginning of extended EEPROM

//╔═══╗ ════════════════════════════════════════════════════════ ╔═══╗
//║ ⏱️ TIMING & DELAY CONSTANTS                                  ║
//╚═══╝ ════════════════════════════════════════════════════════ ╚═══╝
unsigned long SENSOR_STABILIZATION_DELAY_MS = 20;     // Sensor reading stabilization delay
const unsigned long CLAMP_FEED_MOTOR_DELAY_MS = 100;   // Delay between clamp and feed motor movement
const unsigned long CYLINDER_ACTION_DELAY_MS = 150;    // Delay for cylinder actions
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 30; // Sensor verification stabilization delay
