#include "ConfigApi/MachineSettings.h"
#include "Config/Config.h"
#include "Config/MotorConfig.h"

// MACHINE SETTINGS — Stage 1 curated schema + EEPROM passthrough

// Live runtime variables owned by other translation units.
// CUT_TRAVEL_DISTANCE / FEED_TRAVEL_DISTANCE / ROTATION_SERVO_HOME_POSITION are
// defined in websocket_dashboard.cpp; the remaining five are declared extern by
// Config.h / MotorConfig.h (already included above).
extern float CUT_TRAVEL_DISTANCE;
extern float FEED_TRAVEL_DISTANCE;
extern int ROTATION_SERVO_HOME_POSITION;
// FEED_MOTOR_NORMAL_SPEED is declared `float` in MotorConfig.h even though it
// carries a steps/sec magnitude; it is surfaced as an int-typed field but
// read/written through its float storage via fTarget.

// Existing Stage 1 persistence routines (defined in websocket_dashboard.cpp).
// Declared here rather than in a header to keep the footprint minimal.
extern void saveConfiguration();
extern void saveServoPositionsToEEPROM();
extern void updateDynamicConfig();

// CURATED SETTINGS SCHEMA
// Order, keys, ranges and defaults match the central dashboard spec.
//
// Trailing members: { group, fromSteps, collapsed }.
//   group     — dashboard section heading (first-appearance order).
//   fromSteps — true ONLY for fields stored in STEPS; the shared dashboard then
//               divides value/min/max by doc["stepsPerInch"] (= FEED_MOTOR_STEPS_PER_INCH,
//               1000) to display in/s and multiplies back on POST.
//   collapsed — the field's group section starts collapsed.
//
// TWO-DIFFERENT-STEPS-PER-INCH NOTE: only ONE stepsPerInch is emitted
// (FEED_MOTOR_STEPS_PER_INCH = 1000). The FEED speed/accel are stored in STEPS,
// so they get fromSteps=true. The CUT speed/accel are ALREADY stored in
// inches/sec (and in/s^2), so they MUST NOT set fromSteps — flagging them would
// wrongly divide by 1000. They are simply relabeled with their inch units.
const MachineSetting MACHINE_SETTINGS[] = {
    { "CUT_TRAVEL_DISTANCE",                   "Cut Travel (in)",          SETTING_FLOAT,   0.1,    20.0,  0.05, &CUT_TRAVEL_DISTANCE, nullptr, nullptr,                                  "General", false, false },
    { "FEED_TRAVEL_DISTANCE",                  "Feed Travel (in)",         SETTING_FLOAT,   0.1,    10.0,  0.05, &FEED_TRAVEL_DISTANCE, nullptr, nullptr,                                 "General", false, false },
    { "ROTATION_SERVO_HOME_POSITION",          "Servo Home (deg)",         SETTING_INT,     0.0,   180.0,  1.0,  nullptr, nullptr, &ROTATION_SERVO_HOME_POSITION,                            "General", false, false },
    { "ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS","Servo Active Hold (ms)",   SETTING_INT,   100.0, 60000.0,  10.0, nullptr, &ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS, nullptr,                  "General", false, false },
    { "ROTATION_CLAMP_EXTEND_DURATION_MS",     "Rotation Clamp Extend (ms)",SETTING_INT,  200.0,  5000.0,  10.0, nullptr, &ROTATION_CLAMP_EXTEND_DURATION_MS, nullptr,                       "General", false, false },
    { "ROTATION_CLAMP_ACTIVATION_DISTANCE",    "Rotation Clamp Activation (in)",SETTING_FLOAT, 0.1, 20.0,  0.05, &ROTATION_CLAMP_ACTIVATION_DISTANCE, nullptr, nullptr,                  "General", false, false },
    { "TA_SIGNAL_DURATION",                    "TA Signal Duration (ms)",  SETTING_INT,   100.0, 60000.0,  10.0, nullptr, &TA_SIGNAL_DURATION, nullptr,                                      "General", false, false },
    // Motors group — CUT params are stored in inches (NO fromSteps); FEED params
    // are stored in steps (fromSteps=true, dashboard divides by stepsPerInch).
    { "CUT_MOTOR_NORMAL_SPEED",                "Cut Motor Speed (in/s)",   SETTING_FLOAT,   0.1,    50.0,  0.01, &CUT_MOTOR_NORMAL_SPEED, nullptr, nullptr,                              "Motors", false, true },
    { "CUT_MOTOR_NORMAL_ACCELERATION",         "Cut Motor Accel (in/s^2)", SETTING_FLOAT,   0.5,   500.0,  0.1,  &CUT_MOTOR_NORMAL_ACCELERATION, nullptr, nullptr,                       "Motors", false, true },
    { "FEED_MOTOR_NORMAL_SPEED",               "Feed Motor Speed (in/s)",  SETTING_INT,   100.0, 50000.0,  10.0, &FEED_MOTOR_NORMAL_SPEED, nullptr, nullptr,                             "Motors", true,  true },
    { "FEED_MOTOR_NORMAL_ACCELERATION",        "Feed Motor Accel (in/s^2)",SETTING_INT,   100.0,500000.0,  10.0, &FEED_MOTOR_NORMAL_ACCELERATION, nullptr, nullptr,                      "Motors", true,  true },
};

const size_t MACHINE_SETTINGS_COUNT = sizeof(MACHINE_SETTINGS) / sizeof(MACHINE_SETTINGS[0]);

// VALUE ACCESS

double settingCurrentValue(const MachineSetting& s) {
    if (s.fTarget) return (double)(*s.fTarget);
    if (s.ulTarget) return (double)(*s.ulTarget);
    if (s.iTarget) return (double)(*s.iTarget);
    return 0.0;
}

void settingApplyValue(const MachineSetting& s, double value) {
    if (s.fTarget) { *s.fTarget = (float)value; return; }
    if (s.ulTarget) { *s.ulTarget = (unsigned long)(value + 0.5); return; }
    if (s.iTarget) { *s.iTarget = (int)(value + 0.5); return; }
}

// DEFERRED-APPLY STAGING BUFFER
// Holds accepted-but-not-yet-live values from a mid-cycle POST. The buffer is
// parallel to MACHINE_SETTINGS (indexed by setting index). On the deferred path
// applyConfigJson() stages values here and persists them WITHOUT touching any
// live global; the staged values are pushed into the live globals only on the
// next IDLE entry (flushStagedSettingsToLive()).
static double s_stagedValue[MACHINE_SETTINGS_COUNT_MAX];
static bool   s_stagedPending[MACHINE_SETTINGS_COUNT_MAX];

void stageSettingValue(size_t index, double value) {
    if (index >= MACHINE_SETTINGS_COUNT) return;
    s_stagedValue[index]   = value;
    s_stagedPending[index] = true;
}

void flushStagedSettingsToLive() {
    for (size_t i = 0; i < MACHINE_SETTINGS_COUNT; i++) {
        if (s_stagedPending[i]) {
            settingApplyValue(MACHINE_SETTINGS[i], s_stagedValue[i]);
            s_stagedPending[i] = false;
        }
    }
}

// DEFERRED PERSIST (no live-global writes)
// Persist the currently-staged values to EEPROM WITHOUT writing any live global.
// The staged values are packed into a StagedConfigOverrides and written straight
// into the persisted ConfigurationData (and the servo-home EEPROM region) by
// saveConfigurationWithOverrides(); every motion global is left byte-for-byte
// unchanged, so the main loop on the other core never observes a staged value
// mid-cycle. flushStagedSettingsToLive() (on IDLE) remains the ONLY place staged
// values reach the live globals. A power cut after this still keeps the new
// value (it is already on EEPROM).
void persistStagedSettings() {
    // Map staged schema entries (by stable key) into the typed override struct.
    // Only staged keys set their `has*` flag; every other field is sourced from
    // the live global inside saveConfigurationWithOverrides().
    StagedConfigOverrides ov;
    for (size_t i = 0; i < MACHINE_SETTINGS_COUNT; i++) {
        if (!s_stagedPending[i]) continue;
        const char* key = MACHINE_SETTINGS[i].key;
        double v = s_stagedValue[i];
        if (strcmp(key, "CUT_TRAVEL_DISTANCE") == 0) {
            ov.hasCutTravelDistance = true; ov.cutTravelDistance = (float)v;
        } else if (strcmp(key, "FEED_TRAVEL_DISTANCE") == 0) {
            ov.hasFeedTravelDistance = true; ov.feedTravelDistance = (float)v;
        } else if (strcmp(key, "CUT_MOTOR_NORMAL_SPEED") == 0) {
            ov.hasCutMotorNormalSpeed = true; ov.cutMotorNormalSpeed = (float)v;
        } else if (strcmp(key, "FEED_MOTOR_NORMAL_SPEED") == 0) {
            ov.hasFeedMotorNormalSpeed = true; ov.feedMotorNormalSpeed = (float)v;
        } else if (strcmp(key, "CUT_MOTOR_NORMAL_ACCELERATION") == 0) {
            ov.hasCutMotorNormalAccel = true; ov.cutMotorNormalAccel = (float)v;
        } else if (strcmp(key, "FEED_MOTOR_NORMAL_ACCELERATION") == 0) {
            ov.hasFeedMotorNormalAccel = true; ov.feedMotorNormalAccel = (float)v;
        } else if (strcmp(key, "ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS") == 0) {
            ov.hasServoActiveHoldMs = true; ov.servoActiveHoldMs = (unsigned long)(v + 0.5);
        } else if (strcmp(key, "ROTATION_CLAMP_EXTEND_DURATION_MS") == 0) {
            ov.hasRotationClampExtendMs = true; ov.rotationClampExtendMs = (unsigned long)(v + 0.5);
        } else if (strcmp(key, "ROTATION_CLAMP_ACTIVATION_DISTANCE") == 0) {
            ov.hasRotationClampActivationDistance = true; ov.rotationClampActivationDistance = (float)v;
        } else if (strcmp(key, "TA_SIGNAL_DURATION") == 0) {
            ov.hasTaSignalDuration = true; ov.taSignalDuration = (unsigned long)(v + 0.5);
        } else if (strcmp(key, "ROTATION_SERVO_HOME_POSITION") == 0) {
            ov.hasServoHomePosition = true; ov.servoHomePosition = (int)(v + 0.5);
        }
        // Any future schema key without an override slot simply isn't persisted on
        // the deferred path (it still goes live + persists on the next IDLE flush).
    }
    saveConfigurationWithOverrides(ov);
}

// PERSISTENCE PASSTHROUGH

void persistAllSettings() {
    // saveConfiguration() snapshots all the EEPROM-backed config globals
    // (including CUT_TRAVEL_DISTANCE, FEED_TRAVEL_DISTANCE, CUT_MOTOR_NORMAL_SPEED,
    // FEED_MOTOR_NORMAL_SPEED, the timing/duration values, etc.) into the active
    // 3in/Minis EEPROM block. The servo home position lives in its own region.
    saveConfiguration();
    saveServoPositionsToEEPROM();
}

void applySettingsSideEffects() {
    // CUT_TRAVEL_DISTANCE feeds the dynamic activation-distance math; recompute it
    // so a live edit takes effect immediately (mirrors the existing websocket path).
    updateDynamicConfig();
}
