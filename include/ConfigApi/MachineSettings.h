#ifndef MACHINE_SETTINGS_H
#define MACHINE_SETTINGS_H

#include <Arduino.h>
#include <ArduinoJson.h>

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ⚙️ MACHINE SETTINGS — curated schema + persistence passthrough      ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Owns the curated list of dashboard-editable settings for Stage 1.
//
// Stage 1 already persists every one of these variables to EEPROM via the
// existing websocket_dashboard.cpp routines (saveConfiguration() for the
// 3in/Minis config block and saveServoPositionsToEEPROM() for the servo home
// position). Per the contract ("EEPROM passthrough on Stage1/Router"),
// MachineSettings does NOT introduce a second persistence store — it routes
// saves through those existing functions so the on-disk format and the
// 3in/Minis dual-mode behaviour stay intact.

// Field types exposed to the dashboard.
enum SettingType { SETTING_FLOAT, SETTING_INT };

// One curated, dashboard-editable setting. Each maps to an existing live global
// runtime variable (held as either a float* or an unsigned long* / int*).
struct MachineSetting {
    const char* key;     // stable wire key (also the POST key)
    const char* label;   // human label for the dashboard input
    SettingType type;    // float | int (ulong values are surfaced as int)
    double min;          // inclusive lower bound
    double max;          // inclusive upper bound
    double step;         // dashboard input step

    // Exactly one of these target pointers is non-null.
    float* fTarget;        // for SETTING_FLOAT
    unsigned long* ulTarget; // for ulong-backed SETTING_INT
    int* iTarget;            // for int-backed SETTING_INT
};

// Curated schema (defined in MachineSettings.cpp).
extern const MachineSetting MACHINE_SETTINGS[];
extern const size_t MACHINE_SETTINGS_COUNT;

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🗂️ STAGED-OVERRIDE PERSIST (deferred path — no live-global writes)  ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Carries the deferred (staged) values for the dashboard-editable keys so they
// can be written straight into the persisted EEPROM struct WITHOUT ever being
// assigned to a live runtime global. Each `has*` flag selects between the staged
// value (true) and the current live global (false). Defined here (a plain POD)
// so MachineSettings.cpp can populate it from the staging buffer and
// websocket_dashboard.cpp can consume it against the ConfigurationData layout.
struct StagedConfigOverrides {
    bool  hasCutTravelDistance = false;     float        cutTravelDistance = 0;
    bool  hasFeedTravelDistance = false;    float        feedTravelDistance = 0;
    bool  hasCutMotorNormalSpeed = false;   float        cutMotorNormalSpeed = 0;
    bool  hasFeedMotorNormalSpeed = false;  float        feedMotorNormalSpeed = 0;
    bool  hasServoActiveHoldMs = false;     unsigned long servoActiveHoldMs = 0;
    bool  hasRotationClampExtendMs = false; unsigned long rotationClampExtendMs = 0;
    bool  hasTaSignalDuration = false;      unsigned long taSignalDuration = 0;
    bool  hasServoHomePosition = false;     int           servoHomePosition = 0;
};

// Persist a ConfigurationData/servo-home image to EEPROM. Every field is sourced
// from the live globals EXCEPT those flagged in `ov`, which are taken from the
// staged values. NO live global is read-modified-written; the live globals are
// byte-for-byte unchanged across the call. Defined in websocket_dashboard.cpp.
void saveConfigurationWithOverrides(const StagedConfigOverrides& ov);

// Compile-time upper bound for the staging buffer (>= MACHINE_SETTINGS_COUNT).
// MACHINE_SETTINGS_COUNT is a runtime const, so a constexpr is needed to size
// the static deferred-apply arrays.
static constexpr size_t MACHINE_SETTINGS_COUNT_MAX = 16;

// Read the current live value of a setting as a double (for JSON build).
double settingCurrentValue(const MachineSetting& s);

// Write an accepted (already range-validated) value into the live runtime
// variable. Does NOT persist and does NOT apply side effects.
void settingApplyValue(const MachineSetting& s, double value);

// Persist all accepted settings to EEPROM via the existing Stage 1 routines.
// (Snapshots the CURRENT live globals into EEPROM.)
void persistAllSettings();

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ 🗃️ DEFERRED-APPLY STAGING (mid-cycle POST)                          ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// Stage an accepted (range-validated) value for setting `index` without
// touching any live global. Used by the deferred POST path.
void stageSettingValue(size_t index, double value);

// Persist the currently-staged values to EEPROM with a zero net effect on every
// live global (save-then-restore around the existing save path). No live global
// is left changed.
void persistStagedSettings();

// Push every staged value into its live global (called on IDLE entry), then
// clear the staging buffer. Caller applies side effects afterwards.
void flushStagedSettingsToLive();

// Apply live side effects after one or more settings changed (e.g. recompute
// dynamic config that depends on CUT_TRAVEL_DISTANCE). Safe to call once after a
// batch of writes.
void applySettingsSideEffects();

#endif // MACHINE_SETTINGS_H
