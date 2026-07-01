#include "ConfigApi/MachineConfigApi.h"
#include "ConfigApi/MachineSettings.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "Config/Pins_Definitions.h"
#include <ArduinoJson.h>
#include <WiFi.h>

// MACHINE CONFIG + STATUS REST API — Stage 1 implementation
// Implements the canonical cross-machine dashboard contract
// (docs/DASHBOARD_API_CONTRACT.md). Async server (ESPAsyncWebServer).
//
// Keep ALL handling off the motor/stepper-critical path: these handlers only
// touch config globals + EEPROM, never command motion. Live application of a
// POST happens only when the machine is in the motionless IDLE state; otherwise
// the accepted values are staged + persisted (no live global touched) and pushed
// into the live globals on the next IDLE entry via the configDirty flag.

// Identity for this node (see the contract machine table).
static const char* MACHINE_ID   = "stage1";
static const char* MACHINE_NAME = "Stage 1";

// CORS header applied to every /api/* response.
static const char* CORS_HEADER = "Access-Control-Allow-Origin";
static const char* CORS_VALUE  = "*";

// Helpers defined in websocket_dashboard.cpp (not declared in a header there).
extern String getStateName(SystemState state);
extern String getSystemHealth();

// Feed motor steps/inch, used as the single dashboard stepsPerInch (see note in
// buildConfigJson). Declared extern by MotorConfig.h.
extern float FEED_MOTOR_STEPS_PER_INCH;

// Dual-mode (3 Inch / Minis) accessors, defined in WebDashboard.cpp. Exposed in
// /api/status as "mode" and switched via the reserved "__mode__" POST key so the
// central dashboard can drive the toggle that the old standalone dashboard had.
extern int  getCurrentConfigMode();
extern void setConfigMode(int mode);

// DEFERRED-APPLY FLAG
// Set when a POST is accepted + persisted mid-cycle but cannot be applied live.
// The main loop calls applyDeferredConfigIfPending() on entry to IDLE.
static volatile bool configDirty = false;

// SAFETY GATE
bool isSafeToApplyConfig() {
    // ONLY the truly-motionless IDLE state is safe to mutate live motion
    // variables. HOMING actively drives the steppers (speed-sensitive limit
    // seeks), so a mid-home edit could retarget an in-flight move; defer it.
    return (getCurrentState() == STATE_IDLE);
}

// GET /api/status
String buildStatusJson() {
    JsonDocument doc;
    doc["id"]       = MACHINE_ID;
    doc["name"]     = MACHINE_NAME;
    doc["state"]    = getStateName(getCurrentState());
    doc["health"]   = getSystemHealth();              // HEALTHY | WARNING | ERROR
    doc["uptimeMs"] = (uint32_t)millis();
    doc["freeHeap"] = (uint32_t)ESP.getFreeHeap();
    doc["rssi"]     = (int)WiFi.RSSI();
    doc["mode"]     = getCurrentConfigMode();         // 0 = 3 Inch, 1 = Minis

    // Flat sensor object. Polarity matches updateSensorStatus() in the dashboard.
    JsonObject sensors = doc["sensors"].to<JsonObject>();
    sensors["_2x4Present"]        = digitalRead(_2x4_PRESENT_SENSOR) == LOW;
    sensors["woodSuctionConfirm"] = digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == LOW;
    sensors["cutMotorHomeSwitch"] = digitalRead(CUT_MOTOR_HOME_SWITCH) == HIGH;
    sensors["feedMotorHomeSensor"]= digitalRead(FEED_MOTOR_HOME_SENSOR) == LOW;
    sensors["startCycleSwitch"]   = digitalRead(START_CYCLE_SWITCH) == HIGH;

    // Rolling cuts-per-minute averaged over 1/3/5/15-minute windows.
    float r1, r3, r5, r15;
    getCutRates(r1, r3, r5, r15);
    JsonObject cutRates = doc["cutRates"].to<JsonObject>();
    cutRates["m1"]  = r1;
    cutRates["m3"]  = r3;
    cutRates["m5"]  = r5;
    cutRates["m15"] = r15;

    String out;
    serializeJson(doc, out);
    return out;
}

// GET /api/config
String buildConfigJson() {
    JsonDocument doc;
    doc["id"]     = MACHINE_ID;
    doc["schema"] = 1;
    // Single steps/inch for the shared dashboard's steps<->inches conversion. The
    // dashboard divides value/min/max of any fromSteps field by this and rounds
    // back to whole steps on POST. Only the FEED motor fields are stored in steps
    // (FEED_MOTOR_STEPS_PER_INCH = 1000); the CUT fields are already in inches and
    // carry NO fromSteps, so this constant never touches them.
    doc["stepsPerInch"] = FEED_MOTOR_STEPS_PER_INCH;

    JsonArray fields = doc["fields"].to<JsonArray>();
    for (size_t i = 0; i < MACHINE_SETTINGS_COUNT; i++) {
        const MachineSetting& s = MACHINE_SETTINGS[i];
        JsonObject f = fields.add<JsonObject>();
        f["key"]   = s.key;
        f["label"] = s.label;
        f["type"]  = (s.type == SETTING_FLOAT) ? "float" : "int";
        if (s.type == SETTING_INT) {
            f["value"] = (long)(settingCurrentValue(s) + 0.5);
        } else {
            f["value"] = settingCurrentValue(s);
        }
        f["min"]  = s.min;
        f["max"]  = s.max;
        f["step"] = s.step;
        f["group"] = s.group;
        if (s.fromSteps) f["fromSteps"] = true;
        if (s.collapsed) f["collapsed"] = true;
    }

    String out;
    serializeJson(doc, out);
    return out;
}

// POST /api/config — core
// Returns true on success (all keys known + in range). On success the values are
// always persisted; outDeferred reports whether live-apply was deferred.
bool applyConfigJson(const String& body, bool& outDeferred, String& outMsg) {
    outDeferred = false;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        outMsg = "invalid JSON";
        return false;
    }
    JsonObject obj = doc.as<JsonObject>();
    if (obj.isNull()) {
        outMsg = "expected JSON object";
        return false;
    }

    // Reserved key: switch the 3 Inch / Minis config mode. Not a MACHINE_SETTINGS
    // field — handle it here (before field validation) so the central dashboard's
    // mode toggle works. Switching reloads the active EEPROM block (mirrors the
    // old websocket set_config_mode), so it is sent on its own with no other keys.
    if (obj["__mode__"].is<int>() || obj["__mode__"].is<long>()) {
        int m = obj["__mode__"].as<int>();
        if (m != 0 && m != 1) {
            outMsg = "__mode__ invalid/out of range";
            return false;
        }
        setConfigMode(m);
        outDeferred = false;
        outMsg = "saved";
        return true;
    }

    // PASS 1 — validate every key/value BEFORE mutating anything. A single bad
    // key rejects the whole request and changes nothing.
    for (JsonPair kv : obj) {
        const char* key = kv.key().c_str();
        const MachineSetting* match = nullptr;
        for (size_t i = 0; i < MACHINE_SETTINGS_COUNT; i++) {
            if (strcmp(MACHINE_SETTINGS[i].key, key) == 0) { match = &MACHINE_SETTINGS[i]; break; }
        }
        if (!match) {
            outMsg = String(key) + " invalid/out of range";
            return false;
        }
        if (!kv.value().is<double>() && !kv.value().is<long>()) {
            outMsg = String(key) + " invalid/out of range";
            return false;
        }
        double v = kv.value().as<double>();
        if (v < match->min || v > match->max) {
            outMsg = String(key) + " invalid/out of range";
            return false;
        }
    }

    // Branch on safety BEFORE any mutation. Live globals are written ONLY when
    // it is safe to apply (motionless IDLE). On the deferred path NOTHING live
    // is touched — values are staged + persisted only, and pushed into the live
    // globals on the next IDLE entry (applyDeferredConfigIfPending()).
    if (isSafeToApplyConfig()) {
        // SAFE PATH — write accepted values into the live runtime variables now.
        for (JsonPair kv : obj) {
            const char* key = kv.key().c_str();
            for (size_t i = 0; i < MACHINE_SETTINGS_COUNT; i++) {
                if (strcmp(MACHINE_SETTINGS[i].key, key) == 0) {
                    settingApplyValue(MACHINE_SETTINGS[i], kv.value().as<double>());
                    break;
                }
            }
        }
        // Persist the now-live values, then recompute dependent dynamic config.
        persistAllSettings();
        applySettingsSideEffects();
        outDeferred = false;
        outMsg = "saved";
        return true;
    }

    // DEFERRED PATH (mid-cycle) — stage the accepted values and persist them to
    // EEPROM WITHOUT writing any live runtime variable. The staged values are
    // copied into the live globals + recomputed on the next IDLE entry.
    for (JsonPair kv : obj) {
        const char* key = kv.key().c_str();
        for (size_t i = 0; i < MACHINE_SETTINGS_COUNT; i++) {
            if (strcmp(MACHINE_SETTINGS[i].key, key) == 0) {
                stageSettingValue(i, kv.value().as<double>());
                break;
            }
        }
    }
    // Persist staged values now (power-cut safe) with zero net effect on globals.
    persistStagedSettings();
    configDirty = true;
    outDeferred = true;
    outMsg = "pending — applies at next idle";
    return true;
}

// DEFERRED APPLY (called from main loop on IDLE entry)
void applyDeferredConfigIfPending() {
    if (!configDirty) return;
    configDirty = false;
    // Push the staged (already-persisted) values into the live globals now that
    // the machine is motionless, then recompute dependent dynamic config.
    flushStagedSettingsToLive();
    applySettingsSideEffects();
}

// ROUTE REGISTRATION
void setupConfigApi(AsyncWebServer& server) {
    // GET /api/status
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse* response =
            request->beginResponse(200, "application/json", buildStatusJson());
        response->addHeader(CORS_HEADER, CORS_VALUE);
        request->send(response);
    });

    // GET /api/config
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse* response =
            request->beginResponse(200, "application/json", buildConfigJson());
        response->addHeader(CORS_HEADER, CORS_VALUE);
        request->send(response);
    });

    // POST /api/config — body arrives as text/plain (CORS simple request). Use a
    // raw onBody accumulator (NOT AsyncCallbackJsonWebHandler) so all three async
    // machines share the identical pattern. No OPTIONS handler.
    server.on("/api/config", HTTP_POST,
        // onRequest: fires AFTER onBody. When a body was present, onBody already
        // sent the response, so only handle the body-less case here (avoids a
        // double-send). contentLength()==0 means no body arrived.
        [](AsyncWebServerRequest* request) {
            if (request->contentLength() == 0) {
                bool deferred = false;
                String msg;
                bool ok = applyConfigJson(String(), deferred, msg);
                JsonDocument resp;
                resp["ok"] = ok;
                if (ok) { resp["applied"] = !deferred; resp["deferred"] = deferred; }
                resp["message"] = msg;
                String out;
                serializeJson(resp, out);
                AsyncWebServerResponse* response =
                    request->beginResponse(ok ? 200 : 400, "application/json", out);
                response->addHeader(CORS_HEADER, CORS_VALUE);
                request->send(response);
            }
        },
        // onUpload: unused.
        nullptr,
        // onBody: accumulate chunks into a String stored on the request, then on
        // the final chunk parse + respond.
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            String* acc = reinterpret_cast<String*>(request->_tempObject);
            if (index == 0) {
                acc = new String();
                acc->reserve(total);
                request->_tempObject = acc;
            }
            if (acc) {
                for (size_t i = 0; i < len; i++) {
                    acc->concat((char)data[i]);
                }
            }
            if (index + len == total) {
                String body = acc ? *acc : String();
                if (acc) {
                    delete acc;
                    request->_tempObject = nullptr;
                }
                bool deferred = false;
                String msg;
                bool ok = applyConfigJson(body, deferred, msg);
                JsonDocument resp;
                resp["ok"] = ok;
                if (ok) { resp["applied"] = !deferred; resp["deferred"] = deferred; }
                resp["message"] = msg;
                String out;
                serializeJson(resp, out);
                AsyncWebServerResponse* response =
                    request->beginResponse(ok ? 200 : 400, "application/json", out);
                response->addHeader(CORS_HEADER, CORS_VALUE);
                request->send(response);
            }
        }
    );
}
