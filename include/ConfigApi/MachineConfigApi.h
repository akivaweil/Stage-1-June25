#ifndef MACHINE_CONFIG_API_H
#define MACHINE_CONFIG_API_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

// SHARED MACHINE CONFIG + STATUS REST API
// Canonical cross-machine dashboard contract (see docs/DASHBOARD_API_CONTRACT.md).
// Every machine exposes the SAME three routes on port 80 with the SAME JSON
// envelope. Stage 1 is an async machine (ESPAsyncWebServer), so the server is
// passed by reference.
//
//   GET  /api/status  — live, read-only snapshot
//   GET  /api/config  — current editable settings (self-describing)
//   POST /api/config  — apply + persist (text/plain body parsed as JSON)
//
// All /api/* responses carry "Access-Control-Allow-Origin: *".

// Register the three /api/* routes on the shared AsyncWebServer.
// Call this from setupWebSocketDashboard() AFTER the "/" route and BEFORE
// server.begin().
void setupConfigApi(AsyncWebServer& server);

// GET /api/status body builder.
String buildStatusJson();

// GET /api/config body builder.
String buildConfigJson();

// POST /api/config handler core. Parses the flat key->value JSON body, validates
// every key against the curated schema, persists accepted values immediately and
// either applies live (when safe) or defers to the next IDLE entry.
//   returns true  => HTTP 200, body { ok:true, applied, deferred, message }
//   returns false => HTTP 400, body { ok:false, message }
// outDeferred is true when the change was persisted but live-apply was deferred.
bool applyConfigJson(const String& body, bool& outDeferred, String& outMsg);

// True only when the machine is safe to mutate live motion variables (IDLE/HOMING).
bool isSafeToApplyConfig();

// Called from the main loop on entry to IDLE: if a POST was deferred mid-cycle,
// apply the persisted settings to the live runtime now.
void applyDeferredConfigIfPending();

#endif // MACHINE_CONFIG_API_H
