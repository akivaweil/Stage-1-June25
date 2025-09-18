#ifndef WEBSOCKET_DASHBOARD_H
#define WEBSOCKET_DASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>

//* ************************************************************************
//* ********************** WEBSOCKET DASHBOARD ****************************
//* ************************************************************************
// Simple websocket dashboard for tracking cutting cycles
// Updates only when motors are not moving to avoid timing interference

// WebSocket server configuration
#define WEB_SERVER_PORT 80

// Function declarations
void setupWebSocketDashboard();
void incrementCuttingCycleCounter();
unsigned long getCuttingCycleCount();
void broadcastCuttingCycleCount();
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Global variables
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern unsigned long cuttingCycleCount;

#endif // WEBSOCKET_DASHBOARD_H
