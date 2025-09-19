#ifndef WEBSOCKET_DASHBOARD_H
#define WEBSOCKET_DASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ********************** WEBSOCKET DASHBOARD ****************************
//* ************************************************************************
// Simple websocket dashboard for tracking cutting cycles
// Updates only when motors are not moving to avoid timing interference

// WebSocket server configuration
#define WEB_SERVER_PORT 80

// Function declarations
void setupWebSocketDashboard();
void startCuttingCycleTimer();
void incrementCuttingCycleCounter();
unsigned long getCuttingCycleCount();
void broadcastCuttingCycleCount();
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Enhanced dashboard functions
void broadcastSystemStatus();
void broadcastSensorStatus();
void broadcastClampStatus();
void broadcastLEDStatus();
void broadcastPerformanceMetrics();
void broadcastErrorStatus();
void broadcastNetworkInfo();
void broadcastEventLog();
void addEventToLog(const String& event);
void updatePerformanceMetrics(unsigned long cycleTime);
void calculateTimeBasedMetrics();
void updateErrorCount(const String& errorType);
void onStateChange(SystemState newState);
void onErrorOccurred(const String& errorType);
void updateDashboardStatus();

// Global variables
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern unsigned long cuttingCycleCount;

// Enhanced dashboard data structures
struct SystemStatus {
    String currentState;
    String previousState;
    String systemHealth;
    unsigned long uptime;
    unsigned long lastStateChange;
};

struct SensorStatus {
    bool _2x4Present;
    bool woodSuctionConfirm;
    bool firstCutOrWoodFwdOne;
    bool cutMotorHomeSwitch;
    bool feedMotorHomeSensor;
    bool reloadSwitch;
    bool startCycleSwitch;
    bool manualFeedSwitch;
};

struct ClampStatus {
    bool feedClamp;
    bool _2x4SecureClamp;
    bool rotationClamp;
};

struct LEDStatus {
    bool red;
    bool yellow;
    bool green;
    bool blue;
};

struct PerformanceMetrics {
    float lastCycleTime;
    float averageCycleTime;
    unsigned long totalCycles;
    
    // Time-based cycle tracking
    unsigned long cycles1Min;
    unsigned long cycles3Min;
    unsigned long cycles5Min;
    unsigned long cycles15Min;
    unsigned long cycles30Min;
    
    // Time-based averages (cycles per minute)
    float avgCycles1Min;
    float avgCycles3Min;
    float avgCycles5Min;
    float avgCycles15Min;
    float avgCycles30Min;
    
    // Cycle timestamps for time-based calculations
    unsigned long cycleTimestamps[100]; // Store last 100 cycle timestamps
    int cycleTimestampIndex;
    int cycleTimestampCount;
    
    unsigned long totalUptime;
    float efficiency;
};

struct ErrorInfo {
    String lastError;
    unsigned long lastErrorTime;
    int errorCount;
    String errorHistory[10];
    int errorHistoryIndex;
};

struct NetworkInfo {
    int wifiSignal;
    int freeHeap;
    int freePSRAM;
    float temperature;
    unsigned long uptime;
};

struct EventLog {
    String events[50];
    int eventIndex;
    int eventCount;
};

extern SystemStatus systemStatus;
extern SensorStatus sensorStatus;
extern ClampStatus clampStatus;
extern LEDStatus ledStatus;
extern PerformanceMetrics performanceMetrics;
extern ErrorInfo errorInfo;
extern NetworkInfo networkInfo;
extern EventLog eventLog;

#endif // WEBSOCKET_DASHBOARD_H
