#include "WebSocketDashboard/websocket_dashboard.h"
#include "WebSocketDashboard/dashboard_html.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pins_Definitions.h"
#include <ArduinoJson.h>

//* ************************************************************************
//* ********************** WEBSOCKET DASHBOARD ****************************
//* ************************************************************************
// Enhanced websocket dashboard for comprehensive table saw monitoring
// Updates only when motors are not moving to avoid timing interference

// Global variables
AsyncWebServer server(WEB_SERVER_PORT);
AsyncWebSocket ws("/ws");
unsigned long cuttingCycleCount = 0;
unsigned long systemStartTime = 0;
unsigned long lastCycleStartTime = 0;
unsigned long lastCycleCompletionTime = 0;
unsigned long lastStateChangeTime = 0;

// Enhanced dashboard data structures
SystemStatus systemStatus;
SensorStatus sensorStatus;
ClampStatus clampStatus;
LEDStatus ledStatus;
PerformanceMetrics performanceMetrics;
ErrorInfo errorInfo;
NetworkInfo networkInfo;
EventLog eventLog;

// Helper function to get state name
String getStateName(SystemState state) {
    switch(state) {
        case STARTUP: return "STARTUP";
        case HOMING: return "HOMING";
        case IDLE: return "IDLE";
        case CUTTING: return "CUTTING";
        case ERROR: return "ERROR";
        case ERROR_RESET: return "ERROR_RESET";
        case SUCTION_ERROR: return "SUCTION_ERROR";
        case Cut_Motor_Homing_Error: return "CUT_MOTOR_ERROR";
        case RETURNING_YES_2x4: return "RETURNING_YES_2x4";
        case RETURNING_NO_2x4: return "RETURNING_NO_2x4";
        case FEED_FIRST_CUT: return "FEED_FIRST_CUT";
        case FEED_WOOD_FWD_ONE: return "FEED_WOOD_FWD_ONE";
        default: return "UNKNOWN";
    }
}

// Helper function to get system health
String getSystemHealth() {
    if (getCurrentState() == ERROR || getCurrentState() == SUCTION_ERROR || getCurrentState() == Cut_Motor_Homing_Error) {
        return "ERROR";
    } else if (getCurrentState() == ERROR_RESET) {
        return "WARNING";
    } else {
        return "HEALTHY";
    }
}

// Initialize dashboard data structures
void initializeDashboardData() {
    systemStartTime = millis();
    lastStateChangeTime = millis();
    
    // Initialize system status
    systemStatus.currentState = getStateName(getCurrentState());
    systemStatus.previousState = getStateName(getPreviousState());
    systemStatus.systemHealth = getSystemHealth();
    systemStatus.uptime = 0;
    systemStatus.lastStateChange = lastStateChangeTime;
    
    // Initialize performance metrics
    performanceMetrics.lastCycleTime = 0;
    performanceMetrics.averageCycleTime = 0;
    performanceMetrics.totalCycles = 0;
    
    // Initialize time-based cycle tracking
    performanceMetrics.cycles1Min = 0;
    performanceMetrics.cycles3Min = 0;
    performanceMetrics.cycles5Min = 0;
    performanceMetrics.cycles15Min = 0;
    performanceMetrics.cycles30Min = 0;
    
    // Initialize time-based averages
    performanceMetrics.avgCycles1Min = 0.0;
    performanceMetrics.avgCycles3Min = 0.0;
    performanceMetrics.avgCycles5Min = 0.0;
    performanceMetrics.avgCycles15Min = 0.0;
    performanceMetrics.avgCycles30Min = 0.0;
    
    // Initialize cycle timestamp tracking
    performanceMetrics.cycleTimestampIndex = 0;
    performanceMetrics.cycleTimestampCount = 0;
    for (int i = 0; i < 100; i++) {
        performanceMetrics.cycleTimestamps[i] = 0;
    }
    
    performanceMetrics.totalUptime = 0;
    performanceMetrics.efficiency = 100.0;
    
    // Initialize error info
    errorInfo.lastError = "None";
    errorInfo.lastErrorTime = 0;
    errorInfo.errorCount = 0;
    errorInfo.errorHistoryIndex = 0;
    for (int i = 0; i < 10; i++) {
        errorInfo.errorHistory[i] = "";
    }
    
    // Initialize event log
    eventLog.eventIndex = 0;
    eventLog.eventCount = 0;
    for (int i = 0; i < 50; i++) {
        eventLog.events[i] = "";
    }
    
    addEventToLog("System initialized");
}

// Update sensor status
void updateSensorStatus() {
    sensorStatus._2x4Present = digitalRead(_2x4_PRESENT_SENSOR) == LOW;
    sensorStatus.woodSuctionConfirm = digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == LOW;
    sensorStatus.firstCutOrWoodFwdOne = digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == HIGH;
    sensorStatus.cutMotorHomeSwitch = digitalRead(CUT_MOTOR_HOME_SWITCH) == HIGH;
    sensorStatus.feedMotorHomeSensor = digitalRead(FEED_MOTOR_HOME_SENSOR) == LOW;
    sensorStatus.reloadSwitch = digitalRead(RELOAD_SWITCH) == HIGH;
    sensorStatus.startCycleSwitch = digitalRead(START_CYCLE_SWITCH) == HIGH;
    sensorStatus.manualFeedSwitch = digitalRead(MANUAL_FEED_SWITCH) == HIGH;
}

// Update clamp status
void updateClampStatus() {
    clampStatus.feedClamp = digitalRead(FEED_CLAMP) == HIGH;
    clampStatus._2x4SecureClamp = digitalRead(_2x4_SECURE_CLAMP) == HIGH;
    clampStatus.rotationClamp = digitalRead(ROTATION_CLAMP) == HIGH;
}

// Update LED status
void updateLEDStatus() {
    ledStatus.red = digitalRead(STATUS_LED_RED) == HIGH;
    ledStatus.yellow = digitalRead(STATUS_LED_YELLOW) == HIGH;
    ledStatus.green = digitalRead(STATUS_LED_GREEN) == HIGH;
    ledStatus.blue = digitalRead(STATUS_LED_BLUE) == HIGH;
}

// Update network info
void updateNetworkInfo() {
    networkInfo.wifiSignal = WiFi.RSSI();
    networkInfo.freeHeap = ESP.getFreeHeap();
    networkInfo.freePSRAM = ESP.getFreePsram();
    networkInfo.temperature = temperatureRead(); // ESP32-S3 temperature sensor
    networkInfo.uptime = millis() - systemStartTime;
}

// Helper function to format timestamp
String formatTimestamp(unsigned long timestamp) {
    unsigned long seconds = timestamp / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    seconds = seconds % 60;
    minutes = minutes % 60;
    hours = hours % 24;
    
    char timeStr[10];
    snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu", hours, minutes, seconds);
    return String(timeStr);
}

// Add event to log
void addEventToLog(const String& event) {
    String timestamp = formatTimestamp(millis());
    String logEntry = "[" + timestamp + "] " + event;
    
    eventLog.events[eventLog.eventIndex] = logEntry;
    eventLog.eventIndex = (eventLog.eventIndex + 1) % 50;
    if (eventLog.eventCount < 50) {
        eventLog.eventCount++;
    }
}

// Calculate time-based performance metrics
void calculateTimeBasedMetrics() {
    unsigned long currentTime = millis();
    unsigned long systemUptime = currentTime - systemStartTime;
    
    // Reset counters
    performanceMetrics.cycles1Min = 0;
    performanceMetrics.cycles3Min = 0;
    performanceMetrics.cycles5Min = 0;
    performanceMetrics.cycles15Min = 0;
    performanceMetrics.cycles30Min = 0;
    
    // Count cycles within each time window
    for (int i = 0; i < performanceMetrics.cycleTimestampCount; i++) {
        unsigned long cycleTime = performanceMetrics.cycleTimestamps[i];
        unsigned long timeDiff = currentTime - cycleTime;
        
        if (timeDiff <= 60000) {        // 1 minute
            performanceMetrics.cycles1Min++;
        }
        if (timeDiff <= 180000) {       // 3 minutes
            performanceMetrics.cycles3Min++;
        }
        if (timeDiff <= 300000) {       // 5 minutes
            performanceMetrics.cycles5Min++;
        }
        if (timeDiff <= 900000) {       // 15 minutes
            performanceMetrics.cycles15Min++;
        }
        if (timeDiff <= 1800000) {      // 30 minutes
            performanceMetrics.cycles30Min++;
        }
    }
    
    // Calculate averages (cycles per minute) - only if enough time has passed
    if (systemUptime >= 60000) {        // 1 minute
        performanceMetrics.avgCycles1Min = (float)performanceMetrics.cycles1Min / 1.0;
    } else {
        performanceMetrics.avgCycles1Min = -1; // Indicate not ready
    }
    
    if (systemUptime >= 180000) {       // 3 minutes
        performanceMetrics.avgCycles3Min = (float)performanceMetrics.cycles3Min / 3.0;
    } else {
        performanceMetrics.avgCycles3Min = -1; // Indicate not ready
    }
    
    if (systemUptime >= 300000) {       // 5 minutes
        performanceMetrics.avgCycles5Min = (float)performanceMetrics.cycles5Min / 5.0;
    } else {
        performanceMetrics.avgCycles5Min = -1; // Indicate not ready
    }
    
    if (systemUptime >= 900000) {       // 15 minutes
        performanceMetrics.avgCycles15Min = (float)performanceMetrics.cycles15Min / 15.0;
    } else {
        performanceMetrics.avgCycles15Min = -1; // Indicate not ready
    }
    
    if (systemUptime >= 1800000) {      // 30 minutes
        performanceMetrics.avgCycles30Min = (float)performanceMetrics.cycles30Min / 30.0;
    } else {
        performanceMetrics.avgCycles30Min = -1; // Indicate not ready
    }
}

// Update performance metrics
void updatePerformanceMetrics(unsigned long cycleTime) {
    // Don't update lastCycleTime here - it will be updated continuously by updateTimeSinceLastCycle()
    // This function is called when a cycle completes, so we just record the completion time
    performanceMetrics.totalCycles++;
    
    // Store cycle timestamp
    performanceMetrics.cycleTimestamps[performanceMetrics.cycleTimestampIndex] = millis();
    performanceMetrics.cycleTimestampIndex = (performanceMetrics.cycleTimestampIndex + 1) % 100;
    if (performanceMetrics.cycleTimestampCount < 100) {
        performanceMetrics.cycleTimestampCount++;
    }
    
    // Update average cycle time (simple moving average) - convert to seconds
    float cycleTimeSeconds = (float)cycleTime / 1000.0;
    if (performanceMetrics.averageCycleTime == 0) {
        performanceMetrics.averageCycleTime = cycleTimeSeconds;
    } else {
        performanceMetrics.averageCycleTime = (performanceMetrics.averageCycleTime + cycleTimeSeconds) / 2;
    }
    
    // Calculate time-based metrics
    calculateTimeBasedMetrics();
    
    // Calculate efficiency (simplified)
    unsigned long totalTime = millis() - systemStartTime;
    float productiveTime = performanceMetrics.totalCycles * performanceMetrics.averageCycleTime * 1000.0; // Convert back to ms for calculation
    if (totalTime > 0) {
        performanceMetrics.efficiency = productiveTime / totalTime * 100.0;
    }
}

// Update error count
void updateErrorCount(const String& errorType) {
    errorInfo.lastError = errorType;
    errorInfo.lastErrorTime = millis();
    errorInfo.errorCount++;
    
    // Add to error history
    String timestamp = formatTimestamp(millis());
    String errorEntry = "[" + timestamp + "] " + errorType;
    errorInfo.errorHistory[errorInfo.errorHistoryIndex] = errorEntry;
    errorInfo.errorHistoryIndex = (errorInfo.errorHistoryIndex + 1) % 10;
    
    addEventToLog("ERROR: " + errorType);
}

// Broadcast system status
void broadcastSystemStatus() {
    if (ws.getClients().size() > 0) {
        systemStatus.currentState = getStateName(getCurrentState());
        systemStatus.previousState = getStateName(getPreviousState());
        systemStatus.systemHealth = getSystemHealth();
        systemStatus.uptime = millis() - systemStartTime;
        
        JsonDocument doc;
        doc["type"] = "system_status";
        doc["currentState"] = systemStatus.currentState;
        doc["previousState"] = systemStatus.previousState;
        doc["systemHealth"] = systemStatus.systemHealth;
        doc["uptime"] = systemStatus.uptime;
        doc["lastStateChange"] = systemStatus.lastStateChange;
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast sensor status
void broadcastSensorStatus() {
    if (ws.getClients().size() > 0) {
        updateSensorStatus();
        
        JsonDocument doc;
        doc["type"] = "sensor_status";
        doc["_2x4Present"] = sensorStatus._2x4Present;
        doc["woodSuctionConfirm"] = sensorStatus.woodSuctionConfirm;
        doc["firstCutOrWoodFwdOne"] = sensorStatus.firstCutOrWoodFwdOne;
        doc["cutMotorHomeSwitch"] = sensorStatus.cutMotorHomeSwitch;
        doc["feedMotorHomeSensor"] = sensorStatus.feedMotorHomeSensor;
        doc["reloadSwitch"] = sensorStatus.reloadSwitch;
        doc["startCycleSwitch"] = sensorStatus.startCycleSwitch;
        doc["manualFeedSwitch"] = sensorStatus.manualFeedSwitch;
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast clamp status
void broadcastClampStatus() {
    if (ws.getClients().size() > 0) {
        updateClampStatus();
        
        JsonDocument doc;
        doc["type"] = "clamp_status";
        doc["feedClamp"] = clampStatus.feedClamp;
        doc["_2x4SecureClamp"] = clampStatus._2x4SecureClamp;
        doc["rotationClamp"] = clampStatus.rotationClamp;
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast LED status
void broadcastLEDStatus() {
    if (ws.getClients().size() > 0) {
        updateLEDStatus();
        
        JsonDocument doc;
        doc["type"] = "led_status";
        doc["red"] = ledStatus.red;
        doc["yellow"] = ledStatus.yellow;
        doc["green"] = ledStatus.green;
        doc["blue"] = ledStatus.blue;
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast performance metrics
void broadcastPerformanceMetrics() {
    if (ws.getClients().size() > 0) {
        // Recalculate time-based metrics before broadcasting
        calculateTimeBasedMetrics();
        
        JsonDocument doc;
        doc["type"] = "performance_metrics";
        doc["lastCycleTime"] = performanceMetrics.lastCycleTime;
        doc["averageCycleTime"] = performanceMetrics.averageCycleTime;
        doc["totalCycles"] = performanceMetrics.totalCycles;
        
        // Time-based totals (removed - no longer needed in dashboard)
        
        // Time-based averages (cycles per minute)
        doc["avgCycles1Min"] = performanceMetrics.avgCycles1Min;
        doc["avgCycles3Min"] = performanceMetrics.avgCycles3Min;
        doc["avgCycles5Min"] = performanceMetrics.avgCycles5Min;
        doc["avgCycles15Min"] = performanceMetrics.avgCycles15Min;
        doc["avgCycles30Min"] = performanceMetrics.avgCycles30Min;
        
        doc["totalUptime"] = performanceMetrics.totalUptime;
        doc["systemUptime"] = millis() - systemStartTime;
        doc["efficiency"] = performanceMetrics.efficiency;
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast error status
void broadcastErrorStatus() {
    if (ws.getClients().size() > 0) {
        JsonDocument doc;
        doc["type"] = "error_status";
        doc["lastError"] = errorInfo.lastError;
        doc["lastErrorTime"] = errorInfo.lastErrorTime;
        doc["errorCount"] = errorInfo.errorCount;
        
        JsonArray history = doc["errorHistory"].to<JsonArray>();
        for (int i = 0; i < 10; i++) {
            if (errorInfo.errorHistory[i].length() > 0) {
                history.add(errorInfo.errorHistory[i]);
            }
        }
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast network info
void broadcastNetworkInfo() {
    if (ws.getClients().size() > 0) {
        updateNetworkInfo();
        
        JsonDocument doc;
        doc["type"] = "network_info";
        doc["wifiSignal"] = networkInfo.wifiSignal;
        doc["freeHeap"] = networkInfo.freeHeap;
        doc["freePSRAM"] = networkInfo.freePSRAM;
        doc["temperature"] = networkInfo.temperature;
        doc["uptime"] = networkInfo.uptime;
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

// Broadcast event log
void broadcastEventLog() {
    if (ws.getClients().size() > 0) {
        JsonDocument doc;
        doc["type"] = "event_log";
        
        JsonArray events = doc["events"].to<JsonArray>();
        for (int i = 0; i < eventLog.eventCount; i++) {
            int index = (eventLog.eventIndex - eventLog.eventCount + i + 50) % 50;
            if (eventLog.events[index].length() > 0) {
                events.add(eventLog.events[index]);
            }
        }
        
        String message;
        serializeJson(doc, message);
        ws.textAll(message);
    }
}

void setupWebSocketDashboard() {
    // Initialize SPIFFS for serving files (if needed in future)
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
    }
    
    // Initialize dashboard data
    initializeDashboardData();
    
    // Setup web server to serve the dashboard
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", dashboardHTML);
    });
    
    // Setup WebSocket event handler
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    
    // Start the server
    server.begin();
    Serial.println("Enhanced Web server and WebSocket started on port 80");
    
    // Print IP address for easy access
    Serial.print("Dashboard available at: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT: {
            Serial.printf("Client %u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            
            // Send all current data to newly connected client
            String counterMessage = "{\"type\":\"counter\",\"count\":" + String(cuttingCycleCount) + "}";
            client->text(counterMessage);
            
            // Send all status data
            broadcastSystemStatus();
            broadcastSensorStatus();
            broadcastClampStatus();
            broadcastLEDStatus();
            broadcastPerformanceMetrics();
            broadcastErrorStatus();
            broadcastNetworkInfo();
            broadcastEventLog();
            break;
        }
            
        case WS_EVT_DISCONNECT:
            Serial.printf("Client %u disconnected\n", client->id());
            break;
            
        case WS_EVT_DATA: {
            // Handle incoming data
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0;
                String message = (char*)data;
                
                // Parse JSON message
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, message);
                
                if (!error) {
                    String type = doc["type"];
                    if (type == "ping") {
                        // Respond with pong
                        String pongMessage = "{\"type\":\"pong\"}";
                        client->text(pongMessage);
                    } else if (type == "request_all_data") {
                        // Send all current data
                        broadcastSystemStatus();
                        broadcastSensorStatus();
                        broadcastClampStatus();
                        broadcastLEDStatus();
                        broadcastPerformanceMetrics();
                        broadcastErrorStatus();
                        broadcastNetworkInfo();
                        broadcastEventLog();
                    }
                }
            }
            break;
        }
            
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void startCuttingCycleTimer() {
    lastCycleStartTime = millis();
}

void updateTimeSinceLastCycle() {
    // Update the time since last cycle completion
    // Show time since last cycle if we've completed at least one cycle
    if (lastCycleCompletionTime > 0 && cuttingCycleCount > 0) {
        performanceMetrics.lastCycleTime = (float)(millis() - lastCycleCompletionTime) / 1000.0;
    } else {
        performanceMetrics.lastCycleTime = 0; // Show 0 for no cycles completed
    }
    
    // Debug output (remove after testing)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 5000) { // Every 5 seconds
        Serial.print("Debug - cuttingCycleCount: ");
        Serial.print(cuttingCycleCount);
        Serial.print(", lastCycleCompletionTime: ");
        Serial.print(lastCycleCompletionTime);
        Serial.print(", lastCycleTime: ");
        Serial.println(performanceMetrics.lastCycleTime);
        lastDebugTime = millis();
    }
}

void incrementCuttingCycleCounter() {
    cuttingCycleCount++;
    unsigned long cycleTime = millis() - lastCycleStartTime;
    lastCycleCompletionTime = millis(); // Record when this cycle completed
    
    Serial.print("Cutting cycle completed. Total cycles: ");
    Serial.println(cuttingCycleCount);
    
    // Update performance metrics
    updatePerformanceMetrics(cycleTime);
    
    // Add event to log
    addEventToLog("Cutting cycle completed - " + String((float)cycleTime / 1000.0, 1) + "s");
    
    // Broadcast the updated count to all connected clients
    broadcastCuttingCycleCount();
    broadcastPerformanceMetrics();
}

unsigned long getCuttingCycleCount() {
    return cuttingCycleCount;
}

void broadcastCuttingCycleCount() {
    if (ws.getClients().size() > 0) {
        String message = "{\"type\":\"counter\",\"count\":" + String(cuttingCycleCount) + "}";
        ws.textAll(message);
    }
}

// Function to be called from state machine when state changes
void onStateChange(SystemState newState) {
    SystemState oldState = getCurrentState();
    lastStateChangeTime = millis();
    
    addEventToLog("State changed: " + getStateName(oldState) + " -> " + getStateName(newState));
    
    // Update system status
    systemStatus.previousState = getStateName(oldState);
    systemStatus.currentState = getStateName(newState);
    systemStatus.systemHealth = getSystemHealth();
    systemStatus.lastStateChange = lastStateChangeTime;
    
    // Broadcast updated status
    broadcastSystemStatus();
}

// Function to be called when errors occur
void onErrorOccurred(const String& errorType) {
    updateErrorCount(errorType);
    broadcastErrorStatus();
}

// Function to be called periodically to update all status
void updateDashboardStatus() {
    // Update time since last cycle continuously
    updateTimeSinceLastCycle();
    
    // Always broadcast performance metrics to keep "time since last cycle" updated
    broadcastPerformanceMetrics();
    
    // Only update other status when motors are not moving to avoid timing interference
    if (getCurrentState() != CUTTING) {
        broadcastSystemStatus();
        broadcastSensorStatus();
        broadcastClampStatus();
        broadcastLEDStatus();
        broadcastNetworkInfo();
    }
}
