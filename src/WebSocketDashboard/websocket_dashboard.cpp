#include "WebSocketDashboard/websocket_dashboard.h"
#include "WebSocketDashboard/dashboard_html.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "StateMachine/STATES/States_Config.h"
#include "Config/Pins_Definitions.h"
#include "Config/Motor_Config.h"
#include "Config/config.h"
#include <ArduinoJson.h>
#include <EEPROM.h>

//* ************************************************************************
//* ********************** WEBSOCKET DASHBOARD ****************************
//* ************************************************************************
// Enhanced websocket dashboard for comprehensive table saw monitoring
// Updates only when motors are not moving to avoid timing interference

// Global variables
AsyncWebServer server(WEB_SERVER_PORT);
AsyncWebSocket ws("/ws");
unsigned long systemStartTime = 0;
unsigned long lastCycleStartTime = 0;
unsigned long lastCycleCompletionTime = 0;
unsigned long lastStateChangeTime = 0;
unsigned long reloadTimeStart = 0;
float reloadTimeSeconds = 0.0;
bool reloadTimeActive = false;

// Dashboard configuration variables - these are loaded from EEPROM/config and can be modified via dashboard
float CUT_TRAVEL_DISTANCE = 9.2;
float FEED_TRAVEL_DISTANCE = 3.43;

// EEPROM configuration constants
const int CONFIG_EEPROM_SIZE = 2048; // Increase EEPROM size for configuration
const int CONFIG_OFFSET = 0; // Configuration starts at beginning of extended EEPROM

// Enhanced dashboard data structures
SystemStatus systemStatus;
SensorStatus sensorStatus;
ClampStatus clampStatus;
LEDStatus ledStatus;
PerformanceMetrics performanceMetrics;
ErrorInfo errorInfo;
NetworkInfo networkInfo;
EventLog eventLog;
SerialLog serialLog;

// Previous values for change detection
SystemStatus previousSystemStatus;
SensorStatus previousSensorStatus;
ClampStatus previousClampStatus;
LEDStatus previousLEDStatus;
PerformanceMetrics previousPerformanceMetrics;
ErrorInfo previousErrorInfo;
NetworkInfo previousNetworkInfo;
EventLog previousEventLog;
SerialLog previousSerialLog;
float previousReloadTime = 0.0;

// Forward declarations
void updateSensorStatus();
void updateClampStatus();
void updateLEDStatus();
void updateNetworkInfo();

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
        case RELOAD: return "RELOAD";
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

// Change detection helper functions
bool hasSystemStatusChanged() {
    return (systemStatus.currentState != previousSystemStatus.currentState ||
            systemStatus.previousState != previousSystemStatus.previousState ||
            systemStatus.systemHealth != previousSystemStatus.systemHealth ||
            systemStatus.uptime != previousSystemStatus.uptime ||
            systemStatus.lastStateChange != previousSystemStatus.lastStateChange);
}

bool hasSensorStatusChanged() {
    return (sensorStatus._2x4Present != previousSensorStatus._2x4Present ||
            sensorStatus.woodSuctionConfirm != previousSensorStatus.woodSuctionConfirm ||
            sensorStatus.firstCutOrWoodFwdOne != previousSensorStatus.firstCutOrWoodFwdOne ||
            sensorStatus.cutMotorHomeSwitch != previousSensorStatus.cutMotorHomeSwitch ||
            sensorStatus.feedMotorHomeSensor != previousSensorStatus.feedMotorHomeSensor ||
            sensorStatus.reloadSwitch != previousSensorStatus.reloadSwitch ||
            sensorStatus.startCycleSwitch != previousSensorStatus.startCycleSwitch ||
            sensorStatus.manualFeedSwitch != previousSensorStatus.manualFeedSwitch);
}

bool hasClampStatusChanged() {
    return (clampStatus.feedClamp != previousClampStatus.feedClamp ||
            clampStatus._2x4SecureClamp != previousClampStatus._2x4SecureClamp ||
            clampStatus.rotationClamp != previousClampStatus.rotationClamp);
}

bool hasLEDStatusChanged() {
    return (ledStatus.red != previousLEDStatus.red ||
            ledStatus.yellow != previousLEDStatus.yellow ||
            ledStatus.green != previousLEDStatus.green ||
            ledStatus.blue != previousLEDStatus.blue);
}

bool hasPerformanceMetricsChanged() {
    return (performanceMetrics.lastCycleTime != previousPerformanceMetrics.lastCycleTime ||
            performanceMetrics.cycles1Min != previousPerformanceMetrics.cycles1Min ||
            performanceMetrics.cycles3Min != previousPerformanceMetrics.cycles3Min ||
            performanceMetrics.cycles5Min != previousPerformanceMetrics.cycles5Min ||
            performanceMetrics.cycles15Min != previousPerformanceMetrics.cycles15Min ||
            performanceMetrics.cycles30Min != previousPerformanceMetrics.cycles30Min ||
            performanceMetrics.avgCycles1Min != previousPerformanceMetrics.avgCycles1Min ||
            performanceMetrics.avgCycles3Min != previousPerformanceMetrics.avgCycles3Min ||
            performanceMetrics.avgCycles5Min != previousPerformanceMetrics.avgCycles5Min ||
            performanceMetrics.avgCycles15Min != previousPerformanceMetrics.avgCycles15Min ||
            performanceMetrics.avgCycles30Min != previousPerformanceMetrics.avgCycles30Min ||
            performanceMetrics.totalUptime != previousPerformanceMetrics.totalUptime ||
            performanceMetrics.efficiency != previousPerformanceMetrics.efficiency);
}

bool hasErrorStatusChanged() {
    return (errorInfo.lastError != previousErrorInfo.lastError ||
            errorInfo.lastErrorTime != previousErrorInfo.lastErrorTime ||
            errorInfo.errorCount != previousErrorInfo.errorCount ||
            errorInfo.cutMotorErrorCount != previousErrorInfo.cutMotorErrorCount ||
            errorInfo.suctionErrorCount != previousErrorInfo.suctionErrorCount);
}

bool hasNetworkInfoChanged() {
    return (networkInfo.wifiSignal != previousNetworkInfo.wifiSignal ||
            networkInfo.freeHeap != previousNetworkInfo.freeHeap ||
            networkInfo.freePSRAM != previousNetworkInfo.freePSRAM ||
            networkInfo.temperature != previousNetworkInfo.temperature ||
            networkInfo.uptime != previousNetworkInfo.uptime);
}

bool hasEventLogChanged() {
    return (eventLog.eventCount != previousEventLog.eventCount ||
            eventLog.eventIndex != previousEventLog.eventIndex);
}



// Configuration structure for all settings
struct ConfigurationData {
    // Motor Configuration
    float CUT_MOTOR_STEPS_PER_INCH;
    float FEED_MOTOR_STEPS_PER_INCH;
    float CUT_TRAVEL_DISTANCE;
    float FEED_TRAVEL_DISTANCE;
    float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
    float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
    
    // Cut Motor Speed Settings
    float CUT_MOTOR_NORMAL_SPEED;
    float CUT_MOTOR_NORMAL_ACCELERATION;
    float CUT_MOTOR_RETURN_SPEED;
    float CUT_MOTOR_HOMING_SPEED;
    
    // Feed Motor Speed Settings
    float FEED_MOTOR_NORMAL_SPEED;
    float FEED_MOTOR_NORMAL_ACCELERATION;
    float FEED_MOTOR_RETURN_SPEED;
    float FEED_MOTOR_RETURN_ACCELERATION;
    float FEED_MOTOR_HOMING_SPEED;
    
    // Timing Configuration
    unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
    unsigned long CUT_HOME_TIMEOUT;
    unsigned long TA_SIGNAL_DURATION;
    
    // Operational Constants
    float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
    float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
    float TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;
    
    // Safety Constants
    unsigned long ROTATION_SERVO_RETURN_DELAY_MS;
    
    // Motor Control Constants
    float FEED_MOTOR_RETURN_DISTANCE;
    // FEED_MOTOR_OFFSET_FROM_SENSOR removed - now hardcoded in config file
    
    // Timing Constants
    unsigned long CUT_MOTOR_RECOVERY_TIMEOUT_MS;
    unsigned long CUT_MOTOR_VERIFICATION_DELAY_MS;
    unsigned long SENSOR_STABILIZATION_DELAY_MS;
    float SUCTION_SENSOR_CHECK_DISTANCE_INCHES;
    
    // Version and checksum
    uint32_t version;
    uint32_t checksum;
};

// Default configuration values
ConfigurationData getDefaultConfiguration() {
    ConfigurationData config;
    
    // Motor Configuration
    config.CUT_MOTOR_STEPS_PER_INCH = 500.0;
    config.FEED_MOTOR_STEPS_PER_INCH = 1000.0;
    config.CUT_TRAVEL_DISTANCE = 9.2;
    config.FEED_TRAVEL_DISTANCE = 3.43;
    config.CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1;
    config.CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4;
    
    // Cut Motor Speed Settings - use Motor_Config defaults
    config.CUT_MOTOR_NORMAL_SPEED = CUT_MOTOR_NORMAL_SPEED;
    config.CUT_MOTOR_NORMAL_ACCELERATION = CUT_MOTOR_NORMAL_ACCELERATION;
    config.CUT_MOTOR_RETURN_SPEED = CUT_MOTOR_RETURN_SPEED;
    config.CUT_MOTOR_HOMING_SPEED = CUT_MOTOR_HOMING_SPEED;

    // Feed Motor Speed Settings - use Motor_Config defaults
    config.FEED_MOTOR_NORMAL_SPEED = FEED_MOTOR_NORMAL_SPEED;
    config.FEED_MOTOR_NORMAL_ACCELERATION = FEED_MOTOR_NORMAL_ACCELERATION;
    config.FEED_MOTOR_RETURN_SPEED = FEED_MOTOR_RETURN_SPEED;
    config.FEED_MOTOR_RETURN_ACCELERATION = FEED_MOTOR_RETURN_ACCELERATION;
    config.FEED_MOTOR_HOMING_SPEED = FEED_MOTOR_HOMING_SPEED;
    
    // Timing Configuration - use Motor_Config defaults
    config.ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
    config.CUT_HOME_TIMEOUT = CUT_HOME_TIMEOUT;
    config.TA_SIGNAL_DURATION = TA_SIGNAL_DURATION;

    // Operational Constants - use Motor_Config defaults
    config.ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
    config.ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
    config.TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;

    // Safety Constants - use Motor_Config defaults
    config.ROTATION_SERVO_RETURN_DELAY_MS = ROTATION_SERVO_RETURN_DELAY_MS;

    // Motor Control Constants - use Motor_Config defaults
    config.FEED_MOTOR_RETURN_DISTANCE = FEED_MOTOR_RETURN_DISTANCE;
    // FEED_MOTOR_OFFSET_FROM_SENSOR removed - now hardcoded in config file

    // Timing Constants - use Motor_Config defaults
    config.CUT_MOTOR_RECOVERY_TIMEOUT_MS = CUT_MOTOR_RECOVERY_TIMEOUT_MS;
    config.CUT_MOTOR_VERIFICATION_DELAY_MS = CUT_MOTOR_VERIFICATION_DELAY_MS;
    config.SENSOR_STABILIZATION_DELAY_MS = SENSOR_STABILIZATION_DELAY_MS;
    config.SUCTION_SENSOR_CHECK_DISTANCE_INCHES = SUCTION_SENSOR_CHECK_DISTANCE_INCHES;
    
    config.version = 1;
    config.checksum = 0; // Will be calculated
    
    return config;
}

// Apply configuration to global variables
void applyConfiguration(const ConfigurationData& config) {
    // Motor Configuration
    CUT_MOTOR_STEPS_PER_INCH = config.CUT_MOTOR_STEPS_PER_INCH;
    FEED_MOTOR_STEPS_PER_INCH = config.FEED_MOTOR_STEPS_PER_INCH;
    CUT_TRAVEL_DISTANCE = config.CUT_TRAVEL_DISTANCE;
    FEED_TRAVEL_DISTANCE = config.FEED_TRAVEL_DISTANCE;
    CUT_MOTOR_INCREMENTAL_MOVE_INCHES = config.CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
    CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = config.CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
    
    // Cut Motor Speed Settings
    CUT_MOTOR_NORMAL_SPEED = config.CUT_MOTOR_NORMAL_SPEED;
    CUT_MOTOR_NORMAL_ACCELERATION = config.CUT_MOTOR_NORMAL_ACCELERATION;
    CUT_MOTOR_RETURN_SPEED = config.CUT_MOTOR_RETURN_SPEED;
    CUT_MOTOR_HOMING_SPEED = config.CUT_MOTOR_HOMING_SPEED;
    
    // Feed Motor Speed Settings
    FEED_MOTOR_NORMAL_SPEED = config.FEED_MOTOR_NORMAL_SPEED;
    FEED_MOTOR_NORMAL_ACCELERATION = config.FEED_MOTOR_NORMAL_ACCELERATION;
    FEED_MOTOR_RETURN_SPEED = config.FEED_MOTOR_RETURN_SPEED;
    FEED_MOTOR_RETURN_ACCELERATION = config.FEED_MOTOR_RETURN_ACCELERATION;
    FEED_MOTOR_HOMING_SPEED = config.FEED_MOTOR_HOMING_SPEED;
    
    // Timing Configuration
    ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = config.ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
    CUT_HOME_TIMEOUT = config.CUT_HOME_TIMEOUT;
    TA_SIGNAL_DURATION = config.TA_SIGNAL_DURATION;
    
    // Operational Constants
    ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = config.ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
    ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = config.ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
    TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = config.TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;
    
    // Safety Constants
    ROTATION_SERVO_RETURN_DELAY_MS = config.ROTATION_SERVO_RETURN_DELAY_MS;
    
    // Motor Control Constants
    FEED_MOTOR_RETURN_DISTANCE = config.FEED_MOTOR_RETURN_DISTANCE;
    // FEED_MOTOR_OFFSET_FROM_SENSOR removed - now hardcoded in config file
    
    // Timing Constants
    CUT_MOTOR_RECOVERY_TIMEOUT_MS = config.CUT_MOTOR_RECOVERY_TIMEOUT_MS;
    CUT_MOTOR_VERIFICATION_DELAY_MS = config.CUT_MOTOR_VERIFICATION_DELAY_MS;
    SENSOR_STABILIZATION_DELAY_MS = config.SENSOR_STABILIZATION_DELAY_MS;
    SUCTION_SENSOR_CHECK_DISTANCE_INCHES = config.SUCTION_SENSOR_CHECK_DISTANCE_INCHES;
}

// Calculate checksum for configuration
uint32_t calculateChecksum(const ConfigurationData& config) {
    uint32_t checksum = 0;
    const uint8_t* data = (const uint8_t*)&config;
    size_t size = sizeof(config) - sizeof(config.checksum); // Exclude checksum field
    
    for (size_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

// Configuration management functions
void loadConfiguration() {
    EEPROM.begin(CONFIG_EEPROM_SIZE);
    
    ConfigurationData config;
    EEPROM.get(CONFIG_OFFSET, config);
    
    // Validate configuration
    bool isValid = true;
    
    // Check version
    if (config.version != 1) {
        isValid = false;
        Serial.println("Configuration version mismatch, using defaults");
    }
    
    // Check checksum
    uint32_t calculatedChecksum = calculateChecksum(config);
    if (config.checksum != calculatedChecksum) {
        isValid = false;
        Serial.println("Configuration checksum invalid, using defaults");
    }
    
    // Check for reasonable value ranges
    if (config.FEED_TRAVEL_DISTANCE < 0.1 || config.FEED_TRAVEL_DISTANCE > 10.0) {
        isValid = false;
        Serial.println("FEED_TRAVEL_DISTANCE out of range, using defaults");
    }
    
    if (!isValid) {
        config = getDefaultConfiguration();
        saveConfiguration();
        Serial.println("Configuration loaded: Using default values");
    } else {
        Serial.println("Configuration loaded: Using stored values");
    }
    
    applyConfiguration(config);
}

void saveConfiguration() {
    EEPROM.begin(CONFIG_EEPROM_SIZE);
    
    ConfigurationData config;
    
    // Get current values
    config.CUT_MOTOR_STEPS_PER_INCH = CUT_MOTOR_STEPS_PER_INCH;
    config.FEED_MOTOR_STEPS_PER_INCH = FEED_MOTOR_STEPS_PER_INCH;
    config.CUT_TRAVEL_DISTANCE = CUT_TRAVEL_DISTANCE;
    config.FEED_TRAVEL_DISTANCE = FEED_TRAVEL_DISTANCE;
    config.CUT_MOTOR_INCREMENTAL_MOVE_INCHES = CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
    config.CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
    config.CUT_MOTOR_NORMAL_SPEED = CUT_MOTOR_NORMAL_SPEED;
    config.CUT_MOTOR_NORMAL_ACCELERATION = CUT_MOTOR_NORMAL_ACCELERATION;
    config.CUT_MOTOR_RETURN_SPEED = CUT_MOTOR_RETURN_SPEED;
    config.CUT_MOTOR_HOMING_SPEED = CUT_MOTOR_HOMING_SPEED;
    config.FEED_MOTOR_NORMAL_SPEED = FEED_MOTOR_NORMAL_SPEED;
    config.FEED_MOTOR_NORMAL_ACCELERATION = FEED_MOTOR_NORMAL_ACCELERATION;
    config.FEED_MOTOR_RETURN_SPEED = FEED_MOTOR_RETURN_SPEED;
    config.FEED_MOTOR_RETURN_ACCELERATION = FEED_MOTOR_RETURN_ACCELERATION;
    config.FEED_MOTOR_HOMING_SPEED = FEED_MOTOR_HOMING_SPEED;
    config.ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
    config.CUT_HOME_TIMEOUT = CUT_HOME_TIMEOUT;
    config.TA_SIGNAL_DURATION = TA_SIGNAL_DURATION;
    config.ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
    config.ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;
    config.TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES = TA_SIGNAL_EARLY_ACTIVATION_OFFSET_INCHES;
    config.ROTATION_SERVO_RETURN_DELAY_MS = ROTATION_SERVO_RETURN_DELAY_MS;
    config.FEED_MOTOR_RETURN_DISTANCE = FEED_MOTOR_RETURN_DISTANCE;
    // FEED_MOTOR_OFFSET_FROM_SENSOR removed - now hardcoded in config file
    config.CUT_MOTOR_RECOVERY_TIMEOUT_MS = CUT_MOTOR_RECOVERY_TIMEOUT_MS;
    config.CUT_MOTOR_VERIFICATION_DELAY_MS = CUT_MOTOR_VERIFICATION_DELAY_MS;
    config.SENSOR_STABILIZATION_DELAY_MS = SENSOR_STABILIZATION_DELAY_MS;
    config.SUCTION_SENSOR_CHECK_DISTANCE_INCHES = SUCTION_SENSOR_CHECK_DISTANCE_INCHES;
    
    config.version = 1;
    config.checksum = calculateChecksum(config);
    
    EEPROM.put(CONFIG_OFFSET, config);
    EEPROM.commit();
    Serial.println("Configuration saved");
}

float getFeedTravelDistance() {
    return FEED_TRAVEL_DISTANCE;
}

float getCutTravelDistance() {
    return CUT_TRAVEL_DISTANCE;
}

void setFeedTravelDistance(float value) {
    if (value >= 0.1 && value <= 10.0) {
        FEED_TRAVEL_DISTANCE = value;
        saveConfiguration();
        addEventToLog("Configuration updated: FEED_TRAVEL_DISTANCE = " + String(value));
    }
}

// Initialize dashboard data structures
void initializeDashboardData() {
    systemStartTime = millis();
    lastStateChangeTime = millis();
    
    
    // Load configuration
    loadConfiguration();
    
    // Initialize system status
    systemStatus.currentState = getStateName(getCurrentState());
    systemStatus.previousState = getStateName(getPreviousState());
    systemStatus.systemHealth = getSystemHealth();
    systemStatus.uptime = 0;
    systemStatus.lastStateChange = lastStateChangeTime;
    
    // Initialize performance metrics
    performanceMetrics.lastCycleTime = 0;
    
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
    errorInfo.cutMotorErrorCount = 0;
    errorInfo.suctionErrorCount = 0;
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
    
    // Initialize current sensor, clamp, and LED statuses
    updateSensorStatus();
    updateClampStatus();
    updateLEDStatus();
    updateNetworkInfo();
    
    // Initialize previous values for change detection
    previousSystemStatus = systemStatus;
    previousSensorStatus = sensorStatus;
    previousClampStatus = clampStatus;
    previousLEDStatus = ledStatus;
    previousPerformanceMetrics = performanceMetrics;
    previousErrorInfo = errorInfo;
    previousNetworkInfo = networkInfo;
    previousEventLog = eventLog;
}

// Update sensor status
void updateSensorStatus() {
    sensorStatus._2x4Present = digitalRead(_2x4_PRESENT_SENSOR) == LOW;
    sensorStatus.woodSuctionConfirm = digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == LOW;
    sensorStatus.firstCutOrWoodFwdOne = digitalRead(FIRST_CUT_OR_WOOD_FWD_ONE) == LOW;
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
    // Calculate time since last cycle completion (in seconds)
    // Show time since last cycle if we've completed at least one cycle
    if (lastCycleCompletionTime > 0) {
        performanceMetrics.lastCycleTime = (float)(millis() - lastCycleCompletionTime) / 1000.0;
    } else {
        performanceMetrics.lastCycleTime = 0; // Show 0 for no cycles completed
    }
    
    // Store cycle timestamp
    performanceMetrics.cycleTimestamps[performanceMetrics.cycleTimestampIndex] = millis();
    performanceMetrics.cycleTimestampIndex = (performanceMetrics.cycleTimestampIndex + 1) % 100;
    if (performanceMetrics.cycleTimestampCount < 100) {
        performanceMetrics.cycleTimestampCount++;
    }
    
    // Calculate time-based metrics
    calculateTimeBasedMetrics();
}

// Update error count
void updateErrorCount(const String& errorType) {
    errorInfo.lastError = errorType;
    errorInfo.lastErrorTime = millis();
    errorInfo.errorCount++;
    
    // Increment specific error type counters
    if (errorType.indexOf("Cut motor") >= 0 || errorType.indexOf("cut motor") >= 0) {
        errorInfo.cutMotorErrorCount++;
    } else if (errorType.indexOf("Wood suction") >= 0 || errorType.indexOf("suction") >= 0) {
        errorInfo.suctionErrorCount++;
    }
    
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
        
        // Only broadcast if something changed
        if (hasSystemStatusChanged()) {
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
            
            // Update previous values
            previousSystemStatus = systemStatus;
        }
    }
}

// Broadcast sensor status
void broadcastSensorStatus() {
    if (ws.getClients().size() > 0) {
        updateSensorStatus();
        
        // Only broadcast if something changed
        if (hasSensorStatusChanged()) {
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
            
            // Update previous values
            previousSensorStatus = sensorStatus;
        }
    }
}

// Broadcast clamp status
void broadcastClampStatus() {
    if (ws.getClients().size() > 0) {
        updateClampStatus();
        
        // Only broadcast if something changed
        if (hasClampStatusChanged()) {
            JsonDocument doc;
            doc["type"] = "clamp_status";
            doc["feedClamp"] = clampStatus.feedClamp;
            doc["_2x4SecureClamp"] = clampStatus._2x4SecureClamp;
            doc["rotationClamp"] = clampStatus.rotationClamp;
            
            String message;
            serializeJson(doc, message);
            ws.textAll(message);
            
            // Update previous values
            previousClampStatus = clampStatus;
        }
    }
}

// Broadcast LED status
void broadcastLEDStatus() {
    if (ws.getClients().size() > 0) {
        updateLEDStatus();
        
        // Only broadcast if something changed
        if (hasLEDStatusChanged()) {
            JsonDocument doc;
            doc["type"] = "led_status";
            doc["red"] = ledStatus.red;
            doc["yellow"] = ledStatus.yellow;
            doc["green"] = ledStatus.green;
            doc["blue"] = ledStatus.blue;
            
            String message;
            serializeJson(doc, message);
            ws.textAll(message);
            
            // Update previous values
            previousLEDStatus = ledStatus;
        }
    }
}

// Broadcast performance metrics
void broadcastPerformanceMetrics() {
    if (ws.getClients().size() > 0) {
        // Recalculate time-based metrics before broadcasting
        calculateTimeBasedMetrics();
        
        // Check if reload time changed (for active reload timer)
        float currentReloadTime = getReloadTime();
        bool reloadTimeChanged = (currentReloadTime != previousReloadTime);
        
        // Only broadcast if something changed
        if (hasPerformanceMetricsChanged() || reloadTimeChanged) {
            JsonDocument doc;
            doc["type"] = "performance_metrics";
            doc["reloadTime"] = currentReloadTime;
            
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
            
            // Update previous values
            previousPerformanceMetrics = performanceMetrics;
            previousReloadTime = currentReloadTime;
        }
    }
}

// Broadcast error status
void broadcastErrorStatus() {
    if (ws.getClients().size() > 0) {
        // Only broadcast if something changed
        if (hasErrorStatusChanged()) {
            JsonDocument doc;
            doc["type"] = "error_status";
            doc["lastError"] = errorInfo.lastError;
            doc["lastErrorTime"] = errorInfo.lastErrorTime;
            doc["errorCount"] = errorInfo.errorCount;
            doc["cutMotorErrorCount"] = errorInfo.cutMotorErrorCount;
            doc["suctionErrorCount"] = errorInfo.suctionErrorCount;
            
            JsonArray history = doc["errorHistory"].to<JsonArray>();
            for (int i = 0; i < 10; i++) {
                if (errorInfo.errorHistory[i].length() > 0) {
                    history.add(errorInfo.errorHistory[i]);
                }
            }
            
            String message;
            serializeJson(doc, message);
            ws.textAll(message);
            
            // Update previous values
            previousErrorInfo = errorInfo;
        }
    }
}

// Broadcast network info
void broadcastNetworkInfo() {
    if (ws.getClients().size() > 0) {
        updateNetworkInfo();
        
        // Only broadcast if something changed (check every 10 seconds for network info)
        static unsigned long lastNetworkBroadcast = 0;
        if (hasNetworkInfoChanged() || (millis() - lastNetworkBroadcast > 10000)) {
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
            
            // Update previous values
            previousNetworkInfo = networkInfo;
            lastNetworkBroadcast = millis();
        }
    }
}

// Broadcast event log
void broadcastEventLog() {
    if (ws.getClients().size() > 0) {
        // Only broadcast if something changed
        if (hasEventLogChanged()) {
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
            
            // Update previous values
            previousEventLog = eventLog;
        }
    }
}

// Add serial log message
void addSerialLog(const String& message) {
    // Add timestamp to message
    unsigned long currentTime = millis();
    String timestampedMessage = "[" + String(currentTime) + "ms] " + message;
    
    // Store in circular buffer
    serialLog.logs[serialLog.logIndex] = timestampedMessage;
    serialLog.logIndex = (serialLog.logIndex + 1) % 100;
    if (serialLog.logCount < 100) {
        serialLog.logCount++;
    }
    
    // Also print to Serial
    Serial.println(message);
}

// Broadcast serial log
void broadcastSerialLog() {
    if (ws.getClients().size() > 0) {
        // Always broadcast serial logs (they're always changing)
        JsonDocument doc;
        doc["type"] = "serial_log";
        
        JsonArray logs = doc["logs"].to<JsonArray>();
        for (int i = 0; i < serialLog.logCount; i++) {
            int index = (serialLog.logIndex - serialLog.logCount + i + 100) % 100;
            if (serialLog.logs[index].length() > 0) {
                logs.add(serialLog.logs[index]);
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
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", dashboardHTML);
        response->addHeader("Connection", "close");
        request->send(response);
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
            Serial.println("WebSocket connection established successfully");
            
            // Send all current data to newly connected client
            
            // Send all status data
            broadcastSystemStatus();
            broadcastSensorStatus();
            broadcastClampStatus();
            broadcastLEDStatus();
            broadcastPerformanceMetrics();
            broadcastErrorStatus();
            broadcastNetworkInfo();
            broadcastEventLog();
            broadcastSerialLog();
            break;
        }
            
        case WS_EVT_DISCONNECT:
            Serial.printf("Client %u disconnected\n", client->id());
            Serial.println("WebSocket client disconnected");
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
                        broadcastSerialLog();
                    } else if (type == "update_config") {
                        String configKey = doc["key"];
                        JsonDocument response;
                        response["type"] = "config_updated";
                        response["key"] = configKey;
                        
                        // Handle configuration settings
                        if (configKey == "cut_travel_distance") {
                            float newValue = doc["value"];
                            if (newValue >= 0.1 && newValue <= 20.0) {
                                CUT_TRAVEL_DISTANCE = newValue;
                                saveConfiguration();
                                response["value"] = newValue;
                                addEventToLog("Configuration updated: CUT_TRAVEL_DISTANCE = " + String(newValue));
                            } else {
                                response["error"] = "Value out of range (0.1-20.0)";
                            }
                        } else if (configKey == "feed_travel_distance") {
                            float newValue = doc["value"];
                            if (newValue >= 0.1 && newValue <= 10.0) {
                                FEED_TRAVEL_DISTANCE = newValue;
                                saveConfiguration();
                                response["value"] = newValue;
                                addEventToLog("Configuration updated: FEED_TRAVEL_DISTANCE = " + String(newValue));
                            } else {
                                response["error"] = "Value out of range (0.1-10.0)";
                            }
                        } else if (configKey == "cut_motor_normal_speed") {
                            float newValue = doc["value"];
                            if (newValue >= 100 && newValue <= 5000) {
                                CUT_MOTOR_NORMAL_SPEED = newValue;
                                saveConfiguration();
                                response["value"] = newValue;
                                addEventToLog("Configuration updated: CUT_MOTOR_NORMAL_SPEED = " + String(newValue));
                            } else {
                                response["error"] = "Value out of range (100-5000)";
                            }
                        } else {
                            response["error"] = "Unknown configuration key";
                        }
                        
                        String message;
                        serializeJson(response, message);
                        client->text(message);
                        
                    } else if (type == "request_config") {
                        String configKey = doc["key"];
                        JsonDocument response;
                        response["type"] = "config_value";
                        response["key"] = configKey;
                        
                        // Return current values for configuration settings
                        if (configKey == "cut_travel_distance") {
                            response["value"] = CUT_TRAVEL_DISTANCE;
                        } else if (configKey == "feed_travel_distance") {
                            response["value"] = FEED_TRAVEL_DISTANCE;
                        } else if (configKey == "cut_motor_normal_speed") {
                            response["value"] = CUT_MOTOR_NORMAL_SPEED;
                        } else {
                            response["error"] = "Unknown configuration key";
                        }
                        
                        String message;
                        serializeJson(response, message);
                        client->text(message);
                        
                    } else if (type == "request_all_config") {
                        // Send all configuration values at once
                        JsonDocument response;
                        response["type"] = "all_config";
                        response["cut_travel_distance"] = CUT_TRAVEL_DISTANCE;
                        response["feed_travel_distance"] = FEED_TRAVEL_DISTANCE;
                        response["cut_motor_normal_speed"] = CUT_MOTOR_NORMAL_SPEED;
                        
                        String message;
                        serializeJson(response, message);
                        client->text(message);
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
    // Reset time since last cycle to 0 when starting a new cycle
    performanceMetrics.lastCycleTime = 0;
}

void updateTimeSinceLastCycle() {
    // Only update time since last cycle when system is in IDLE state
    // During cutting and returning sequences, the system is actively working, not idle
    SystemState currentState = getCurrentState();
    
    if (currentState != IDLE) {
        return; // Don't update time counter during active operations
    }
    
    // Update the time since last cycle completion
    // Show time since last cycle if we've completed at least one cycle
    if (lastCycleCompletionTime > 0) {
        performanceMetrics.lastCycleTime = (float)(millis() - lastCycleCompletionTime) / 1000.0;
    } else {
        performanceMetrics.lastCycleTime = 0; // Show 0 for no cycles completed
    }
}

void incrementCuttingCycleCounter() {
    unsigned long cycleTime = millis() - lastCycleStartTime;
    lastCycleCompletionTime = millis(); // Record when this cycle completed
    
    
    Serial.println("Cutting cycle completed.");
    
    // Update performance metrics
    updatePerformanceMetrics(cycleTime);
    
    // Add event to log
    addEventToLog("Cutting cycle completed - " + String((float)cycleTime / 1000.0, 1) + "s");
    
    // Broadcast the updated performance metrics
    broadcastPerformanceMetrics();
}

// Start reload time timer when exiting RETURNING_NO_2x4 state
void startReloadTimer() {
    reloadTimeStart = millis();
    reloadTimeActive = true;
    reloadTimeSeconds = 0.0;
    Serial.println("Reload timer started");
}

// Stop reload time timer when entering FEED_FIRST_CUT or CUTTING state
void stopReloadTimer() {
    if (reloadTimeActive) {
        reloadTimeSeconds = (float)(millis() - reloadTimeStart) / 1000.0;
        reloadTimeActive = false;
        Serial.println("Reload timer stopped - Time: " + String(reloadTimeSeconds, 1) + "s");
        addEventToLog("Reload completed - " + String(reloadTimeSeconds, 1) + "s");
    }
}

// Get current reload time (either active timer or last completed time)
float getReloadTime() {
    if (reloadTimeActive) {
        return (float)(millis() - reloadTimeStart) / 1000.0;
    } else {
        return reloadTimeSeconds;
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
    
    // Skip immediate broadcast during critical motor transitions to avoid blocking
    // The periodic update will catch it within 1 second
    bool isCriticalTransition = (oldState == CUTTING && 
                                 (newState == RETURNING_YES_2x4 || newState == RETURNING_NO_2x4));
    
    if (!isCriticalTransition) {
        broadcastSystemStatus();
    }
}

// Function to be called when errors occur
void onErrorOccurred(const String& errorType) {
    updateErrorCount(errorType);
    broadcastErrorStatus();
}

// Function to be called periodically to update all status
void updateDashboardStatus() {
    // Update reload time continuously when timer is active
    if (reloadTimeActive) {
        // Force broadcast performance metrics when reload timer is running (time changes frequently)
        broadcastPerformanceMetrics();
    }
    
    // Update time since last cycle continuously
    updateTimeSinceLastCycle();
    
    
    // Only broadcast when motors are not moving to avoid timing interference
    // Now each broadcast function checks for changes internally
    if (getCurrentState() != CUTTING) {
        broadcastSystemStatus();
        broadcastSensorStatus();
        broadcastClampStatus();
        broadcastLEDStatus();
        broadcastNetworkInfo();
        broadcastPerformanceMetrics();
    }
}
