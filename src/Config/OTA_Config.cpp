#include "OTA_Config.h"

//* ************************************************************************
//* ************************ OTA CONFIGURATION ***************************
//* ************************************************************************
// Configuration settings for Over-The-Air (OTA) updates

// WiFi Configuration
const char* WIFI_SSID = "Everwood";
const char* WIFI_PASSWORD = "Everwood-Staff";

// OTA Configuration
const char* OTA_HOSTNAME = "transfer-arm";
const char* OTA_PASSWORD = "transfer-arm-ota";
const float OTA_PORT = 3232.0;

// Connection timeouts
const unsigned long WIFI_TIMEOUT = 30000;  // 30 seconds
const unsigned long OTA_TIMEOUT = 10000;   // 10 seconds 