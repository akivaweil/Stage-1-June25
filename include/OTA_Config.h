#ifndef OTA_CONFIG_H
#define OTA_CONFIG_H

//* ************************************************************************
//* ************************ OTA CONFIGURATION ***************************
//* ************************************************************************
// Configuration settings for Over-The-Air (OTA) updates

// WiFi Configuration
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;

// OTA Configuration
extern const char* OTA_HOSTNAME;
extern const char* OTA_PASSWORD;
extern const float OTA_PORT;

// Connection timeouts
extern const unsigned long WIFI_TIMEOUT;  // 30 seconds
extern const unsigned long OTA_TIMEOUT;   // 10 seconds

#endif  // OTA_CONFIG_H 