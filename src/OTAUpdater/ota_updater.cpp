#include "OTAUpdater/ota_updater.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* *********************** OTA UPDATER IMPLEMENTATION *********************
//* ************************************************************************
// Handles WiFi connection and Over-The-Air updates for the ESP32.

const char* ssid = "Everwood";
const char* password = "Everwood-Staff";

// LED functions for OTA progress indication
void otaAllLedsOff() {
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
}

void otaUpdateProgressLEDs(unsigned int progress, unsigned int total) {
  float percentage = (float)progress / (float)total * 100.0;
  
  // Turn off all LEDs first
  otaAllLedsOff();
  
  // Light appropriate LED based on progress
  if (percentage < 25.0) {
    // 0-25%: Red LED
    digitalWrite(STATUS_LED_RED, HIGH);
  } else if (percentage < 50.0) {
    // 25-50%: Yellow LED
    digitalWrite(STATUS_LED_YELLOW, HIGH);
  } else if (percentage < 75.0) {
    // 50-75%: Green LED
    digitalWrite(STATUS_LED_GREEN, HIGH);
  } else {
    // 75-100%: Blue LED
    digitalWrite(STATUS_LED_BLUE, HIGH);
  }
}

void onOTAStart() {
    // Start updating
    //serial.println("Start updating " + type);
    
    // Turn on LED to indicate OTA upload in progress
    //serial.println("OTA Upload started - LED progress indication active");
}

void onOTAProgress(unsigned int progress, unsigned int total) {
    // Calculate percentage
    int percent = (progress / (total / 100));
    
    // Flash LED based on progress
    if (percent % 10 == 0) {
        // Flash every 10%
        digitalWrite(STATUS_LED_BLUE, !digitalRead(STATUS_LED_BLUE));
    }
}

void onOTAComplete() {
    // OTA upload completed
    //serial.println("\nOTA Upload completed!");
    
    // Flash LED sequence to indicate completion
    for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_BLUE, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_BLUE, LOW);
        delay(100);
    }
    //serial.println("OTA completion flash sequence finished");
}

void onOTAError(ota_error_t error) {
    // Handle OTA errors
    switch (error) {
        case OTA_AUTH_ERROR:
            //serial.println("Auth Failed");
            break;
        case OTA_BEGIN_ERROR:
            //serial.println("Begin Failed");
            break;
        case OTA_CONNECT_ERROR:
            //serial.println("Connect Failed");
            break;
        case OTA_RECEIVE_ERROR:
            //serial.println("Receive Failed");
            break;
        case OTA_END_ERROR:
            //serial.println("End Failed");
            break;
    }
    
    // Flash red LED to indicate error
    for (int i = 0; i < 10; i++) {
        digitalWrite(STATUS_LED_RED, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_RED, LOW);
        delay(100);
    }
    //serial.println("OTA error indication completed");
}

void initOTA() {
    // Initialize OTA
    //serial.println("OTA Initialized");
    //serial.print("IP address: ");
    //serial.println(WiFi.localIP());
    
    // Set up OTA callbacks
    ArduinoOTA.onStart(onOTAStart);
    ArduinoOTA.onProgress(onOTAProgress);
    // Note: onComplete might not be available in all ArduinoOTA versions
    // ArduinoOTA.onComplete(onOTAComplete);
    ArduinoOTA.onError(onOTAError);
    
    // Start OTA service
    ArduinoOTA.begin();
}

void handleOTA() {
  ArduinoOTA.handle();
} 