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

void setupOTA() {
  //serial.println("Booting for OTA...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("stage1-esp32s3");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS, ensure SPIFFS is mounted via SPIFFS.begin()
      //serial.println("Start updating " + type);
      otaAllLedsOff(); // Clear all LEDs at start
      digitalWrite(STATUS_LED_RED, HIGH); // Start with red LED
      //serial.println("OTA Upload started - LED progress indication active");
    })
    .onEnd([]() {
      //serial.println("\nOTA Upload completed!");
      otaAllLedsOff(); // Clear LEDs
      // Briefly flash all LEDs to indicate completion
      for(int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_RED, HIGH);
        digitalWrite(STATUS_LED_YELLOW, HIGH);
        digitalWrite(STATUS_LED_GREEN, HIGH);
        digitalWrite(STATUS_LED_BLUE, HIGH);
        delay(200);
        otaAllLedsOff();
        delay(200);
      }
      //serial.println("OTA completion flash sequence finished");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      otaUpdateProgressLEDs(progress, total);
    })
    .onError([](ota_error_t error) {
      if (error == OTA_AUTH_ERROR) {
        //serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        //serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        //serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        //serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        //serial.println("End Failed");
      }
      // Error indication: rapid red blinking
      otaAllLedsOff();
      for(int i = 0; i < 10; i++) {
        digitalWrite(STATUS_LED_RED, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_RED, LOW);
        delay(100);
      }
      //serial.println("OTA error indication completed");
    });

  ArduinoOTA.begin();

  //serial.println("OTA Initialized");
  //serial.print("IP address: ");
  //serial.println(WiFi.localIP());
}

void handleOTA() {
  ArduinoOTA.handle();
} 