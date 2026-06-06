#include "OTA/OTA_Upload.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Config/Pins_Definitions.h"

// OTA Updater Implementation
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
  
  // Light LEDs progressively to show upload progress (like a progress bar)
  // 0-25%: Red LED
  digitalWrite(STATUS_LED_RED, HIGH);
  
  if (percentage >= 25.0) {
    // 25-50%: Red + Yellow LEDs
    digitalWrite(STATUS_LED_YELLOW, HIGH);
  }
  
  if (percentage >= 50.0) {
    // 50-75%: Red + Yellow + Green LEDs
    digitalWrite(STATUS_LED_GREEN, HIGH);
  }
  
  if (percentage >= 75.0) {
    // 75-100%: All 4 LEDs (Red + Yellow + Green + Blue)
    digitalWrite(STATUS_LED_BLUE, HIGH);
  }
}

void setupOTA() {
  //serial.println("Booting for OTA...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Bounded connect attempt, then proceed so the machine still boots and runs
  // on its locally-saved EEPROM settings if the network/TA is down. WiFi keeps
  // retrying in the background after the loop falls through.
  const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
  const uint32_t WIFI_CONNECT_POLL_MS = 250;
  WiFi.setAutoReconnect(true);
  uint32_t wifiConnectStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - wifiConnectStart < WIFI_CONNECT_TIMEOUT_MS) {
    delay(WIFI_CONNECT_POLL_MS);
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
      // Force the TA signal LOW before flashing. OTA suspends the main loop's
      // signal-timing handler, so an in-flight pulse would otherwise stay HIGH
      // for the duration of the upload and confuse the Transfer Arm.
      digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
      extern bool taSignalActive;
      extern bool taSignalDelayPending;
      extern unsigned long taSignalOffTime;
      taSignalActive = false;
      taSignalDelayPending = false;
      taSignalOffTime = millis();

      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS, ensure SPIFFS is mounted via SPIFFS.begin()
      //serial.println("Start updating " + type);

      // Flash all LEDs 3 times to clearly indicate upload start
      for(int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_RED, HIGH);
        digitalWrite(STATUS_LED_YELLOW, HIGH);
        digitalWrite(STATUS_LED_GREEN, HIGH);
        digitalWrite(STATUS_LED_BLUE, HIGH);
        delay(100);
        otaAllLedsOff();
        delay(100);
      }
      
      // Start with red LED for 0% progress
      digitalWrite(STATUS_LED_RED, HIGH);
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

  Serial.println("OTA Initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void handleOTA() {
  ArduinoOTA.handle();
} 