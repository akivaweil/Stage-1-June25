#include <Arduino.h>
#include "StateMachine/General_Functions.h"

// STATUS LEDS
// Single-color show* functions also turn the other three LEDs off so the
// panel always shows exactly one active color. turn*Off functions only
// affect their own LED.

void showRedLed() {
  static bool lastRedLedState = false;
  digitalWrite(STATUS_LED_RED, HIGH);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastRedLedState) {
    lastRedLedState = true;
  }
}

void turnRedLedOff() {
  static bool lastRedLedState = true;
  digitalWrite(STATUS_LED_RED, LOW);
  if (lastRedLedState) {
    lastRedLedState = false;
  }
}

void showYellowLed() {
  static bool lastYellowLedState = false;
  digitalWrite(STATUS_LED_YELLOW, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastYellowLedState) {
    lastYellowLedState = true;
  }
}

void turnYellowLedOff() {
  static bool lastYellowLedState = true;
  digitalWrite(STATUS_LED_YELLOW, LOW);
  if (lastYellowLedState) {
    lastYellowLedState = false;
  }
}

void showGreenLed() {
  static bool lastGreenLedState = false;
  digitalWrite(STATUS_LED_GREEN, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastGreenLedState) {
    lastGreenLedState = true;
  }
}

void turnGreenLedOff() {
  static bool lastGreenLedState = true;
  digitalWrite(STATUS_LED_GREEN, LOW);
  if (lastGreenLedState) {
    lastGreenLedState = false;
  }
}

void showBlueLed() {
  static bool lastBlueLedState = false;
  digitalWrite(STATUS_LED_BLUE, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  if (!lastBlueLedState) {
    lastBlueLedState = true;
  }
}

void turnBlueLedOff() {
  static bool lastBlueLedState = true;
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (lastBlueLedState) {
    lastBlueLedState = false;
  }
}

// NO-WOOD LED WAVE PATTERN
const unsigned long LED_WAVE_INTERVAL_MS = 200;    // Time between each LED starting
const unsigned long LED_ON_DURATION_MS = 250;      // How long each LED stays on

// Static variables for LED wave pattern (shared between states)
static unsigned long ledWaveOnTime[4] = {0, 0, 0, 0};     // When each LED turned on (0 = off)
static unsigned long ledWaveTurnOnTime[4] = {0, 0, 0, 0}; // When each LED should next turn on
static bool ledWaveInitialized = false;

void handleNoWoodLedWavePattern(float speedMultiplier) {
    // Scale timing constants by speedMultiplier (>1 = slower)
    unsigned long interval = (unsigned long)(LED_WAVE_INTERVAL_MS * speedMultiplier);
    unsigned long onDuration = (unsigned long)(LED_ON_DURATION_MS * speedMultiplier);

    // Initialize wave pattern if not already done
    if (!ledWaveInitialized) {
        unsigned long now = millis();
        ledWaveTurnOnTime[0] = now;                 // Red starts immediately
        ledWaveTurnOnTime[1] = now + interval;      // Yellow starts after red
        ledWaveTurnOnTime[2] = now + (interval * 2); // Green starts after yellow
        ledWaveTurnOnTime[3] = now + (interval * 3); // Blue starts after green
        ledWaveOnTime[0] = 0;
        ledWaveOnTime[1] = 0;
        ledWaveOnTime[2] = 0;
        ledWaveOnTime[3] = 0;
        ledWaveInitialized = true;
    }

    // Handle LED wave pattern: sequential red -> yellow -> green -> blue, max 2 on at once
    unsigned long now = millis();
    for (int i = 0; i < 4; i++) {
        // Check if LED should turn on
        if (now >= ledWaveTurnOnTime[i] && ledWaveOnTime[i] == 0) {
            switch (i) {
                case 0: digitalWrite(STATUS_LED_RED, HIGH); break;
                case 1: digitalWrite(STATUS_LED_YELLOW, HIGH); break;
                case 2: digitalWrite(STATUS_LED_GREEN, HIGH); break;
                case 3: digitalWrite(STATUS_LED_BLUE, HIGH); break;
            }
            ledWaveOnTime[i] = now;
            // Schedule next turn on (continuous wave)
            ledWaveTurnOnTime[i] = now + onDuration + (interval * 3);
        }

        // Check if LED should turn off (after being on for onDuration)
        if (ledWaveOnTime[i] > 0 && now - ledWaveOnTime[i] >= onDuration) {
            switch (i) {
                case 0: digitalWrite(STATUS_LED_RED, LOW); break;
                case 1: digitalWrite(STATUS_LED_YELLOW, LOW); break;
                case 2: digitalWrite(STATUS_LED_GREEN, LOW); break;
                case 3: digitalWrite(STATUS_LED_BLUE, LOW); break;
            }
            ledWaveOnTime[i] = 0;
        }
    }
}

void resetNoWoodLedWavePattern(bool preserveYellowLed) {
    ledWaveInitialized = false;
    digitalWrite(STATUS_LED_RED, LOW);
    if (!preserveYellowLed) {
        digitalWrite(STATUS_LED_YELLOW, LOW);
    }
    digitalWrite(STATUS_LED_GREEN, LOW);
    digitalWrite(STATUS_LED_BLUE, LOW);
}

void allLedsOff() {
    turnRedLedOff();
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

void handleHomingLedBlink() {
    static unsigned long blinkTimer = 0;
    if (millis() - blinkTimer > 500) {
        blinkState = !blinkState;
        if (blinkState) showBlueLed(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}
