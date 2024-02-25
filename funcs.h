#ifndef FUNCS_H
#define FUNCS_H

#include "defs.h"

void errorLoop(const __FlashStringHelper* message, const uint8_t pattern, const int wait) {
  while (1) {
    Serial.println(message);
    // Blink LED according to the pattern
    for (int i = 0; i < 8; i++) {
      digitalWrite(2, (pattern & (1 << i)) ? HIGH : LOW); // Turn LED on for '1', off for '0'
      delay(wait);
    }
  }
}

void handleErrors(int errorCode) {
  // Determine message and pattern based on error code
  switch (errorCode) {
    case 0:
      digitalWrite(2, HIGH);
      return;
    case 1: // LSM9DS1 error
      errorLoop(F("LSM9DS1 :("), 0b00001010, 200);
    case 10: // BME280 internal error
      errorLoop(F("BME280_int :("), 0b00010100, 200);
    case 20: // BME280 external error
      errorLoop(F("BME280_ext :("), 0b00010001, 200);
    case 30: // Both BME280 error
      errorLoop(F("Both BME280 :("), 0b11001100, 200);
    case 100: // SD card error
      errorLoop(F("SD :("), 0b11101100, 200);
    case 200: // File error
      errorLoop(F("File :("), 0b11101100, 200);
    default:
      errorLoop(F("Several errors :("), 0b10101010, 75);
  }
}

static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gps.available())
      nmea.process(gps.read());
  } while (millis() - start < ms);
}

#endif