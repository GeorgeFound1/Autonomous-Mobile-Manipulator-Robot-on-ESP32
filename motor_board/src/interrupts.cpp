#include <Arduino.h>
#include "pin_modes.hpp"
#include "target_search.hpp"
#include "interrupts.hpp"

// Left encoder: compare A/B to detect direction
void IRAM_ATTR readLeftEncoder() {
  if (digitalRead(encoderLeftA) != digitalRead(encoderLeftB)) {
    leftTicks--;
  } else {
    leftTicks++;
  }
}

// Right encoder: compare A/B to detect direction
void IRAM_ATTR readRightEncoder() {
  if (digitalRead(encoderRightA) == digitalRead(encoderRightB)) {
    rightTicks--;
  } else {
    rightTicks++;
  }
}

// Ultrasonic echo: start timer on HIGH, stop on LOW
void IRAM_ATTR readUltrasonic() {
  if (digitalRead(ECHO) == HIGH) {
        echoStartTime = micros(); 
    } else {
        echoDuration = micros() - echoStartTime;
        newReadingAvailable = true;
    }
}