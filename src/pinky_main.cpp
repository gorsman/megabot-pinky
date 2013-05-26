/*
 * Pinky entry point.
 */

#include "Arduino.h"

#include "example/example.h"

uint8_t pin = 13;
long duration = 1000;

void setup() {
  pinMode(pin, OUTPUT);
}

void loop() {
	blink_led(pin, duration);
}

