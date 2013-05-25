/*
 * Pinky entry point.
 */

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

uint8_t pin = 13;
long duration = 1000;

void setup() {
  pinMode(pin, OUTPUT);
}

void loop() {
  digitalWrite(pin, HIGH);   // set the LED on
  delay(duration);           // wait for a second
  digitalWrite(pin, LOW);    // set the LED off
  delay(duration);           // wait for a second
}

