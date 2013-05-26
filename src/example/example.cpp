#include "example.h"

#include "Arduino.h"

void blink_led(uint8_t pin, long duration) {
	digitalWrite(pin, HIGH);   // set the LED on
	delay(duration);           // wait for a second
	digitalWrite(pin, LOW);    // set the LED off
	delay(duration);           // wait for a second
}
