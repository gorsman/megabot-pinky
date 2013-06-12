#include <Arduino.h>
#include "MshieldMotor.h"

PinkyMotor::PinkyMotor(uint8_t motor_id) {
  if (motor_id == PINKY_MOTOR_RIGHT) {
    speed_pin_ = 6;
    direction_pin_ = 7;
  } else {
    speed_pin_ = 5;
    direction_pin_ = 4;
  }
  pinMode(direction_pin_, OUTPUT);
  pinMode(speed_pin_, OUTPUT);
}

void PinkyMotor::setSpeed(int8_t speed) {
  if (speed >= 0) {
    digitalWrite(direction_pin_, HIGH);
    analogWrite(speed_pin_, ((uint8_t) speed) << 1);
  } else {
    if (speed == -128)
      speed = -127;
    digitalWrite(direction_pin_, LOW);
    analogWrite(speed_pin_, ((uint8_t) -speed) << 1);
  }
}
