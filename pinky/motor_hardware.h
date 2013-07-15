#ifndef MOTOR_HARDWARE_H_
#define MOTOR_HARDWARE_H_

#include <inttypes.h>

// Motor implementation for the Monster Moto Shield (DEV-10182) from Sparkfun Electronics.
// https://www.sparkfun.com/products/10182
class MotorHardware {
public:
  MotorHardware(int8_t pinA, int8_t pinB, int8_t pinPwm, volatile int32_t& ticksCount);

  int32_t getTicks();

  void setPower(int8_t power);
  int8_t getPower();

  static MotorHardware LEFT;
  static MotorHardware RIGHT;

private:
  // Power set from outside.
  int8_t power;
  
  int8_t pinA;
  int8_t pinB;
  int8_t pinPwm;
  
  volatile int32_t& ticksCount;
  
  void motorGo(uint8_t direct, uint8_t pwm);
};

#endif  // MOTOR_HARDWARE_H_
