#include "motor.h"

// TODO: remove this define (as it's defined in motor_emulator.h)
// #define MOTOR_EMULATOR_ENABLED

#ifdef MOTOR_HARDWARE_ENABLED

#include "Arduino.h"

#define DIRECT_BRAKE_VCC 0
#define DIRECT_CLOCKWISE 1
#define DIRECT_COUNTER_CLOCKWISE 2
#define DIRECT_BRAKE_GND 3

namespace {
#define HALL_LEFT_A 2
#define HALL_LEFT_B 10
#define HALL_RIGHT_A 3
#define HALL_RIGHT_B 11

volatile int32_t motorTicksLeft = 0;
volatile int32_t motorTicksRight = 0;

void motorInterruptLeft() {
 if (digitalRead(HALL_LEFT_A) ^ digitalRead(HALL_LEFT_B))
   ++motorTicksLeft;
 else
   --motorTicksLeft;
}

void motorInterruptRight() {
 if (digitalRead(HALL_RIGHT_A) ^ digitalRead(HALL_RIGHT_B))
   --motorTicksRight;
 else
   ++motorTicksRight;
}

void initHallSensors() {
  static boolean initialized = false;
  if (initialized) {
    return;
  } 
  pinMode(HALL_LEFT_A, INPUT);
  pinMode(HALL_LEFT_B, INPUT);
  attachInterrupt(0, motorInterruptLeft, CHANGE);
 
  pinMode(HALL_RIGHT_A, INPUT);
  pinMode(HALL_RIGHT_B, INPUT);
  attachInterrupt(1, motorInterruptRight, CHANGE);
  initialized = true;
}
}  // namespace

MotorHardware MotorHardware::LEFT(7, 8, 5, motorTicksLeft);
MotorHardware MotorHardware::RIGHT(4, 9, 6, motorTicksRight);

MotorHardware::MotorHardware(int8_t pinA, int8_t pinB, int8_t pinPwm, volatile int32_t& ticksCount)
  : pinA(pinA),
    pinB(pinB),
    pinPwm(pinPwm),
    ticksCount(ticksCount) {
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinPwm, OUTPUT);
  
  // Making sure there is nothing on the pins.
  setPower(0);
  
  // Lazy initialization of Hall sensors.
  initHallSensors();
}

int32_t MotorHardware::getTicks() {
  return ticksCount; 
}

void MotorHardware::setPower(int8_t power) {
  if (power == 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    analogWrite(pinPwm, 0);
  } else {
    if (power < -MOTOR_MAX_POWER) {
      // This prevents integer overflow when we take abs(power).
      power = -MOTOR_MAX_POWER;
    }
    uint8_t pwm = abs(power);
    pwm = pwm << 1;
    if (power > 0) {
      motorGo(DIRECT_CLOCKWISE, pwm);
    } else {
      motorGo(DIRECT_COUNTER_CLOCKWISE, pwm);
    }
  }
  
  this->power = power; 
}

int8_t MotorHardware::getPower() {
  return power;
}

void MotorHardware::motorGo(uint8_t direct, uint8_t pwm) {
  digitalWrite(pinA, (direct & 2) ? LOW : HIGH);
  digitalWrite(pinB, (direct & 1) ? LOW : HIGH);
  analogWrite(pinPwm, pwm);
  /*
  Serial.print("Motor GO direct=");
  Serial.print(direct);
  Serial.print(" pwm=");
  Serial.println(pwm);
  */
}


#endif MOTOR_HARDWARE_ENABLED
