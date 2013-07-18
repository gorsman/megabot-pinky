#include "motor_speed_measurer.h"

#include "Arduino.h"

LowPassMotorSpeedMeasurer::LowPassMotorSpeedMeasurer(Motor& motor)
    : motor(motor), lastTicks(0), instantSpeed(0), speed(0) {
  lastUpdate = millis();
}

bool LowPassMotorSpeedMeasurer::update(long curTime) {
  long timeDelta = curTime - lastUpdate;
  if (timeDelta < MOTOR_SPEED_MEASURER_UPDATE_PERIOD) {
    return false;
  }
  int32_t curTicks = motor.getTicks();
  instantSpeed = (curTicks - lastTicks) * 1000 / timeDelta;
  speed = (speed + instantSpeed) >> 1;
  if (speed == -1) speed = 0;  // forcing the speed to reach 0
  lastUpdate = curTime;
  lastTicks = curTicks;
  return true;
}

int32_t LowPassMotorSpeedMeasurer::getSpeed() {
  return speed;
}


// ------------------------------------------------------------------

SlidingWindowMotorSpeedMeasurer::SlidingWindowMotorSpeedMeasurer(Motor& motor)
    : motor(motor), speed(0) {
  lastUpdate = millis();
  for (int i = 0; i < MOTOR_SPEED_MEASURER_SLIDING_WINDOW_SIZE; ++i) {
    ticksLog[i].time = lastUpdate;
    ticksLog[i].ticks = 0;
  }
  index = 0;
}

int32_t SlidingWindowMotorSpeedMeasurer::getSpeed() {
  return speed;
}

bool SlidingWindowMotorSpeedMeasurer::update(long curTime) {
  if (curTime - lastUpdate < MOTOR_SPEED_MEASURER_UPDATE_PERIOD) {
    return false;
  }
  int32_t curTicks = motor.getTicks();
  int8_t i = index + 1;
  if (i >= MOTOR_SPEED_MEASURER_SLIDING_WINDOW_SIZE) {
    i = 0;
  }
  long timeDelta = curTime - ticksLog[i].time;
  if (timeDelta <= 0) {
    // Wtf?! o_O
    return false;
  }
  int32_t ticksDelta = curTicks - ticksLog[i].ticks;
  if (ticksDelta == 0) {
    speed = 0;
  } else {
    speed = ticksDelta * 1000 / timeDelta;
  }

  ticksLog[i].time = curTime;
  ticksLog[i].ticks = curTicks;
  index = i;

  lastUpdate = curTime;
  return true;
}
