#include "motor.h"

// TODO: remove this define (as it's defined in motor_emulator.h)
// #define MOTOR_EMULATOR_ENABLED

#ifdef MOTOR_EMULATOR_ENABLED

#include "Arduino.h"

// TODOs:
//   + stale power (minimum power required to move the motor)
//   + acceleration
//   - weight (affects max-speed and actual stale power)
//   - ticks measurement spread

// Motor parameters.
#define STALE_POWER 20
#define MAX_SPEED_ROUNDS_PER_MINUTE 120
#define MAX_SPEED_ACCELERATION_TIME_MILLIS 600

#define TICKS_CACHE_TIME_MILLIS 3

namespace {
static const int32_t MAX_SPEED =
    MAX_SPEED_ROUNDS_PER_MINUTE * MOTOR_TICKS_PER_ROUND / 60;  // ticks/sec ~ 6000
static const int32_t ACCELRATION =
    MAX_SPEED * 1000 / MAX_SPEED_ACCELERATION_TIME_MILLIS;  // ticks/sec^2 ~ 15000
}  // namespace

MotorEmulator MotorEmulator::LEFT;
MotorEmulator MotorEmulator::RIGHT;

int32_t distanceWithAcceleration(int32_t v0, int32_t a, int32_t t) {
  // General formula: s = a * t * t /2 + v0 * t
  //   v0 - ticks/sec
  //   a  - ticks/sec^2
  //   t  - millis
  if (t <= 100) {
    return (a * t * t / 2000 + v0 * t) / 1000;
  } else {
    // avoiding integer overflow
    int32_t tmp = t * t / 1000;
    return (a * tmp + ((v0 * t) << 1)) / 2000;
  }
}

MotorEmulator::MotorEmulator()
  : power(0),
    targetSpeed(0),
    lastUpdate(0),
    lastTicks(0),
    lastSpeed(0),
    timeToAccelerate(0),
    ticksToAccelerate(0),
    cacheTime(0),
    cachedTicks (0) {

}

int32_t MotorEmulator::getTicks() {
  long curTime = millis();
  if (curTime - cacheTime > TICKS_CACHE_TIME_MILLIS) {
    cachedTicks = getTicksInternal(curTime);
  }
	return cachedTicks;
}

void MotorEmulator::setPower(int8_t power) {
  if (power == this->power) {
    // No need to do anything - power is already set.
    return;
  }
  long curTime = millis();
  lastTicks = getTicksInternal(curTime);
  lastSpeed = getCurrentSpeed(curTime);

  lastUpdate = millis();
  this->power = power;
  this->targetSpeed = speedFromPower(power);

  if (targetSpeed != lastSpeed) {
    timeToAccelerate = abs(targetSpeed - lastSpeed) * 1000 / ACCELRATION;
    ticksToAccelerate = distanceWithAcceleration(
        lastSpeed, (targetSpeed > lastSpeed) ? ACCELRATION : -ACCELRATION, timeToAccelerate);
  } else {
    timeToAccelerate = 0;
    ticksToAccelerate = 0;
  }
}

int8_t MotorEmulator::getPower() {
  return power;
}

int32_t MotorEmulator::speedFromPower(int8_t power) {
  if (abs(power) < STALE_POWER) {
    return 0;
  }
  return power * MAX_SPEED / MOTOR_MAX_POWER;
}

int32_t MotorEmulator::getCurrentSpeed(long curTime) {
  if (targetSpeed == 0 && lastSpeed == 0) {
    return 0;
  }
  long time = curTime - lastUpdate;
  if (time >= timeToAccelerate) {
    return targetSpeed;
  } else {
    return lastSpeed + time * ((targetSpeed > lastSpeed) ? ACCELRATION : -ACCELRATION) / 1000;
  }
}

int32_t MotorEmulator::getTicksInternal(long curTime) {
  if (targetSpeed == 0 && lastSpeed == 0) {
    return lastTicks;
  }
  long time = curTime - lastUpdate;
  if (time >= timeToAccelerate) {
    int32_t ticksDelta = ticksToAccelerate + (time - timeToAccelerate) * targetSpeed / 1000;
    return lastTicks + ticksDelta;
  } else {
    int32_t ticksDelta = distanceWithAcceleration(
        lastSpeed, (targetSpeed > lastSpeed) ? ACCELRATION : -ACCELRATION, time);
    return lastTicks + ticksDelta;
  }
}

#endif  // MOTOR_EMULATOR_ENABLED
