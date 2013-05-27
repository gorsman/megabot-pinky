#include "motor_controller.h"

#include "Arduino.h"

// #define POWER_INCREMENT 10
// #define TICKS_DELTA_THRESHOLD 2
#define TICKS_DELTA_ACCELERATION_THRESHOLD 5

// Time in millis we allow the motor to catch up with the target distance.
// This affects the real speed we want to be shooting for.
#define CATCHUP_TIME 500


namespace {

const int32_t MAX_POWER_STEP = MOTOR_MAX_POWER >> 1;

#define SPEED_PER_TICK 1000 / MOTOR_CONTROLLER_UPDATE_PERIOD
const int32_t ACCELERATION_SPEED_DELTA_THRESHOLD = 5 * SPEED_PER_TICK;

inline bool accelerating(int32_t lastSpeed, int32_t currentSpeed) {
  return abs(lastSpeed - currentSpeed) > ACCELERATION_SPEED_DELTA_THRESHOLD;
}

}  // namespace


MotorController::MotorController(Motor& motor)
  : motor(motor),
    targetSpeed(0),
    motorPower(0),
    speed(0),
    lastCheckpoint(0),
    checkpointTicks(0),
    powerStep(0),
    lastUpdate(0),
    lastTicks(0),
    lastInstantSpeed(0) {
  ticksLog.index = 0;
  long curTime = millis();
  for (int i = 0; i < MOTOR_CONTROLLER_NUM_UPDATES_TO_AVERAGE_SPEED; ++i) {
    ticksLog.time[i] = curTime;
    ticksLog.count[i] = 0;
  }
}

void MotorController::setTargetSpeed(int32_t targetSpeed) {
  this->targetSpeed = targetSpeed;

  long curTime = millis();
  lastCheckpoint = curTime;
  checkpointTicks = motor.getTicks();
  updateInternal(curTime, checkpointTicks);
}

int32_t MotorController::getSpeed() {
  return speed;
}

bool MotorController::isMaxed() {
  return false;
}

void MotorController::update() {
  long curTime = millis();
  if (curTime - lastUpdate < MOTOR_CONTROLLER_UPDATE_PERIOD) {
    // Too early to do an update.
    return;
  }
  int32_t curTicks = motor.getTicks();
  updateSpeed(curTime, curTicks);
  updateInternal(curTime, curTicks);
}

void MotorController::updateSpeed(long curTime, int32_t curTicks) {
  int8_t i = ticksLog.index + 1;
  if (i >= MOTOR_CONTROLLER_NUM_UPDATES_TO_AVERAGE_SPEED) {
    i = 0;
  }
  long timeDelta = curTime - ticksLog.time[i];
  if (timeDelta <= 0) {
    // Wtf?! o_O
    return;
  }
  int32_t ticksDelta = curTicks - ticksLog.count[i];
  if (ticksDelta == 0) {
    speed = 0;
  } else {
    speed = ticksDelta * 1000 / timeDelta;
  }

  ticksLog.time[i] = curTime;
  ticksLog.count[i] = curTicks;
  ticksLog.index = i;
}

namespace {

inline int16_t computePowerDelta(int16_t curPower, int32_t curSpeed, int32_t targetSpeed) {
  if (curSpeed == 0) {
    return targetSpeed > 0 ? 1 : -1;
  }
  int32_t motorPowerDelta = curPower * targetSpeed / curSpeed - curPower;
  if (motorPowerDelta > MAX_POWER_STEP) {
    motorPowerDelta = MAX_POWER_STEP;
  } else if (motorPowerDelta < -MAX_POWER_STEP) {
    motorPowerDelta = -MAX_POWER_STEP;
  }
  return motorPowerDelta;
}

}  // namespace

void MotorController::updateInternal(long curTime, int32_t curTicks) {
  long timeDelta = curTime - lastUpdate;
  int32_t ticksDelta = curTicks - lastTicks;

  int32_t instantSpeed = ticksDelta * 1000 / timeDelta;

  if (targetSpeed == 0) {
    if (motorPower != 0) {
      motorPower = 0;
      updateMotorPower();
    }

    lastUpdate = curTime;
    lastTicks = curTicks;
    lastInstantSpeed = timeDelta >= MOTOR_CONTROLLER_UPDATE_PERIOD ? instantSpeed : 0;
    return;
  }

  if (lastUpdate <= lastCheckpoint) {
    if (targetSpeed > 0) {
      if (motorPower <= 0 || instantSpeed <= 0) {
        powerStep = MAX_POWER_STEP;
      } else {
        powerStep = computePowerDelta(motorPower, instantSpeed, targetSpeed);
      }
    } else {
      if (motorPower >= 0 || instantSpeed >= 0) {
        powerStep = -MAX_POWER_STEP;
      } else {
        powerStep = computePowerDelta(motorPower, instantSpeed, targetSpeed);
      }
    }
    motorPower = powerStep;
  } else {
    int32_t targetTicks = checkpointTicks + targetSpeed * (curTime - lastCheckpoint) / 1000;
    int32_t controlTicksDelta = targetTicks - curTicks;
    int32_t realTargetSpeed = targetSpeed + controlTicksDelta * 1000 / CATCHUP_TIME;

    if (!accelerating(lastInstantSpeed, instantSpeed)) {
      powerStep = computePowerDelta(motorPower, instantSpeed, realTargetSpeed);
      motorPower += powerStep;
    } else if (powerStep > 0) {
      // We're currently accelerating forwards.
      if (instantSpeed > realTargetSpeed) {
        // We overshoot and still accelerating - using binary-search-like approach to adjust the power.
        powerStep = - (powerStep >> 1);
        if (powerStep == 0) {
          powerStep = -1;
        }
        motorPower += powerStep;
      }
    } else {
      // We're currently accelerating backwards.
      if (instantSpeed < realTargetSpeed) {
        // We overshoot and still accelerating - using binary-search-like approach to adjust the power.
        powerStep = (-powerStep) >> 1;
        if (powerStep == 0) {
          powerStep = 1;
        }
        motorPower += powerStep;
      }
    }

  }
  updateMotorPower();

  lastUpdate = curTime;
  lastTicks = curTicks;
  lastInstantSpeed = instantSpeed;
}

void MotorController::updateMotorPower() {
  if (targetSpeed >= 0) {
    if (motorPower > MOTOR_MAX_POWER) {
      motorPower = MOTOR_MAX_POWER;
      // TODO: handle motor being maxed out
    } else if (motorPower < 0) {
      motorPower = 0;
    }
  } else {
    if (motorPower < -MOTOR_MAX_POWER) {
      motorPower = -MOTOR_MAX_POWER;
      // TODO: handle motor being maxed out
    } else if (motorPower > 0) {
      motorPower = 0;
    }
  }
  if (motorPower != motor.getPower()) {
    motor.setPower(motorPower);
  }
}
