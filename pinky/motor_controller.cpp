#include "motor_controller.h"

#include "Arduino.h"

// #define POWER_INCREMENT 10
// #define TICKS_DELTA_THRESHOLD 2
#define TICKS_DELTA_ACCELERATION_THRESHOLD 5

// Time in millis we allow the motor to catch up with the target distance.
// This affects the real speed we want to be shooting for.
#define CATCHUP_TIME 1000


namespace {

const int32_t MAX_POWER_STEP = MOTOR_MAX_POWER >> 1;

#define SPEED_PER_TICK 1000 / MOTOR_CONTROLLER_UPDATE_PERIOD
const int32_t ACCELERATION_SPEED_DELTA_THRESHOLD = 5 * SPEED_PER_TICK;

inline bool isStableSpeed(int32_t lastSpeed, int32_t currentSpeed) {
  return abs(lastSpeed - currentSpeed) <= ACCELERATION_SPEED_DELTA_THRESHOLD;
}

}  // namespace


MotorController::MotorController(Motor& motor)
  : motor(motor),
    speedMeasurer(motor),
    targetSpeed(0),
    internalTargetSpeed(0),
    motorPower(0),
    updateDelay(MOTOR_CONTROLLER_UPDATE_PERIOD),
    lastCheckpoint(0),
    checkpointTicks(0),
    powerStep(0),
    lastUpdate(0),
    lastTicks(0),
    lastInstantSpeed(0),
    maxed(false) {
}

Motor& MotorController::getMotor() {
  return motor;
}

void MotorController::setTargetSpeed(int32_t targetSpeed, bool preserveOdometry) {
  long curTime = millis();
  checkpointTicks = motor.getTicks();
  if (preserveOdometry) {
    checkpointTicks = checkpointTicks + targetSpeed * (curTime - lastCheckpoint) / 1000;
  }
  lastCheckpoint = curTime;

  this->targetSpeed = targetSpeed;
  updateInternal(curTime, checkpointTicks);
}

int32_t MotorController::getTargetSpeed() {
  return targetSpeed;
}

int32_t MotorController::getInternalTargetSpeed() {
  return internalTargetSpeed;
}

int32_t MotorController::getSpeed() {
  return speedMeasurer.getSpeed();
}

bool MotorController::isMaxed() {
  return maxed;
}

bool MotorController::update() {
  long curTime = millis();

  speedMeasurer.update(curTime);
  if (curTime - lastUpdate < updateDelay) {
    // Too early to do an update.
    return false;
  }
  int32_t curTicks = motor.getTicks();
  updateInternal(curTime, curTicks);
  return true;
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

  internalTargetSpeed = targetSpeed;
  int32_t instantSpeed = ticksDelta * 1000 / timeDelta;

  if (targetSpeed == 0) {
    if (motorPower != 0) {
      motorPower = 0;
      updateMotorPower(0);
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
    internalTargetSpeed = targetSpeed + controlTicksDelta * 1000 / CATCHUP_TIME;
    
    updateDelay = MOTOR_CONTROLLER_UPDATE_PERIOD;
    if (isStableSpeed(lastInstantSpeed, instantSpeed) || abs(powerStep) <= 1) {
      powerStep = computePowerDelta(motorPower, instantSpeed, internalTargetSpeed);
      motorPower += powerStep;
      updateDelay = 100;
    } else if (powerStep > 0) {
      // We're currently accelerating forwards.
      if (instantSpeed > internalTargetSpeed) {
        // We overshoot and still accelerating - using binary-search-like approach to adjust the power.
        powerStep = - (powerStep >> 1);
        if (powerStep == 0) {
          powerStep = -1;
        }
        motorPower += powerStep;
      }
    } else {
      // We're currently accelerating backwards.
      if (instantSpeed < internalTargetSpeed) {
        // We overshoot and still accelerating - using binary-search-like approach to adjust the power.
        powerStep = (-powerStep) >> 1;
        if (powerStep == 0) {
          powerStep = 1;
        }
        motorPower += powerStep;
      }
    }
  }
  updateMotorPower(instantSpeed - lastInstantSpeed);

  lastUpdate = curTime;
  lastTicks = curTicks;
  lastInstantSpeed = instantSpeed;
}

void MotorController::updateMotorPower(int32_t instantSpeedDelta) {
  if (targetSpeed >= 0) {
    if (motorPower < 0) {
        motorPower = 0;
    }
  } else {
    if (motorPower > 0) {
      motorPower = 0;
    }
  }
  if (motorPower >= MOTOR_MAX_POWER) {
    motorPower = MOTOR_MAX_POWER;
    maxed |= instantSpeedDelta <= 0;
  } else if (motorPower <= -MOTOR_MAX_POWER) {
    motorPower = -MOTOR_MAX_POWER;
    maxed |= instantSpeedDelta <= 0;
  } else {
    maxed = false;
  }
  if (motorPower != motor.getPower()) {
    motor.setPower(motorPower);
  }
}
