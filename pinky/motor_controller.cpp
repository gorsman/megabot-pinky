#include "motor_controller.h"

#include "Arduino.h"

// #define POWER_INCREMENT 10
// #define TICKS_DELTA_THRESHOLD 2
#define TICKS_DELTA_ACCELERATION_THRESHOLD 5

// Time in millis we allow the motor to catch up with the target distance.
// This affects the real speed we want to be shooting for.
#define CATCHUP_TIME 1000


namespace {

const int32_t MAX_POWER_STEP = Motor::MAX_POWER >> 1;

#define SPEED_PER_TICK 1000 / MOTOR_CONTROLLER_UPDATE_PERIOD
const int32_t ACCELERATION_SPEED_DELTA_THRESHOLD = 5 * SPEED_PER_TICK;

inline bool isStableSpeed(int32_t lastSpeed, int32_t currentSpeed) {
  return lastSpeed != 0 && abs(lastSpeed - currentSpeed) <= ACCELERATION_SPEED_DELTA_THRESHOLD;
}

}  // namespace

MotorController::Checkpoint::Checkpoint()
  : time(0),
    targetSpeed(0),
    targetTicks(0) {}

MotorController::State::State()
  : time(0),
    ticks(0),
    instantSpeed(0),
    targetSpeed(0) {}

MotorController::MotorController(Motor& motor)
  : motor(motor),
    speedMeasurer(motor),
    curStateIndex(0),
    motorPower(0),
    powerStep(0),
    updateDelay(MOTOR_CONTROLLER_UPDATE_PERIOD),
    maxed(0) {
  cur = &states[curStateIndex];
  prev = &states[curStateIndex^1];
}

Motor& MotorController::getMotor() {
  return motor;
}

void MotorController::setTargetSpeed(int32_t targetSpeed, bool preserveOdometry) {
  long curTime = millis();
  int32_t curTicks = motor.getTicks();
  if (preserveOdometry) {
    checkpoint.targetTicks = getTargetTicksSinceCheckpoint(curTime);
  } else {
    checkpoint.targetTicks = curTicks;
  }
  checkpoint.time = curTime;
  checkpoint.targetSpeed = targetSpeed;

  // Resetting update delay to its default value.
  updateDelay = MOTOR_CONTROLLER_UPDATE_PERIOD;
}

int32_t MotorController::getTargetSpeed() {
  return checkpoint.targetSpeed;
}

int32_t MotorController::getInternalTargetSpeed() {
  return cur->targetSpeed;
}

int32_t MotorController::getSpeed() {
  return speedMeasurer.getSpeed();
}

bool MotorController::isMaxed() {
  return maxed > 1;
}

bool MotorController::update() {
  long curTime = millis();
  speedMeasurer.update(curTime);
  if (curTime - cur->time < updateDelay) {
    // Too early to do an update.
    return false;
  }
  int32_t curTicks = motor.getTicks();
  updateInternal(curTime, curTicks);
  return true;
}

int32_t MotorController::getTargetTicksSinceCheckpoint(long curTime) {
  return checkpoint.targetTicks + checkpoint.targetSpeed * (curTime - checkpoint.time) / 1000;
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
  curStateIndex = (curStateIndex ^ 1);
  prev = cur;
  cur = &states[curStateIndex];

  cur->time = curTime;
  cur->ticks = curTicks;
  cur->instantSpeed = (cur->ticks - prev->ticks) * 1000 / (cur->time - prev->time);

  int32_t ticksDebt = getTargetTicksSinceCheckpoint(cur->time) - cur->ticks;
  cur->targetSpeed = checkpoint.targetSpeed + ticksDebt * 1000 / CATCHUP_TIME;

  if (cur->targetSpeed == 0) {
    if (motorPower != 0) {
      motorPower = 0;
      updateMotorPower();
    }
    return;
  }

  if (cur->time == checkpoint.time) {
    if (cur->targetSpeed > 0) {
      if (motorPower <= 0 || cur->instantSpeed <= 0) {
        motorPower = 0;
        powerStep = MAX_POWER_STEP;
      } else {
        powerStep = computePowerDelta(motorPower, cur->instantSpeed, cur->targetSpeed);
      }
    } else {
      if (motorPower >= 0 || cur->instantSpeed >= 0) {
        motorPower = 0;
        powerStep = -MAX_POWER_STEP;
      } else {
        powerStep = computePowerDelta(motorPower, cur->instantSpeed, cur->targetSpeed);
      }
    }
    motorPower += powerStep;
  } else {
    updateDelay = MOTOR_CONTROLLER_UPDATE_PERIOD;
    if (cur->instantSpeed == 0) {
      if (checkpoint.targetSpeed > 0) {
        powerStep = cur->targetSpeed > checkpoint.targetSpeed ? 1 : 0;
      } else {
        powerStep = cur->targetSpeed < checkpoint.targetSpeed ? -1 : 0;
      }
      motorPower += powerStep;
    } else if (isStableSpeed(prev->instantSpeed, cur->instantSpeed) || abs(powerStep) <= 1) {
      powerStep = computePowerDelta(motorPower, cur->instantSpeed, cur->targetSpeed);
      motorPower += powerStep;
      updateDelay = MOTOR_CONTROLLER_UPDATE_PERIOD << 1;
    } else if (powerStep > 0) {
      // We're currently accelerating forwards.
      if (cur->instantSpeed > cur->targetSpeed) {
        // We overshoot and still accelerating - using binary-search-like approach to adjust the power.
        powerStep = - (powerStep >> 1);
        if (powerStep == 0) {
          powerStep = -1;
        }
        motorPower += powerStep;
      }
    } else {
      // We're currently accelerating backwards.
      if (cur->instantSpeed < cur->targetSpeed) {
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
}

void MotorController::updateMotorPower() {
  if (checkpoint.targetSpeed >= 0) {
    if (motorPower < 0) {
        motorPower = 0;
    }
  } else {
    if (motorPower > 0) {
      motorPower = 0;
    }
  }
  if (motorPower >= Motor::MAX_POWER) {
    motorPower = Motor::MAX_POWER;
    if (cur->instantSpeed < prev->instantSpeed && maxed < 100) {
      ++maxed;
    }
  } else if (motorPower <= -Motor::MAX_POWER) {
    motorPower = -Motor::MAX_POWER;
    if (cur->instantSpeed > prev->instantSpeed && maxed < 100) {
      ++maxed;
    }
  } else {
    maxed = 0;
  }
  if (motorPower != motor.getPower()) {
    motor.setPower(motorPower);
  }
}
