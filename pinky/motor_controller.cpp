#include "motor_controller.h"

#include "Arduino.h"

// #define POWER_INCREMENT 10
// #define TICKS_DELTA_THRESHOLD 2
#define TICKS_DELTA_ACCELERATION_THRESHOLD 5

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
    lastInstantSpeed(0),
    lastTicksDelta(0) {
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
    lastInstantSpeed = instantSpeed;
    lastTicksDelta = ticksDelta;
    return;
  }

  if (lastUpdate <= lastCheckpoint) {
    if (targetSpeed > 0) {
      if (motorPower <= 0) {
        powerStep = MOTOR_MAX_POWER >> 1;
        motorPower = powerStep;
      } else {
        // TODO: implemenet here
      }
    } else {
      if (motorPower >= 0) {
        powerStep = -(MOTOR_MAX_POWER >> 1);
        motorPower = powerStep;
      } else {
        // TODO: implement here
      }
    }
  } else {
    int32_t targetTicks = checkpointTicks + targetSpeed * (curTime - lastCheckpoint) / 1000;
    int32_t controlTicksDelta = targetTicks - curTicks;

    if (powerStep > 0) {
      // We're currently accelerating forwards.

      //if (controlTicksDelta < 0) {
      if (instantSpeed > targetSpeed) {
        // We overshoot.

        powerStep = - ((powerStep * 3) >> 2);
        if (powerStep == 0) {
          powerStep = -1;
        }
        motorPower += powerStep;
      // } else if (controlTicksDelta > 0) {
      } else {
        bool accelerating = abs(ticksDelta - lastTicksDelta) > TICKS_DELTA_ACCELERATION_THRESHOLD;
        if (!accelerating) {
          motorPower += powerStep;
        }
      }
    } else {
      // We're currently accelerating backwards.

      // if (controlTicksDelta > 0) {
      if (instantSpeed < targetSpeed) {
        // We overshoot.

        powerStep = (-powerStep * 3) >> 2;
        if (powerStep == 0) {
          powerStep = 1;
        }
        motorPower += powerStep;
      // } else if (controlTicksDelta < 0) {
      } else {
        bool accelerating = abs(ticksDelta - lastTicksDelta) > TICKS_DELTA_ACCELERATION_THRESHOLD;
        if (!accelerating) {
          motorPower += powerStep;
        }
      }

    }

  }
  updateMotorPower();

  lastUpdate = curTime;
  lastTicks = curTicks;
  lastInstantSpeed = instantSpeed;
  lastTicksDelta = ticksDelta;
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
