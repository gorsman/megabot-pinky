#include "driver.h"

#include "Arduino.h"


Driver::Driver(Motor& l, Motor& r)
    : left(l), right(r), dieTime(0) { }

void Driver::update() {
  long curTime = millis();
  if (curTime > dieTime) {
    left.setTargetSpeed(0, false);
    right.setTargetSpeed(0, false);
  }
  left.update();
  right.update();
}

void Driver::executeCommand(const megabot::DriveCommand& command) {
  dieTime = millis() + command.TimeToLive;
  executeMotorCommand(left, command.Cmd[0]);
  executeMotorCommand(right, command.Cmd[1]);
}

void Driver::getTelemetry(megabot::DriveTelemetry* telemetry) {
  getMotorTelemetry(left, &telemetry->Telemetry[0]);
  getMotorTelemetry(right, &telemetry->Telemetry[1]);
}

void Driver::executeMotorCommand(MotorController& motorController, const megabot::MotorCommand& command) {
  motorController.setTargetSpeed(command.TargetSpeed, command.PreserveOdometry);
}

void Driver::getMotorTelemetry(MotorController& motorController, megabot::MotorTelemetry* telemetry) {
  Motor& motor = motorController.getMotor();
  telemetry->Distance = motor.getTicks();
  telemetry->CurrentSpeed = motorController.getSpeed();
  telemetry->InstantSpeed = motorController.getInstantSpeed();
  telemetry->TargetSpeed = motorController.getTargetSpeed();
  telemetry->InternalTargetSpeed = motorController.getInternalTargetSpeed();
  telemetry->Maxed = motorController.isMaxed();
  telemetry->MotorPower = motor.getPower();
}
