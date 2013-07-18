#ifndef DRIVER_H_
#define DRIVER_H_

#include <ros.h>
#include <megabot/DriveCommand.h>
#include <megabot/DriveTelemetry.h>

#include "motor.h"
#include "motor_controller.h"

class Driver {
  public:
    Driver(Motor& l, Motor& r);

    void update();
    void executeCommand(const megabot::DriveCommand& command);
    void getTelemetry(megabot::DriveTelemetry* telemetry);

  private:
    void executeMotorCommand(MotorController& motorController, const megabot::MotorCommand& command);
    void getMotorTelemetry(MotorController& motorController, megabot::MotorTelemetry* telemetry);

    MotorController left;
    MotorController right;
    long dieTime;
};

#endif  // DRIVER_H_
