#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <inttypes.h>

#include "motor.h"
#include "motor_speed_measurer.h"

#define MOTOR_CONTROLLER_UPDATE_PERIOD 50

class MotorController {
public:
  MotorController(Motor& motor);

  Motor& getMotor();

  // Set target speed in ticks/sec.
  void setTargetSpeed(int32_t targetSpeed, bool preserveOdometry);
  // Returns the current target speed.
  int32_t getTargetSpeed();
  int32_t getInternalTargetSpeed();

  // Returns the actual current speed of the motor.
  int32_t getSpeed();

  // Returns true in case the motor is at maximum speed already
  // and can't reach the target speed.
  bool isMaxed();

  // This method should be called periodically in order to let
  // the controller do its job and update the power on the motor
  // to maintain target speed. (this should be called every 10-50 ms)
  //
  // This update takes around 300 mocroseconds of CPU time (as benchmarked on Arduino Uno).
  bool update();

  Motor& motor;
private:
  LowPassMotorSpeedMeasurer speedMeasurer;

  int32_t targetSpeed;
  int32_t internalTargetSpeed;

  int16_t motorPower;
  long updateDelay;

  long lastCheckpoint;
  int32_t checkpointTicks;

  int16_t powerStep;
  long lastUpdate;
  int32_t lastTicks;
  int32_t lastInstantSpeed;

  bool maxed;

  void updateInternal(long curTime, int32_t curTicks);
  void updateMotorPower(int32_t instantSpeedDelta);
};


#endif  // MOTOR_CONTROLLER_H_
