#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <inttypes.h>

#include "motor.h"

#define MOTOR_CONTROLLER_UPDATE_PERIOD 50
// Average the speed over the last 20 updates (1000ms).
#define MOTOR_CONTROLLER_NUM_UPDATES_TO_AVERAGE_SPEED 20

class MotorController {
public:
  MotorController(Motor& motor);

  // Set target speed in ticks/sec.
  void setTargetSpeed(int32_t targetSpeed);

  // Returns the actual current speed of the motor.
  int32_t getSpeed();

  // Returns true in case the motor is at maximum speed already
  // and can't reach the target speed.
  bool isMaxed();

  // This method should be called periodically in order to let
  // the controller do its job and update the power on the motor
  // to maintain target speed. (this should be called every 10-50 ms)
  void update();

  Motor& motor;
// private:
  int32_t targetSpeed;
  int16_t motorPower;

  // Actual speed measured based on tick-data from the motor.
  int32_t speed;

  long lastCheckpoint;
  int32_t checkpointTicks;

  int16_t powerStep;
  long lastUpdate;
  int32_t lastTicks;
  int32_t lastInstantSpeed;

  // Log of tick measurements used in order to average speed.
  struct TicksLog {
    long time[MOTOR_CONTROLLER_NUM_UPDATES_TO_AVERAGE_SPEED];
    int32_t count[MOTOR_CONTROLLER_NUM_UPDATES_TO_AVERAGE_SPEED];
    int8_t index;
  };
  TicksLog ticksLog;

  void updateSpeed(long curTime, int32_t curTicks);
  void updateInternal(long curTime, int32_t curTicks);
  void updateMotorPower();
};


#endif  // MOTOR_CONTROLLER_H_
