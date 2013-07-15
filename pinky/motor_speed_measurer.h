#ifndef MOTOR_SPEED_MEASURER_H_
#define MOTOR_SPEED_MEASURER_H_

#include <inttypes.h>

#include "motor.h"

#define MOTOR_SPEED_MEASURER_UPDATE_PERIOD 50
// Average the speed over the last 20 updates (1000ms).
#define MOTOR_SPEED_MEASURER_SLIDING_WINDOW_SIZE 20

class LowPassMotorSpeedMeasurer {
public:
  LowPassMotorSpeedMeasurer(Motor& motor);

  bool update(long curTime);

  int32_t getSpeed();

  Motor& motor;
private:

  long lastUpdate;
  int32_t lastTicks;

  int32_t instantSpeed;
  int32_t speed;
};

class SlidingWindowMotorSpeedMeasurer {
public:
  SlidingWindowMotorSpeedMeasurer(Motor& motor);

  bool update(long curTime);

  int32_t getSpeed();

  Motor& motor;
private:

  long lastUpdate;

  int32_t speed;

  // Log of tick measurements used in order to average speed.
  struct TicksLog {
    long time;
    int32_t ticks;
  };
  TicksLog ticksLog[MOTOR_SPEED_MEASURER_SLIDING_WINDOW_SIZE];
  int8_t index;
};


#endif  // MOTOR_SPEED_MEASURER_H_
