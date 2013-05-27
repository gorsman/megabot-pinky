#ifndef MOTOR_EMULATOR_H_
#define MOTOR_EMULATOR_H_

#include <inttypes.h>

#define MOTOR_EMULATOR_ENABLED

class MotorEmulator {
public:
	MotorEmulator();

	int32_t getTicks();

	void setPower(int8_t power);
	int8_t getPower();

	static MotorEmulator LEFT;
	static MotorEmulator RIGHT;

private:
	// Power set from outside.
  int8_t power;
  // Speed derived from the #power.
  int32_t targetSpeed;  // ticks/sec

	long lastUpdate;
	int32_t lastTicks;
  int32_t lastSpeed;  // ticks/sec

  long timeToAccelerate;
  int32_t ticksToAccelerate;

	long cacheTime;
	int32_t cachedTicks;

  int32_t speedFromPower(int8_t power);

  int32_t getCurrentSpeed(long curTime);
	int32_t getTicksInternal(long curTime);
};

#endif  // MOTOR_H_
