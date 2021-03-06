#ifndef MOTOR_EMULATOR_H_
#define MOTOR_EMULATOR_H_

#include <inttypes.h>

class MotorEmulator {
public:
	MotorEmulator();

	int32_t getTicks();

	void setPower(int16_t power);
	int16_t getPower();

	static MotorEmulator LEFT;
	static MotorEmulator RIGHT;

	static const int16_t MAX_POWER;

private:
	// Power set from outside.
  int16_t power;
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
