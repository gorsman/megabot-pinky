#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_MAX_POWER 127
#define MOTOR_TICKS_PER_ROUND int32_t(3000)

#define MOTOR_USE_EMULATOR

#ifdef MOTOR_USE_EMULATOR

#include "motor_emulator.h"
typedef MotorEmulator Motor;

#else

// #include "motor_pinky.h"
// typedef MotorPinky Motor;

#endif

#endif  // MOTOR_H_
