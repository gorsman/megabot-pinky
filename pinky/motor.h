#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_USE_EMULATOR

#ifdef MOTOR_USE_EMULATOR

#include "motor_emulator.h"
typedef MotorEmulator Motor;

#else

// #include "motor_pinky.h"
// typedef MotorPinky Motor;

#endif

#endif /* MOTOR_H_ */
