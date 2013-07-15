#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_MAX_POWER 127
#define MOTOR_TICKS_PER_ROUND int32_t(3000)

// #define MOTOR_USE_EMULATOR

#ifdef MOTOR_USE_EMULATOR

#define MOTOR_EMULATOR_ENABLED
#include "motor_emulator.h"
typedef MotorEmulator Motor;

#else

#define MOTOR_HARDWARE_ENABLED
#include "motor_hardware.h"
typedef MotorHardware Motor;

#endif

#endif  // MOTOR_H_
