#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_MAX_POWER 127

#define MOTOR_HARDWARE_ENABLED
#include "motor_hardware.h"
typedef MotorHardware Motor;

/*
#define MOTOR_EMULATOR_ENABLED
#include "motor_emulator.h"
typedef MotorEmulator Motor;
*/


#endif  // MOTOR_H_
