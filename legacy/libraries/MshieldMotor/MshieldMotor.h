#ifndef _MegaBot_motor_h_
#define _MegaBot_motor_h_

#include <inttypes.h>

#define PINKY_MOTOR_LEFT 0
#define PINKY_MOTOR_RIGHT 1

class PinkyMotor {
  public:
    PinkyMotor(uint8_t id);
    void setSpeed(int8_t speed);
  private:
    uint8_t direction_pin_;
    uint8_t speed_pin_;
};

#endif //#ifndef _MegaBot_motor_h_
