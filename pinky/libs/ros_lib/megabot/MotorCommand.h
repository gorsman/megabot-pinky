#ifndef _ROS_megabot_MotorCommand_h
#define _ROS_megabot_MotorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace megabot
{

  class MotorCommand : public ros::Msg
  {
    public:
      int32_t TargetSpeed;
      bool ResetOdometry;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_TargetSpeed;
      u_TargetSpeed.real = this->TargetSpeed;
      *(outbuffer + offset + 0) = (u_TargetSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_TargetSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_TargetSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_TargetSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->TargetSpeed);
      union {
        bool real;
        uint8_t base;
      } u_ResetOdometry;
      u_ResetOdometry.real = this->ResetOdometry;
      *(outbuffer + offset + 0) = (u_ResetOdometry.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ResetOdometry);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_TargetSpeed;
      u_TargetSpeed.base = 0;
      u_TargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_TargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_TargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_TargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->TargetSpeed = u_TargetSpeed.real;
      offset += sizeof(this->TargetSpeed);
      union {
        bool real;
        uint8_t base;
      } u_ResetOdometry;
      u_ResetOdometry.base = 0;
      u_ResetOdometry.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ResetOdometry = u_ResetOdometry.real;
      offset += sizeof(this->ResetOdometry);
     return offset;
    }

    const char * getType(){ return "megabot/MotorCommand"; };
    const char * getMD5(){ return "8300b7edfc7daafac198af6900b197fc"; };

  };

}
#endif