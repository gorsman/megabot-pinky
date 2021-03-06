#ifndef _ROS_megabot_MotorTelemetry_h
#define _ROS_megabot_MotorTelemetry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace megabot
{

  class MotorTelemetry : public ros::Msg
  {
    public:
      int32_t Distance;
      int32_t CurrentSpeed;
      int32_t InstantSpeed;
      int32_t TargetSpeed;
      int32_t InternalTargetSpeed;
      bool Maxed;
      int16_t MotorPower;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Distance;
      u_Distance.real = this->Distance;
      *(outbuffer + offset + 0) = (u_Distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Distance);
      union {
        int32_t real;
        uint32_t base;
      } u_CurrentSpeed;
      u_CurrentSpeed.real = this->CurrentSpeed;
      *(outbuffer + offset + 0) = (u_CurrentSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_CurrentSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_CurrentSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_CurrentSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->CurrentSpeed);
      union {
        int32_t real;
        uint32_t base;
      } u_InstantSpeed;
      u_InstantSpeed.real = this->InstantSpeed;
      *(outbuffer + offset + 0) = (u_InstantSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_InstantSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_InstantSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_InstantSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->InstantSpeed);
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
        int32_t real;
        uint32_t base;
      } u_InternalTargetSpeed;
      u_InternalTargetSpeed.real = this->InternalTargetSpeed;
      *(outbuffer + offset + 0) = (u_InternalTargetSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_InternalTargetSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_InternalTargetSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_InternalTargetSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->InternalTargetSpeed);
      union {
        bool real;
        uint8_t base;
      } u_Maxed;
      u_Maxed.real = this->Maxed;
      *(outbuffer + offset + 0) = (u_Maxed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Maxed);
      union {
        int16_t real;
        uint16_t base;
      } u_MotorPower;
      u_MotorPower.real = this->MotorPower;
      *(outbuffer + offset + 0) = (u_MotorPower.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MotorPower.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->MotorPower);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Distance;
      u_Distance.base = 0;
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Distance = u_Distance.real;
      offset += sizeof(this->Distance);
      union {
        int32_t real;
        uint32_t base;
      } u_CurrentSpeed;
      u_CurrentSpeed.base = 0;
      u_CurrentSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_CurrentSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_CurrentSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_CurrentSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->CurrentSpeed = u_CurrentSpeed.real;
      offset += sizeof(this->CurrentSpeed);
      union {
        int32_t real;
        uint32_t base;
      } u_InstantSpeed;
      u_InstantSpeed.base = 0;
      u_InstantSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_InstantSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_InstantSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_InstantSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->InstantSpeed = u_InstantSpeed.real;
      offset += sizeof(this->InstantSpeed);
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
        int32_t real;
        uint32_t base;
      } u_InternalTargetSpeed;
      u_InternalTargetSpeed.base = 0;
      u_InternalTargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_InternalTargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_InternalTargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_InternalTargetSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->InternalTargetSpeed = u_InternalTargetSpeed.real;
      offset += sizeof(this->InternalTargetSpeed);
      union {
        bool real;
        uint8_t base;
      } u_Maxed;
      u_Maxed.base = 0;
      u_Maxed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Maxed = u_Maxed.real;
      offset += sizeof(this->Maxed);
      union {
        int16_t real;
        uint16_t base;
      } u_MotorPower;
      u_MotorPower.base = 0;
      u_MotorPower.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MotorPower.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->MotorPower = u_MotorPower.real;
      offset += sizeof(this->MotorPower);
     return offset;
    }

    const char * getType(){ return "megabot/MotorTelemetry"; };
    const char * getMD5(){ return "26e8128e91c9644c00327c1fcfc638c3"; };

  };

}
#endif