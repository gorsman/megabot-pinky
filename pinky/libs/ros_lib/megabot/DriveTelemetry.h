#ifndef _ROS_megabot_DriveTelemetry_h
#define _ROS_megabot_DriveTelemetry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "megabot/MotorTelemetry.h"

namespace megabot
{

  class DriveTelemetry : public ros::Msg
  {
    public:
      megabot::MotorTelemetry Telemetry[2];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      unsigned char * Telemetry_val = (unsigned char *) this->Telemetry;
      for( uint8_t i = 0; i < 2; i++){
      offset += this->Telemetry[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t * Telemetry_val = (uint8_t*) this->Telemetry;
      for( uint8_t i = 0; i < 2; i++){
      offset += this->Telemetry[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "megabot/DriveTelemetry"; };
    const char * getMD5(){ return "e34ba3db901d50117a3cf38ec27cf5ed"; };

  };

}
#endif