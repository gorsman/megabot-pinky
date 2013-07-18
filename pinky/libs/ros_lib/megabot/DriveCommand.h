#ifndef _ROS_megabot_DriveCommand_h
#define _ROS_megabot_DriveCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "megabot/MotorCommand.h"

namespace megabot
{

  class DriveCommand : public ros::Msg
  {
    public:
      megabot::MotorCommand Cmd[2];
      int32_t TimeToLive;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      unsigned char * Cmd_val = (unsigned char *) this->Cmd;
      for( uint8_t i = 0; i < 2; i++){
      offset += this->Cmd[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_TimeToLive;
      u_TimeToLive.real = this->TimeToLive;
      *(outbuffer + offset + 0) = (u_TimeToLive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_TimeToLive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_TimeToLive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_TimeToLive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->TimeToLive);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t * Cmd_val = (uint8_t*) this->Cmd;
      for( uint8_t i = 0; i < 2; i++){
      offset += this->Cmd[i].deserialize(inbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_TimeToLive;
      u_TimeToLive.base = 0;
      u_TimeToLive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_TimeToLive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_TimeToLive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_TimeToLive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->TimeToLive = u_TimeToLive.real;
      offset += sizeof(this->TimeToLive);
     return offset;
    }

    const char * getType(){ return "megabot/DriveCommand"; };
    const char * getMD5(){ return "803f660ab79852767fdeadd7657af716"; };

  };

}
#endif