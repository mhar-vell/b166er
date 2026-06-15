#ifndef _ROS_rosaria_BumperState_h
#define _ROS_rosaria_BumperState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rosaria
{

  class BumperState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t front_bumpers_length;
      typedef bool _front_bumpers_type;
      _front_bumpers_type st_front_bumpers;
      _front_bumpers_type * front_bumpers;
      uint32_t rear_bumpers_length;
      typedef bool _rear_bumpers_type;
      _rear_bumpers_type st_rear_bumpers;
      _rear_bumpers_type * rear_bumpers;

    BumperState():
      header(),
      front_bumpers_length(0), st_front_bumpers(), front_bumpers(nullptr),
      rear_bumpers_length(0), st_rear_bumpers(), rear_bumpers(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->front_bumpers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->front_bumpers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->front_bumpers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->front_bumpers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_bumpers_length);
      for( uint32_t i = 0; i < front_bumpers_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_front_bumpersi;
      u_front_bumpersi.real = this->front_bumpers[i];
      *(outbuffer + offset + 0) = (u_front_bumpersi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->front_bumpers[i]);
      }
      *(outbuffer + offset + 0) = (this->rear_bumpers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rear_bumpers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rear_bumpers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rear_bumpers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_bumpers_length);
      for( uint32_t i = 0; i < rear_bumpers_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_rear_bumpersi;
      u_rear_bumpersi.real = this->rear_bumpers[i];
      *(outbuffer + offset + 0) = (u_rear_bumpersi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rear_bumpers[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t front_bumpers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      front_bumpers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      front_bumpers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      front_bumpers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->front_bumpers_length);
      if(front_bumpers_lengthT > front_bumpers_length)
        this->front_bumpers = (bool*)realloc(this->front_bumpers, front_bumpers_lengthT * sizeof(bool));
      front_bumpers_length = front_bumpers_lengthT;
      for( uint32_t i = 0; i < front_bumpers_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_front_bumpers;
      u_st_front_bumpers.base = 0;
      u_st_front_bumpers.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_front_bumpers = u_st_front_bumpers.real;
      offset += sizeof(this->st_front_bumpers);
        memcpy( &(this->front_bumpers[i]), &(this->st_front_bumpers), sizeof(bool));
      }
      uint32_t rear_bumpers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rear_bumpers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rear_bumpers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rear_bumpers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rear_bumpers_length);
      if(rear_bumpers_lengthT > rear_bumpers_length)
        this->rear_bumpers = (bool*)realloc(this->rear_bumpers, rear_bumpers_lengthT * sizeof(bool));
      rear_bumpers_length = rear_bumpers_lengthT;
      for( uint32_t i = 0; i < rear_bumpers_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_rear_bumpers;
      u_st_rear_bumpers.base = 0;
      u_st_rear_bumpers.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_rear_bumpers = u_st_rear_bumpers.real;
      offset += sizeof(this->st_rear_bumpers);
        memcpy( &(this->rear_bumpers[i]), &(this->st_rear_bumpers), sizeof(bool));
      }
     return offset;
    }

    virtual const char * getType() override { return "rosaria/BumperState"; };
    virtual const char * getMD5() override { return "f81947761ff7e166a3bbaf937b9869b5"; };

  };

}
#endif
