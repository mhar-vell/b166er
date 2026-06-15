#ifndef _ROS_movemaster_msg_setpoint_h
#define _ROS_movemaster_msg_setpoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace movemaster_msg
{

  class setpoint : public ros::Msg
  {
    public:
      typedef float _set_1_type;
      _set_1_type set_1;
      typedef float _set_2_type;
      _set_2_type set_2;
      typedef float _set_3_type;
      _set_3_type set_3;
      typedef float _set_4_type;
      _set_4_type set_4;
      typedef float _set_5_type;
      _set_5_type set_5;
      typedef bool _set_GRIP_type;
      _set_GRIP_type set_GRIP;
      typedef bool _emergency_stop_type;
      _emergency_stop_type emergency_stop;
      typedef int8_t _GoHome_type;
      _GoHome_type GoHome;

    setpoint():
      set_1(0),
      set_2(0),
      set_3(0),
      set_4(0),
      set_5(0),
      set_GRIP(0),
      emergency_stop(0),
      GoHome(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_set_1;
      u_set_1.real = this->set_1;
      *(outbuffer + offset + 0) = (u_set_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_1);
      union {
        float real;
        uint32_t base;
      } u_set_2;
      u_set_2.real = this->set_2;
      *(outbuffer + offset + 0) = (u_set_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_2);
      union {
        float real;
        uint32_t base;
      } u_set_3;
      u_set_3.real = this->set_3;
      *(outbuffer + offset + 0) = (u_set_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_3);
      union {
        float real;
        uint32_t base;
      } u_set_4;
      u_set_4.real = this->set_4;
      *(outbuffer + offset + 0) = (u_set_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_4);
      union {
        float real;
        uint32_t base;
      } u_set_5;
      u_set_5.real = this->set_5;
      *(outbuffer + offset + 0) = (u_set_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_5);
      union {
        bool real;
        uint8_t base;
      } u_set_GRIP;
      u_set_GRIP.real = this->set_GRIP;
      *(outbuffer + offset + 0) = (u_set_GRIP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_GRIP);
      union {
        bool real;
        uint8_t base;
      } u_emergency_stop;
      u_emergency_stop.real = this->emergency_stop;
      *(outbuffer + offset + 0) = (u_emergency_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_stop);
      union {
        int8_t real;
        uint8_t base;
      } u_GoHome;
      u_GoHome.real = this->GoHome;
      *(outbuffer + offset + 0) = (u_GoHome.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->GoHome);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_set_1;
      u_set_1.base = 0;
      u_set_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_1 = u_set_1.real;
      offset += sizeof(this->set_1);
      union {
        float real;
        uint32_t base;
      } u_set_2;
      u_set_2.base = 0;
      u_set_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_2 = u_set_2.real;
      offset += sizeof(this->set_2);
      union {
        float real;
        uint32_t base;
      } u_set_3;
      u_set_3.base = 0;
      u_set_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_3 = u_set_3.real;
      offset += sizeof(this->set_3);
      union {
        float real;
        uint32_t base;
      } u_set_4;
      u_set_4.base = 0;
      u_set_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_4 = u_set_4.real;
      offset += sizeof(this->set_4);
      union {
        float real;
        uint32_t base;
      } u_set_5;
      u_set_5.base = 0;
      u_set_5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_5 = u_set_5.real;
      offset += sizeof(this->set_5);
      union {
        bool real;
        uint8_t base;
      } u_set_GRIP;
      u_set_GRIP.base = 0;
      u_set_GRIP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_GRIP = u_set_GRIP.real;
      offset += sizeof(this->set_GRIP);
      union {
        bool real;
        uint8_t base;
      } u_emergency_stop;
      u_emergency_stop.base = 0;
      u_emergency_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_stop = u_emergency_stop.real;
      offset += sizeof(this->emergency_stop);
      union {
        int8_t real;
        uint8_t base;
      } u_GoHome;
      u_GoHome.base = 0;
      u_GoHome.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->GoHome = u_GoHome.real;
      offset += sizeof(this->GoHome);
     return offset;
    }

    virtual const char * getType() override { return "movemaster_msg/setpoint"; };
    virtual const char * getMD5() override { return "aaa39b4215432c08ac9b36247ded9610"; };

  };

}
#endif
