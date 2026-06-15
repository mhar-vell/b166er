#ifndef _ROS_movemaster_msg_status_h
#define _ROS_movemaster_msg_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace movemaster_msg
{

  class status : public ros::Msg
  {
    public:
      typedef const char* _joint_type;
      _joint_type joint;
      typedef float _setpoint_type;
      _setpoint_type setpoint;
      typedef float _pulse_count_type;
      _pulse_count_type pulse_count;
      typedef float _error_type;
      _error_type error;
      typedef float _output_type;
      _output_type output;
      typedef float _control_loop_type;
      _control_loop_type control_loop;
      typedef bool _IsDone_type;
      _IsDone_type IsDone;

    status():
      joint(""),
      setpoint(0),
      pulse_count(0),
      error(0),
      output(0),
      control_loop(0),
      IsDone(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_joint = strlen(this->joint);
      varToArr(outbuffer + offset, length_joint);
      offset += 4;
      memcpy(outbuffer + offset, this->joint, length_joint);
      offset += length_joint;
      union {
        float real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.real = this->setpoint;
      *(outbuffer + offset + 0) = (u_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->setpoint);
      union {
        float real;
        uint32_t base;
      } u_pulse_count;
      u_pulse_count.real = this->pulse_count;
      *(outbuffer + offset + 0) = (u_pulse_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pulse_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pulse_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pulse_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pulse_count);
      union {
        float real;
        uint32_t base;
      } u_error;
      u_error.real = this->error;
      *(outbuffer + offset + 0) = (u_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error);
      union {
        float real;
        uint32_t base;
      } u_output;
      u_output.real = this->output;
      *(outbuffer + offset + 0) = (u_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output);
      union {
        float real;
        uint32_t base;
      } u_control_loop;
      u_control_loop.real = this->control_loop;
      *(outbuffer + offset + 0) = (u_control_loop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_control_loop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_control_loop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_control_loop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->control_loop);
      union {
        bool real;
        uint8_t base;
      } u_IsDone;
      u_IsDone.real = this->IsDone;
      *(outbuffer + offset + 0) = (u_IsDone.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->IsDone);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_joint;
      arrToVar(length_joint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint-1]=0;
      this->joint = (char *)(inbuffer + offset-1);
      offset += length_joint;
      union {
        float real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.base = 0;
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->setpoint = u_setpoint.real;
      offset += sizeof(this->setpoint);
      union {
        float real;
        uint32_t base;
      } u_pulse_count;
      u_pulse_count.base = 0;
      u_pulse_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pulse_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pulse_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pulse_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pulse_count = u_pulse_count.real;
      offset += sizeof(this->pulse_count);
      union {
        float real;
        uint32_t base;
      } u_error;
      u_error.base = 0;
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error = u_error.real;
      offset += sizeof(this->error);
      union {
        float real;
        uint32_t base;
      } u_output;
      u_output.base = 0;
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output = u_output.real;
      offset += sizeof(this->output);
      union {
        float real;
        uint32_t base;
      } u_control_loop;
      u_control_loop.base = 0;
      u_control_loop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_control_loop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_control_loop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_control_loop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->control_loop = u_control_loop.real;
      offset += sizeof(this->control_loop);
      union {
        bool real;
        uint8_t base;
      } u_IsDone;
      u_IsDone.base = 0;
      u_IsDone.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->IsDone = u_IsDone.real;
      offset += sizeof(this->IsDone);
     return offset;
    }

    virtual const char * getType() override { return "movemaster_msg/status"; };
    virtual const char * getMD5() override { return "283025d1e58370fa7a43c23c572de06b"; };

  };

}
#endif
