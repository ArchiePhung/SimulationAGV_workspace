#ifndef _ROS_sti_msgs_POWER_info_h
#define _ROS_sti_msgs_POWER_info_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

namespace sti_msgs
{
  class POWER_info : public ros::Msg
  {
    public:
        typedef std_msgs::Float32 _pin_voltage_type ;    _pin_voltage_type     pin_voltage;
        typedef std_msgs::Float32 _pin_analog_type ;     _pin_analog_type      pin_analog;
        typedef std_msgs::Float32 _charge_current_type ; _charge_current_type  charge_current;
        typedef std_msgs::Float32 _charge_analog_type ;  _charge_analog_type   charge_analog;
        typedef std_msgs::Bool    _button1_type ;        _button1_type         button1;
        typedef std_msgs::Bool    _button2_type ;        _button2_type         button2;
        typedef std_msgs::Bool    _EMC_status_type ;     _EMC_status_type      EMC_status;
     

    POWER_info():
      pin_voltage(),
      pin_analog(),
      charge_current(),
      charge_analog(),
      button1(),
      button2(),
      EMC_status()

    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pin_voltage.serialize(outbuffer + offset);
      offset += this->pin_analog.serialize(outbuffer + offset);
      offset += this->charge_current.serialize(outbuffer + offset);
      offset += this->charge_analog.serialize(outbuffer + offset);
      offset += this->button1.serialize(outbuffer + offset);
      offset += this->button2.serialize(outbuffer + offset);
      offset += this->EMC_status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pin_voltage.deserialize(inbuffer + offset);
      offset += this->pin_analog.deserialize(inbuffer + offset);
      offset += this->charge_current.deserialize(inbuffer + offset);
      offset += this->charge_analog.deserialize(inbuffer + offset);

      offset += this->button1.deserialize(inbuffer + offset);
      offset += this->button2.deserialize(inbuffer + offset);
      offset += this->EMC_status.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "sti_msgs/POWER_info"; };
    const char * getMD5(){ return "d8090381c026fea4cf86def03b0257a7"; }; 

  };

}
#endif