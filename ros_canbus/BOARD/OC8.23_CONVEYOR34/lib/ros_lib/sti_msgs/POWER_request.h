#ifndef _ROS_sti_msgs_POWER_request_h
#define _ROS_sti_msgs_POWER_request_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

namespace sti_msgs
{
  class POWER_request : public ros::Msg
  {
    public:
        typedef std_msgs::Bool  _charge_type ;      _charge_type      charge;
        typedef std_msgs::Bool  _sound_on_type ;    _sound_on_type    sound_on;
        typedef std_msgs::Int16 _sound_type_type ;  _sound_type_type  sound_type;
        typedef std_msgs::Bool  _EMC_write_type ;   _EMC_write_type   EMC_write;
        typedef std_msgs::Bool  _EMC_reset_type ;   _EMC_reset_type   EMC_reset;
        typedef std_msgs::Bool  _OFF_5v_type ;      _OFF_5v_type      OFF_5v;
        typedef std_msgs::Bool  _OFF_22v_type ;     _OFF_22v_type     OFF_22v;
        typedef std_msgs::Bool  _led_button1_type ; _led_button1_type led_button1;
        typedef std_msgs::Bool  _led_button2_type ; _led_button2_type led_button2;
        typedef std_msgs::Float32 _a_coefficient_type ;    _a_coefficient_type     a_coefficient;
        typedef std_msgs::Float32 _b_coefficient_type ;    _b_coefficient_type     b_coefficient;

    POWER_request():
      charge(),
      sound_on(),
      sound_type(),
      EMC_write(),
      EMC_reset(),
      OFF_5v(),
      OFF_22v(),
      led_button1(),
      led_button2(),
      a_coefficient(),
      b_coefficient()

    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->charge.serialize(outbuffer + offset);
      offset += this->sound_on.serialize(outbuffer + offset);
      offset += this->sound_type.serialize(outbuffer + offset);
      offset += this->EMC_write.serialize(outbuffer + offset);
      offset += this->EMC_reset.serialize(outbuffer + offset);
      offset += this->OFF_5v.serialize(outbuffer + offset);
      offset += this->OFF_22v.serialize(outbuffer + offset);
      offset += this->led_button1.serialize(outbuffer + offset);
      offset += this->led_button2.serialize(outbuffer + offset);
      offset += this->a_coefficient.serialize(outbuffer + offset);
      offset += this->b_coefficient.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->charge.deserialize(inbuffer + offset);
      offset += this->sound_on.deserialize(inbuffer + offset);
      offset += this->sound_type.deserialize(inbuffer + offset);
      offset += this->EMC_write.deserialize(inbuffer + offset);
      offset += this->EMC_reset.deserialize(inbuffer + offset);
      offset += this->OFF_5v.deserialize(inbuffer + offset);
      offset += this->OFF_22v.deserialize(inbuffer + offset);
      offset += this->led_button1.deserialize(inbuffer + offset);
      offset += this->led_button2.deserialize(inbuffer + offset);
      offset += this->a_coefficient.deserialize(inbuffer + offset);
      offset += this->b_coefficient.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "sti_msgs/POWER_request"; };
    const char * getMD5(){ return "416e64738f56bb40b8d1752c301bac5b"; }; 

  };

}
#endif