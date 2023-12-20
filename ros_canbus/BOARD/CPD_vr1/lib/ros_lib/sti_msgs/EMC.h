// Generated by gencpp from file sti_msgs/EMC.msg
// DO NOT EDIT!


#ifndef STI_MSGS_MESSAGE_EMC_H
#define STI_MSGS_MESSAGE_EMC_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sti_msgs
{
template <class ContainerAllocator>
struct EMC_
{
  typedef EMC_<ContainerAllocator> Type;

  EMC_()
    : status_base(0.0)  {
    }
  EMC_(const ContainerAllocator& _alloc)
    : status_base(0.0)  {
  (void)_alloc;
    }



   typedef float _status_base_type;
  _status_base_type status_base;



  enum {
    error_wheel1 = 0u,
    error_wheel2 = 1u,
    EMC = 2u,
  };


  typedef boost::shared_ptr< ::sti_msgs::EMC_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sti_msgs::EMC_<ContainerAllocator> const> ConstPtr;

}; // struct EMC_

typedef ::sti_msgs::EMC_<std::allocator<void> > EMC;

typedef boost::shared_ptr< ::sti_msgs::EMC > EMCPtr;
typedef boost::shared_ptr< ::sti_msgs::EMC const> EMCConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sti_msgs::EMC_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sti_msgs::EMC_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sti_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'sti_msgs': ['/home/stivietnam/catkin_ws/src/sti_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sti_msgs::EMC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sti_msgs::EMC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sti_msgs::EMC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sti_msgs::EMC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sti_msgs::EMC_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sti_msgs::EMC_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sti_msgs::EMC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "02f00ac95cb5a047f352f0e77ecba4af";
  }

  static const char* value(const ::sti_msgs::EMC_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x02f00ac95cb5a047ULL;
  static const uint64_t static_value2 = 0xf352f0e77ecba4afULL;
};

template<class ContainerAllocator>
struct DataType< ::sti_msgs::EMC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sti_msgs/EMC";
  }

  static const char* value(const ::sti_msgs::EMC_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sti_msgs::EMC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 error_wheel1=0\n\
uint8 error_wheel2=1\n\
uint8 EMC=2\n\
\n\
float32 status_base # trang thai base\n\
";
  }

  static const char* value(const ::sti_msgs::EMC_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sti_msgs::EMC_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status_base);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EMC_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sti_msgs::EMC_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sti_msgs::EMC_<ContainerAllocator>& v)
  {
    s << indent << "status_base: ";
    Printer<float>::stream(s, indent + "  ", v.status_base);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STI_MSGS_MESSAGE_EMC_H