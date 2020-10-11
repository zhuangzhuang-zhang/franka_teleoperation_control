// Generated by gencpp from file franka_msgs/servoj.msg
// DO NOT EDIT!


#ifndef FRANKA_MSGS_MESSAGE_SERVOJ_H
#define FRANKA_MSGS_MESSAGE_SERVOJ_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace franka_msgs
{
template <class ContainerAllocator>
struct servoj_
{
  typedef servoj_<ContainerAllocator> Type;

  servoj_()
    : keepalive(0)
    , cmd_q()  {
    }
  servoj_(const ContainerAllocator& _alloc)
    : keepalive(0)
    , cmd_q(_alloc)  {
  (void)_alloc;
    }



   typedef int64_t _keepalive_type;
  _keepalive_type keepalive;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cmd_q_type;
  _cmd_q_type cmd_q;





  typedef boost::shared_ptr< ::franka_msgs::servoj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_msgs::servoj_<ContainerAllocator> const> ConstPtr;

}; // struct servoj_

typedef ::franka_msgs::servoj_<std::allocator<void> > servoj;

typedef boost::shared_ptr< ::franka_msgs::servoj > servojPtr;
typedef boost::shared_ptr< ::franka_msgs::servoj const> servojConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_msgs::servoj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_msgs::servoj_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace franka_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'franka_msgs': ['/home/master/franka_final_ws/src/franka_modern_driver/franka_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::franka_msgs::servoj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_msgs::servoj_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_msgs::servoj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_msgs::servoj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_msgs::servoj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_msgs::servoj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_msgs::servoj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "961076d13ed378a92e75de5ec5b3a69b";
  }

  static const char* value(const ::franka_msgs::servoj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x961076d13ed378a9ULL;
  static const uint64_t static_value2 = 0x2e75de5ec5b3a69bULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_msgs::servoj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_msgs/servoj";
  }

  static const char* value(const ::franka_msgs::servoj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_msgs::servoj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 keepalive\n"
"float64[] cmd_q\n"
;
  }

  static const char* value(const ::franka_msgs::servoj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_msgs::servoj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.keepalive);
      stream.next(m.cmd_q);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct servoj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_msgs::servoj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_msgs::servoj_<ContainerAllocator>& v)
  {
    s << indent << "keepalive: ";
    Printer<int64_t>::stream(s, indent + "  ", v.keepalive);
    s << indent << "cmd_q[]" << std::endl;
    for (size_t i = 0; i < v.cmd_q.size(); ++i)
    {
      s << indent << "  cmd_q[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cmd_q[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_MSGS_MESSAGE_SERVOJ_H
