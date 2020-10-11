// Generated by gencpp from file franka_msgs/error_recoveryRequest.msg
// DO NOT EDIT!


#ifndef FRANKA_MSGS_MESSAGE_ERROR_RECOVERYREQUEST_H
#define FRANKA_MSGS_MESSAGE_ERROR_RECOVERYREQUEST_H


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
struct error_recoveryRequest_
{
  typedef error_recoveryRequest_<ContainerAllocator> Type;

  error_recoveryRequest_()
    : req(false)  {
    }
  error_recoveryRequest_(const ContainerAllocator& _alloc)
    : req(false)  {
  (void)_alloc;
    }



   typedef uint8_t _req_type;
  _req_type req;





  typedef boost::shared_ptr< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> const> ConstPtr;

}; // struct error_recoveryRequest_

typedef ::franka_msgs::error_recoveryRequest_<std::allocator<void> > error_recoveryRequest;

typedef boost::shared_ptr< ::franka_msgs::error_recoveryRequest > error_recoveryRequestPtr;
typedef boost::shared_ptr< ::franka_msgs::error_recoveryRequest const> error_recoveryRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_msgs::error_recoveryRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace franka_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'franka_msgs': ['/home/master/franka_final_ws/src/franka_modern_driver/franka_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "be3c44e19d0c6b00b25e356c69155e2a";
  }

  static const char* value(const ::franka_msgs::error_recoveryRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbe3c44e19d0c6b00ULL;
  static const uint64_t static_value2 = 0xb25e356c69155e2aULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_msgs/error_recoveryRequest";
  }

  static const char* value(const ::franka_msgs::error_recoveryRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool req\n"
;
  }

  static const char* value(const ::franka_msgs::error_recoveryRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.req);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct error_recoveryRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_msgs::error_recoveryRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_msgs::error_recoveryRequest_<ContainerAllocator>& v)
  {
    s << indent << "req: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.req);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_MSGS_MESSAGE_ERROR_RECOVERYREQUEST_H