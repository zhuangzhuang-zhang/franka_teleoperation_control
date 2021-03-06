// Generated by gencpp from file robot_msgs/omega.msg
// DO NOT EDIT!


#ifndef ROBOT_MSGS_MESSAGE_OMEGA_H
#define ROBOT_MSGS_MESSAGE_OMEGA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_msgs
{
template <class ContainerAllocator>
struct omega_
{
  typedef omega_<ContainerAllocator> Type;

  omega_()
    : data()
    , button()  {
    }
  omega_(const ContainerAllocator& _alloc)
    : data(_alloc)
    , button(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _data_type;
  _data_type data;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _button_type;
  _button_type button;





  typedef boost::shared_ptr< ::robot_msgs::omega_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msgs::omega_<ContainerAllocator> const> ConstPtr;

}; // struct omega_

typedef ::robot_msgs::omega_<std::allocator<void> > omega;

typedef boost::shared_ptr< ::robot_msgs::omega > omegaPtr;
typedef boost::shared_ptr< ::robot_msgs::omega const> omegaConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msgs::omega_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msgs::omega_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'robot_msgs': ['/home/master/franka_final_ws/src/teleoperation/robot_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::omega_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::omega_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::omega_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::omega_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::omega_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::omega_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msgs::omega_<ContainerAllocator> >
{
  static const char* value()
  {
    return "90483ddf79d66b127324c21be866ac8d";
  }

  static const char* value(const ::robot_msgs::omega_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x90483ddf79d66b12ULL;
  static const uint64_t static_value2 = 0x7324c21be866ac8dULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msgs::omega_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msgs/omega";
  }

  static const char* value(const ::robot_msgs::omega_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msgs::omega_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] data\n"
"float64[] button\n"
;
  }

  static const char* value(const ::robot_msgs::omega_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msgs::omega_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
      stream.next(m.button);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct omega_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msgs::omega_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msgs::omega_<ContainerAllocator>& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "button[]" << std::endl;
    for (size_t i = 0; i < v.button.size(); ++i)
    {
      s << indent << "  button[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.button[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSGS_MESSAGE_OMEGA_H
