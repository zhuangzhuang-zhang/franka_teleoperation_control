// Generated by gencpp from file franka_msgs/Errors.msg
// DO NOT EDIT!


#ifndef FRANKA_MSGS_MESSAGE_ERRORS_H
#define FRANKA_MSGS_MESSAGE_ERRORS_H


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
struct Errors_
{
  typedef Errors_<ContainerAllocator> Type;

  Errors_()
    : joint_position_limits_violation(false)
    , cartesian_position_limits_violation(false)
    , self_collision_avoidance_violation(false)
    , joint_velocity_violation(false)
    , cartesian_velocity_violation(false)
    , force_control_safety_violation(false)
    , joint_reflex(false)
    , cartesian_reflex(false)
    , max_goal_pose_deviation_violation(false)
    , max_path_pose_deviation_violation(false)
    , cartesian_velocity_profile_safety_violation(false)
    , joint_position_motion_generator_start_pose_invalid(false)
    , joint_motion_generator_position_limits_violation(false)
    , joint_motion_generator_velocity_limits_violation(false)
    , joint_motion_generator_velocity_discontinuity(false)
    , joint_motion_generator_acceleration_discontinuity(false)
    , cartesian_position_motion_generator_start_pose_invalid(false)
    , cartesian_motion_generator_elbow_limit_violation(false)
    , cartesian_motion_generator_velocity_limits_violation(false)
    , cartesian_motion_generator_velocity_discontinuity(false)
    , cartesian_motion_generator_acceleration_discontinuity(false)
    , cartesian_motion_generator_elbow_sign_inconsistent(false)
    , cartesian_motion_generator_start_elbow_invalid(false)
    , cartesian_motion_generator_joint_position_limits_violation(false)
    , cartesian_motion_generator_joint_velocity_limits_violation(false)
    , cartesian_motion_generator_joint_velocity_discontinuity(false)
    , cartesian_motion_generator_joint_acceleration_discontinuity(false)
    , cartesian_position_motion_generator_invalid_frame(false)
    , force_controller_desired_force_tolerance_violation(false)
    , controller_torque_discontinuity(false)
    , start_elbow_sign_inconsistent(false)
    , communication_constraints_violation(false)
    , power_limit_violation(false)
    , joint_p2p_insufficient_torque_for_planning(false)
    , tau_j_range_violation(false)
    , instability_detected(false)  {
    }
  Errors_(const ContainerAllocator& _alloc)
    : joint_position_limits_violation(false)
    , cartesian_position_limits_violation(false)
    , self_collision_avoidance_violation(false)
    , joint_velocity_violation(false)
    , cartesian_velocity_violation(false)
    , force_control_safety_violation(false)
    , joint_reflex(false)
    , cartesian_reflex(false)
    , max_goal_pose_deviation_violation(false)
    , max_path_pose_deviation_violation(false)
    , cartesian_velocity_profile_safety_violation(false)
    , joint_position_motion_generator_start_pose_invalid(false)
    , joint_motion_generator_position_limits_violation(false)
    , joint_motion_generator_velocity_limits_violation(false)
    , joint_motion_generator_velocity_discontinuity(false)
    , joint_motion_generator_acceleration_discontinuity(false)
    , cartesian_position_motion_generator_start_pose_invalid(false)
    , cartesian_motion_generator_elbow_limit_violation(false)
    , cartesian_motion_generator_velocity_limits_violation(false)
    , cartesian_motion_generator_velocity_discontinuity(false)
    , cartesian_motion_generator_acceleration_discontinuity(false)
    , cartesian_motion_generator_elbow_sign_inconsistent(false)
    , cartesian_motion_generator_start_elbow_invalid(false)
    , cartesian_motion_generator_joint_position_limits_violation(false)
    , cartesian_motion_generator_joint_velocity_limits_violation(false)
    , cartesian_motion_generator_joint_velocity_discontinuity(false)
    , cartesian_motion_generator_joint_acceleration_discontinuity(false)
    , cartesian_position_motion_generator_invalid_frame(false)
    , force_controller_desired_force_tolerance_violation(false)
    , controller_torque_discontinuity(false)
    , start_elbow_sign_inconsistent(false)
    , communication_constraints_violation(false)
    , power_limit_violation(false)
    , joint_p2p_insufficient_torque_for_planning(false)
    , tau_j_range_violation(false)
    , instability_detected(false)  {
  (void)_alloc;
    }



   typedef uint8_t _joint_position_limits_violation_type;
  _joint_position_limits_violation_type joint_position_limits_violation;

   typedef uint8_t _cartesian_position_limits_violation_type;
  _cartesian_position_limits_violation_type cartesian_position_limits_violation;

   typedef uint8_t _self_collision_avoidance_violation_type;
  _self_collision_avoidance_violation_type self_collision_avoidance_violation;

   typedef uint8_t _joint_velocity_violation_type;
  _joint_velocity_violation_type joint_velocity_violation;

   typedef uint8_t _cartesian_velocity_violation_type;
  _cartesian_velocity_violation_type cartesian_velocity_violation;

   typedef uint8_t _force_control_safety_violation_type;
  _force_control_safety_violation_type force_control_safety_violation;

   typedef uint8_t _joint_reflex_type;
  _joint_reflex_type joint_reflex;

   typedef uint8_t _cartesian_reflex_type;
  _cartesian_reflex_type cartesian_reflex;

   typedef uint8_t _max_goal_pose_deviation_violation_type;
  _max_goal_pose_deviation_violation_type max_goal_pose_deviation_violation;

   typedef uint8_t _max_path_pose_deviation_violation_type;
  _max_path_pose_deviation_violation_type max_path_pose_deviation_violation;

   typedef uint8_t _cartesian_velocity_profile_safety_violation_type;
  _cartesian_velocity_profile_safety_violation_type cartesian_velocity_profile_safety_violation;

   typedef uint8_t _joint_position_motion_generator_start_pose_invalid_type;
  _joint_position_motion_generator_start_pose_invalid_type joint_position_motion_generator_start_pose_invalid;

   typedef uint8_t _joint_motion_generator_position_limits_violation_type;
  _joint_motion_generator_position_limits_violation_type joint_motion_generator_position_limits_violation;

   typedef uint8_t _joint_motion_generator_velocity_limits_violation_type;
  _joint_motion_generator_velocity_limits_violation_type joint_motion_generator_velocity_limits_violation;

   typedef uint8_t _joint_motion_generator_velocity_discontinuity_type;
  _joint_motion_generator_velocity_discontinuity_type joint_motion_generator_velocity_discontinuity;

   typedef uint8_t _joint_motion_generator_acceleration_discontinuity_type;
  _joint_motion_generator_acceleration_discontinuity_type joint_motion_generator_acceleration_discontinuity;

   typedef uint8_t _cartesian_position_motion_generator_start_pose_invalid_type;
  _cartesian_position_motion_generator_start_pose_invalid_type cartesian_position_motion_generator_start_pose_invalid;

   typedef uint8_t _cartesian_motion_generator_elbow_limit_violation_type;
  _cartesian_motion_generator_elbow_limit_violation_type cartesian_motion_generator_elbow_limit_violation;

   typedef uint8_t _cartesian_motion_generator_velocity_limits_violation_type;
  _cartesian_motion_generator_velocity_limits_violation_type cartesian_motion_generator_velocity_limits_violation;

   typedef uint8_t _cartesian_motion_generator_velocity_discontinuity_type;
  _cartesian_motion_generator_velocity_discontinuity_type cartesian_motion_generator_velocity_discontinuity;

   typedef uint8_t _cartesian_motion_generator_acceleration_discontinuity_type;
  _cartesian_motion_generator_acceleration_discontinuity_type cartesian_motion_generator_acceleration_discontinuity;

   typedef uint8_t _cartesian_motion_generator_elbow_sign_inconsistent_type;
  _cartesian_motion_generator_elbow_sign_inconsistent_type cartesian_motion_generator_elbow_sign_inconsistent;

   typedef uint8_t _cartesian_motion_generator_start_elbow_invalid_type;
  _cartesian_motion_generator_start_elbow_invalid_type cartesian_motion_generator_start_elbow_invalid;

   typedef uint8_t _cartesian_motion_generator_joint_position_limits_violation_type;
  _cartesian_motion_generator_joint_position_limits_violation_type cartesian_motion_generator_joint_position_limits_violation;

   typedef uint8_t _cartesian_motion_generator_joint_velocity_limits_violation_type;
  _cartesian_motion_generator_joint_velocity_limits_violation_type cartesian_motion_generator_joint_velocity_limits_violation;

   typedef uint8_t _cartesian_motion_generator_joint_velocity_discontinuity_type;
  _cartesian_motion_generator_joint_velocity_discontinuity_type cartesian_motion_generator_joint_velocity_discontinuity;

   typedef uint8_t _cartesian_motion_generator_joint_acceleration_discontinuity_type;
  _cartesian_motion_generator_joint_acceleration_discontinuity_type cartesian_motion_generator_joint_acceleration_discontinuity;

   typedef uint8_t _cartesian_position_motion_generator_invalid_frame_type;
  _cartesian_position_motion_generator_invalid_frame_type cartesian_position_motion_generator_invalid_frame;

   typedef uint8_t _force_controller_desired_force_tolerance_violation_type;
  _force_controller_desired_force_tolerance_violation_type force_controller_desired_force_tolerance_violation;

   typedef uint8_t _controller_torque_discontinuity_type;
  _controller_torque_discontinuity_type controller_torque_discontinuity;

   typedef uint8_t _start_elbow_sign_inconsistent_type;
  _start_elbow_sign_inconsistent_type start_elbow_sign_inconsistent;

   typedef uint8_t _communication_constraints_violation_type;
  _communication_constraints_violation_type communication_constraints_violation;

   typedef uint8_t _power_limit_violation_type;
  _power_limit_violation_type power_limit_violation;

   typedef uint8_t _joint_p2p_insufficient_torque_for_planning_type;
  _joint_p2p_insufficient_torque_for_planning_type joint_p2p_insufficient_torque_for_planning;

   typedef uint8_t _tau_j_range_violation_type;
  _tau_j_range_violation_type tau_j_range_violation;

   typedef uint8_t _instability_detected_type;
  _instability_detected_type instability_detected;





  typedef boost::shared_ptr< ::franka_msgs::Errors_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_msgs::Errors_<ContainerAllocator> const> ConstPtr;

}; // struct Errors_

typedef ::franka_msgs::Errors_<std::allocator<void> > Errors;

typedef boost::shared_ptr< ::franka_msgs::Errors > ErrorsPtr;
typedef boost::shared_ptr< ::franka_msgs::Errors const> ErrorsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_msgs::Errors_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_msgs::Errors_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::franka_msgs::Errors_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_msgs::Errors_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_msgs::Errors_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_msgs::Errors_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_msgs::Errors_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_msgs::Errors_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_msgs::Errors_<ContainerAllocator> >
{
  static const char* value()
  {
    return "420c43ed3349b66b110f24531e7b53e5";
  }

  static const char* value(const ::franka_msgs::Errors_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x420c43ed3349b66bULL;
  static const uint64_t static_value2 = 0x110f24531e7b53e5ULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_msgs::Errors_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_msgs/Errors";
  }

  static const char* value(const ::franka_msgs::Errors_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_msgs::Errors_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool joint_position_limits_violation\n"
"bool cartesian_position_limits_violation\n"
"bool self_collision_avoidance_violation\n"
"bool joint_velocity_violation\n"
"bool cartesian_velocity_violation\n"
"bool force_control_safety_violation\n"
"bool joint_reflex\n"
"bool cartesian_reflex\n"
"bool max_goal_pose_deviation_violation\n"
"bool max_path_pose_deviation_violation\n"
"bool cartesian_velocity_profile_safety_violation\n"
"bool joint_position_motion_generator_start_pose_invalid\n"
"bool joint_motion_generator_position_limits_violation\n"
"bool joint_motion_generator_velocity_limits_violation\n"
"bool joint_motion_generator_velocity_discontinuity\n"
"bool joint_motion_generator_acceleration_discontinuity\n"
"bool cartesian_position_motion_generator_start_pose_invalid\n"
"bool cartesian_motion_generator_elbow_limit_violation\n"
"bool cartesian_motion_generator_velocity_limits_violation\n"
"bool cartesian_motion_generator_velocity_discontinuity\n"
"bool cartesian_motion_generator_acceleration_discontinuity\n"
"bool cartesian_motion_generator_elbow_sign_inconsistent\n"
"bool cartesian_motion_generator_start_elbow_invalid\n"
"bool cartesian_motion_generator_joint_position_limits_violation\n"
"bool cartesian_motion_generator_joint_velocity_limits_violation\n"
"bool cartesian_motion_generator_joint_velocity_discontinuity\n"
"bool cartesian_motion_generator_joint_acceleration_discontinuity\n"
"bool cartesian_position_motion_generator_invalid_frame\n"
"bool force_controller_desired_force_tolerance_violation\n"
"bool controller_torque_discontinuity\n"
"bool start_elbow_sign_inconsistent\n"
"bool communication_constraints_violation\n"
"bool power_limit_violation\n"
"bool joint_p2p_insufficient_torque_for_planning\n"
"bool tau_j_range_violation\n"
"bool instability_detected\n"
;
  }

  static const char* value(const ::franka_msgs::Errors_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_msgs::Errors_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_position_limits_violation);
      stream.next(m.cartesian_position_limits_violation);
      stream.next(m.self_collision_avoidance_violation);
      stream.next(m.joint_velocity_violation);
      stream.next(m.cartesian_velocity_violation);
      stream.next(m.force_control_safety_violation);
      stream.next(m.joint_reflex);
      stream.next(m.cartesian_reflex);
      stream.next(m.max_goal_pose_deviation_violation);
      stream.next(m.max_path_pose_deviation_violation);
      stream.next(m.cartesian_velocity_profile_safety_violation);
      stream.next(m.joint_position_motion_generator_start_pose_invalid);
      stream.next(m.joint_motion_generator_position_limits_violation);
      stream.next(m.joint_motion_generator_velocity_limits_violation);
      stream.next(m.joint_motion_generator_velocity_discontinuity);
      stream.next(m.joint_motion_generator_acceleration_discontinuity);
      stream.next(m.cartesian_position_motion_generator_start_pose_invalid);
      stream.next(m.cartesian_motion_generator_elbow_limit_violation);
      stream.next(m.cartesian_motion_generator_velocity_limits_violation);
      stream.next(m.cartesian_motion_generator_velocity_discontinuity);
      stream.next(m.cartesian_motion_generator_acceleration_discontinuity);
      stream.next(m.cartesian_motion_generator_elbow_sign_inconsistent);
      stream.next(m.cartesian_motion_generator_start_elbow_invalid);
      stream.next(m.cartesian_motion_generator_joint_position_limits_violation);
      stream.next(m.cartesian_motion_generator_joint_velocity_limits_violation);
      stream.next(m.cartesian_motion_generator_joint_velocity_discontinuity);
      stream.next(m.cartesian_motion_generator_joint_acceleration_discontinuity);
      stream.next(m.cartesian_position_motion_generator_invalid_frame);
      stream.next(m.force_controller_desired_force_tolerance_violation);
      stream.next(m.controller_torque_discontinuity);
      stream.next(m.start_elbow_sign_inconsistent);
      stream.next(m.communication_constraints_violation);
      stream.next(m.power_limit_violation);
      stream.next(m.joint_p2p_insufficient_torque_for_planning);
      stream.next(m.tau_j_range_violation);
      stream.next(m.instability_detected);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Errors_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_msgs::Errors_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_msgs::Errors_<ContainerAllocator>& v)
  {
    s << indent << "joint_position_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_position_limits_violation);
    s << indent << "cartesian_position_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_position_limits_violation);
    s << indent << "self_collision_avoidance_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.self_collision_avoidance_violation);
    s << indent << "joint_velocity_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_velocity_violation);
    s << indent << "cartesian_velocity_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_velocity_violation);
    s << indent << "force_control_safety_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.force_control_safety_violation);
    s << indent << "joint_reflex: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_reflex);
    s << indent << "cartesian_reflex: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_reflex);
    s << indent << "max_goal_pose_deviation_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.max_goal_pose_deviation_violation);
    s << indent << "max_path_pose_deviation_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.max_path_pose_deviation_violation);
    s << indent << "cartesian_velocity_profile_safety_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_velocity_profile_safety_violation);
    s << indent << "joint_position_motion_generator_start_pose_invalid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_position_motion_generator_start_pose_invalid);
    s << indent << "joint_motion_generator_position_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_motion_generator_position_limits_violation);
    s << indent << "joint_motion_generator_velocity_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_motion_generator_velocity_limits_violation);
    s << indent << "joint_motion_generator_velocity_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_motion_generator_velocity_discontinuity);
    s << indent << "joint_motion_generator_acceleration_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_motion_generator_acceleration_discontinuity);
    s << indent << "cartesian_position_motion_generator_start_pose_invalid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_position_motion_generator_start_pose_invalid);
    s << indent << "cartesian_motion_generator_elbow_limit_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_elbow_limit_violation);
    s << indent << "cartesian_motion_generator_velocity_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_velocity_limits_violation);
    s << indent << "cartesian_motion_generator_velocity_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_velocity_discontinuity);
    s << indent << "cartesian_motion_generator_acceleration_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_acceleration_discontinuity);
    s << indent << "cartesian_motion_generator_elbow_sign_inconsistent: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_elbow_sign_inconsistent);
    s << indent << "cartesian_motion_generator_start_elbow_invalid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_start_elbow_invalid);
    s << indent << "cartesian_motion_generator_joint_position_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_joint_position_limits_violation);
    s << indent << "cartesian_motion_generator_joint_velocity_limits_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_joint_velocity_limits_violation);
    s << indent << "cartesian_motion_generator_joint_velocity_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_joint_velocity_discontinuity);
    s << indent << "cartesian_motion_generator_joint_acceleration_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_motion_generator_joint_acceleration_discontinuity);
    s << indent << "cartesian_position_motion_generator_invalid_frame: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cartesian_position_motion_generator_invalid_frame);
    s << indent << "force_controller_desired_force_tolerance_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.force_controller_desired_force_tolerance_violation);
    s << indent << "controller_torque_discontinuity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.controller_torque_discontinuity);
    s << indent << "start_elbow_sign_inconsistent: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.start_elbow_sign_inconsistent);
    s << indent << "communication_constraints_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.communication_constraints_violation);
    s << indent << "power_limit_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.power_limit_violation);
    s << indent << "joint_p2p_insufficient_torque_for_planning: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.joint_p2p_insufficient_torque_for_planning);
    s << indent << "tau_j_range_violation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tau_j_range_violation);
    s << indent << "instability_detected: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.instability_detected);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_MSGS_MESSAGE_ERRORS_H
