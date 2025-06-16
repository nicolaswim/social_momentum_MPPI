// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_social_nav_interfaces:msg/HumanPoseVel.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__TRAITS_HPP_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_social_nav_interfaces/msg/detail/human_pose_vel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace my_social_nav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const HumanPoseVel & msg,
  std::ostream & out)
{
  out << "{";
  // member: human_id
  {
    out << "human_id: ";
    rosidl_generator_traits::value_to_yaml(msg.human_id, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HumanPoseVel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: human_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "human_id: ";
    rosidl_generator_traits::value_to_yaml(msg.human_id, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HumanPoseVel & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace my_social_nav_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use my_social_nav_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_social_nav_interfaces::msg::HumanPoseVel & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_social_nav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_social_nav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_social_nav_interfaces::msg::HumanPoseVel & msg)
{
  return my_social_nav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_social_nav_interfaces::msg::HumanPoseVel>()
{
  return "my_social_nav_interfaces::msg::HumanPoseVel";
}

template<>
inline const char * name<my_social_nav_interfaces::msg::HumanPoseVel>()
{
  return "my_social_nav_interfaces/msg/HumanPoseVel";
}

template<>
struct has_fixed_size<my_social_nav_interfaces::msg::HumanPoseVel>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_social_nav_interfaces::msg::HumanPoseVel>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_social_nav_interfaces::msg::HumanPoseVel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__TRAITS_HPP_
