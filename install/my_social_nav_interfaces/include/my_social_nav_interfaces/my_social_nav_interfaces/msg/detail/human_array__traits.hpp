// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_social_nav_interfaces:msg/HumanArray.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__TRAITS_HPP_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_social_nav_interfaces/msg/detail/human_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'humans'
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__traits.hpp"

namespace my_social_nav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const HumanArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: humans
  {
    if (msg.humans.size() == 0) {
      out << "humans: []";
    } else {
      out << "humans: [";
      size_t pending_items = msg.humans.size();
      for (auto item : msg.humans) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HumanArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: humans
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.humans.size() == 0) {
      out << "humans: []\n";
    } else {
      out << "humans:\n";
      for (auto item : msg.humans) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HumanArray & msg, bool use_flow_style = false)
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
  const my_social_nav_interfaces::msg::HumanArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_social_nav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_social_nav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_social_nav_interfaces::msg::HumanArray & msg)
{
  return my_social_nav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_social_nav_interfaces::msg::HumanArray>()
{
  return "my_social_nav_interfaces::msg::HumanArray";
}

template<>
inline const char * name<my_social_nav_interfaces::msg::HumanArray>()
{
  return "my_social_nav_interfaces/msg/HumanArray";
}

template<>
struct has_fixed_size<my_social_nav_interfaces::msg::HumanArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_social_nav_interfaces::msg::HumanArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_social_nav_interfaces::msg::HumanArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__TRAITS_HPP_
