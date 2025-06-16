// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_social_nav_interfaces:msg/HumanArray.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__BUILDER_HPP_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_social_nav_interfaces/msg/detail/human_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_social_nav_interfaces
{

namespace msg
{

namespace builder
{

class Init_HumanArray_humans
{
public:
  explicit Init_HumanArray_humans(::my_social_nav_interfaces::msg::HumanArray & msg)
  : msg_(msg)
  {}
  ::my_social_nav_interfaces::msg::HumanArray humans(::my_social_nav_interfaces::msg::HumanArray::_humans_type arg)
  {
    msg_.humans = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_social_nav_interfaces::msg::HumanArray msg_;
};

class Init_HumanArray_header
{
public:
  Init_HumanArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HumanArray_humans header(::my_social_nav_interfaces::msg::HumanArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HumanArray_humans(msg_);
  }

private:
  ::my_social_nav_interfaces::msg::HumanArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_social_nav_interfaces::msg::HumanArray>()
{
  return my_social_nav_interfaces::msg::builder::Init_HumanArray_header();
}

}  // namespace my_social_nav_interfaces

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__BUILDER_HPP_
