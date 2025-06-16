// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_social_nav_interfaces:msg/HumanPoseVel.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__BUILDER_HPP_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_social_nav_interfaces/msg/detail/human_pose_vel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_social_nav_interfaces
{

namespace msg
{

namespace builder
{

class Init_HumanPoseVel_velocity
{
public:
  explicit Init_HumanPoseVel_velocity(::my_social_nav_interfaces::msg::HumanPoseVel & msg)
  : msg_(msg)
  {}
  ::my_social_nav_interfaces::msg::HumanPoseVel velocity(::my_social_nav_interfaces::msg::HumanPoseVel::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_social_nav_interfaces::msg::HumanPoseVel msg_;
};

class Init_HumanPoseVel_position
{
public:
  explicit Init_HumanPoseVel_position(::my_social_nav_interfaces::msg::HumanPoseVel & msg)
  : msg_(msg)
  {}
  Init_HumanPoseVel_velocity position(::my_social_nav_interfaces::msg::HumanPoseVel::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_HumanPoseVel_velocity(msg_);
  }

private:
  ::my_social_nav_interfaces::msg::HumanPoseVel msg_;
};

class Init_HumanPoseVel_human_id
{
public:
  Init_HumanPoseVel_human_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HumanPoseVel_position human_id(::my_social_nav_interfaces::msg::HumanPoseVel::_human_id_type arg)
  {
    msg_.human_id = std::move(arg);
    return Init_HumanPoseVel_position(msg_);
  }

private:
  ::my_social_nav_interfaces::msg::HumanPoseVel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_social_nav_interfaces::msg::HumanPoseVel>()
{
  return my_social_nav_interfaces::msg::builder::Init_HumanPoseVel_human_id();
}

}  // namespace my_social_nav_interfaces

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__BUILDER_HPP_
