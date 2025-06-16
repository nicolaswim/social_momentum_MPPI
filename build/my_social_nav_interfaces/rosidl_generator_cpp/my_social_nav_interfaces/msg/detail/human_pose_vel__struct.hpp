// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_social_nav_interfaces:msg/HumanPoseVel.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__STRUCT_HPP_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_social_nav_interfaces__msg__HumanPoseVel __attribute__((deprecated))
#else
# define DEPRECATED__my_social_nav_interfaces__msg__HumanPoseVel __declspec(deprecated)
#endif

namespace my_social_nav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HumanPoseVel_
{
  using Type = HumanPoseVel_<ContainerAllocator>;

  explicit HumanPoseVel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->human_id = "";
    }
  }

  explicit HumanPoseVel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : human_id(_alloc),
    position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->human_id = "";
    }
  }

  // field types and members
  using _human_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _human_id_type human_id;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__human_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->human_id = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_social_nav_interfaces__msg__HumanPoseVel
    std::shared_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_social_nav_interfaces__msg__HumanPoseVel
    std::shared_ptr<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HumanPoseVel_ & other) const
  {
    if (this->human_id != other.human_id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const HumanPoseVel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HumanPoseVel_

// alias to use template instance with default allocator
using HumanPoseVel =
  my_social_nav_interfaces::msg::HumanPoseVel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_social_nav_interfaces

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__STRUCT_HPP_
