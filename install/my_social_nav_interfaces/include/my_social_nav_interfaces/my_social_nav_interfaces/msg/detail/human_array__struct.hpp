// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_social_nav_interfaces:msg/HumanArray.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__STRUCT_HPP_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'humans'
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_social_nav_interfaces__msg__HumanArray __attribute__((deprecated))
#else
# define DEPRECATED__my_social_nav_interfaces__msg__HumanArray __declspec(deprecated)
#endif

namespace my_social_nav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HumanArray_
{
  using Type = HumanArray_<ContainerAllocator>;

  explicit HumanArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit HumanArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _humans_type =
    std::vector<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>>;
  _humans_type humans;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__humans(
    const std::vector<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_social_nav_interfaces::msg::HumanPoseVel_<ContainerAllocator>>> & _arg)
  {
    this->humans = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_social_nav_interfaces__msg__HumanArray
    std::shared_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_social_nav_interfaces__msg__HumanArray
    std::shared_ptr<my_social_nav_interfaces::msg::HumanArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HumanArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->humans != other.humans) {
      return false;
    }
    return true;
  }
  bool operator!=(const HumanArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HumanArray_

// alias to use template instance with default allocator
using HumanArray =
  my_social_nav_interfaces::msg::HumanArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_social_nav_interfaces

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__STRUCT_HPP_
