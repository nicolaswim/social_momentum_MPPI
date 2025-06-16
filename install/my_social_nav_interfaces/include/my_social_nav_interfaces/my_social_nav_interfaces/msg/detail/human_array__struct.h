// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_social_nav_interfaces:msg/HumanArray.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__STRUCT_H_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'humans'
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__struct.h"

/// Struct defined in msg/HumanArray in the package my_social_nav_interfaces.
/**
  * HumanArray.msg
 */
typedef struct my_social_nav_interfaces__msg__HumanArray
{
  /// Timestamp and frame ID
  std_msgs__msg__Header header;
  /// Array of detected/simulated humans
  my_social_nav_interfaces__msg__HumanPoseVel__Sequence humans;
} my_social_nav_interfaces__msg__HumanArray;

// Struct for a sequence of my_social_nav_interfaces__msg__HumanArray.
typedef struct my_social_nav_interfaces__msg__HumanArray__Sequence
{
  my_social_nav_interfaces__msg__HumanArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_social_nav_interfaces__msg__HumanArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_ARRAY__STRUCT_H_
