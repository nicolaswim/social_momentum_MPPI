// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_social_nav_interfaces:msg/HumanPoseVel.idl
// generated code does not contain a copyright notice

#ifndef MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__STRUCT_H_
#define MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'human_id'
#include "rosidl_runtime_c/string.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/HumanPoseVel in the package my_social_nav_interfaces.
/**
  * HumanPoseVel.msg
 */
typedef struct my_social_nav_interfaces__msg__HumanPoseVel
{
  /// Unique identifier for the human
  rosidl_runtime_c__String human_id;
  /// x, y, z (use x and y)
  geometry_msgs__msg__Point position;
  /// linear x, y, z (use x and y for vx, vy)
  geometry_msgs__msg__Vector3 velocity;
} my_social_nav_interfaces__msg__HumanPoseVel;

// Struct for a sequence of my_social_nav_interfaces__msg__HumanPoseVel.
typedef struct my_social_nav_interfaces__msg__HumanPoseVel__Sequence
{
  my_social_nav_interfaces__msg__HumanPoseVel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_social_nav_interfaces__msg__HumanPoseVel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_SOCIAL_NAV_INTERFACES__MSG__DETAIL__HUMAN_POSE_VEL__STRUCT_H_
