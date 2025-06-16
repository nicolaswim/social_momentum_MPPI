// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_social_nav_interfaces:msg/HumanPoseVel.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__rosidl_typesupport_introspection_c.h"
#include "my_social_nav_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__functions.h"
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__struct.h"


// Include directives for member types
// Member `human_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `velocity`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_social_nav_interfaces__msg__HumanPoseVel__init(message_memory);
}

void my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_fini_function(void * message_memory)
{
  my_social_nav_interfaces__msg__HumanPoseVel__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_member_array[3] = {
  {
    "human_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces__msg__HumanPoseVel, human_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces__msg__HumanPoseVel, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces__msg__HumanPoseVel, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_members = {
  "my_social_nav_interfaces__msg",  // message namespace
  "HumanPoseVel",  // message name
  3,  // number of fields
  sizeof(my_social_nav_interfaces__msg__HumanPoseVel),
  my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_member_array,  // message members
  my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_init_function,  // function to initialize message memory (memory has to be allocated)
  my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_type_support_handle = {
  0,
  &my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_social_nav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_social_nav_interfaces, msg, HumanPoseVel)() {
  my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_type_support_handle.typesupport_identifier) {
    my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_social_nav_interfaces__msg__HumanPoseVel__rosidl_typesupport_introspection_c__HumanPoseVel_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
