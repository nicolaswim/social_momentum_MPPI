// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_social_nav_interfaces:msg/HumanArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_social_nav_interfaces/msg/detail/human_array__rosidl_typesupport_introspection_c.h"
#include "my_social_nav_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_social_nav_interfaces/msg/detail/human_array__functions.h"
#include "my_social_nav_interfaces/msg/detail/human_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `humans`
#include "my_social_nav_interfaces/msg/human_pose_vel.h"
// Member `humans`
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_social_nav_interfaces__msg__HumanArray__init(message_memory);
}

void my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_fini_function(void * message_memory)
{
  my_social_nav_interfaces__msg__HumanArray__fini(message_memory);
}

size_t my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__size_function__HumanArray__humans(
  const void * untyped_member)
{
  const my_social_nav_interfaces__msg__HumanPoseVel__Sequence * member =
    (const my_social_nav_interfaces__msg__HumanPoseVel__Sequence *)(untyped_member);
  return member->size;
}

const void * my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__get_const_function__HumanArray__humans(
  const void * untyped_member, size_t index)
{
  const my_social_nav_interfaces__msg__HumanPoseVel__Sequence * member =
    (const my_social_nav_interfaces__msg__HumanPoseVel__Sequence *)(untyped_member);
  return &member->data[index];
}

void * my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__get_function__HumanArray__humans(
  void * untyped_member, size_t index)
{
  my_social_nav_interfaces__msg__HumanPoseVel__Sequence * member =
    (my_social_nav_interfaces__msg__HumanPoseVel__Sequence *)(untyped_member);
  return &member->data[index];
}

void my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__fetch_function__HumanArray__humans(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const my_social_nav_interfaces__msg__HumanPoseVel * item =
    ((const my_social_nav_interfaces__msg__HumanPoseVel *)
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__get_const_function__HumanArray__humans(untyped_member, index));
  my_social_nav_interfaces__msg__HumanPoseVel * value =
    (my_social_nav_interfaces__msg__HumanPoseVel *)(untyped_value);
  *value = *item;
}

void my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__assign_function__HumanArray__humans(
  void * untyped_member, size_t index, const void * untyped_value)
{
  my_social_nav_interfaces__msg__HumanPoseVel * item =
    ((my_social_nav_interfaces__msg__HumanPoseVel *)
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__get_function__HumanArray__humans(untyped_member, index));
  const my_social_nav_interfaces__msg__HumanPoseVel * value =
    (const my_social_nav_interfaces__msg__HumanPoseVel *)(untyped_value);
  *item = *value;
}

bool my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__resize_function__HumanArray__humans(
  void * untyped_member, size_t size)
{
  my_social_nav_interfaces__msg__HumanPoseVel__Sequence * member =
    (my_social_nav_interfaces__msg__HumanPoseVel__Sequence *)(untyped_member);
  my_social_nav_interfaces__msg__HumanPoseVel__Sequence__fini(member);
  return my_social_nav_interfaces__msg__HumanPoseVel__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces__msg__HumanArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "humans",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_social_nav_interfaces__msg__HumanArray, humans),  // bytes offset in struct
    NULL,  // default value
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__size_function__HumanArray__humans,  // size() function pointer
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__get_const_function__HumanArray__humans,  // get_const(index) function pointer
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__get_function__HumanArray__humans,  // get(index) function pointer
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__fetch_function__HumanArray__humans,  // fetch(index, &value) function pointer
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__assign_function__HumanArray__humans,  // assign(index, value) function pointer
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__resize_function__HumanArray__humans  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_members = {
  "my_social_nav_interfaces__msg",  // message namespace
  "HumanArray",  // message name
  2,  // number of fields
  sizeof(my_social_nav_interfaces__msg__HumanArray),
  my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_member_array,  // message members
  my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_init_function,  // function to initialize message memory (memory has to be allocated)
  my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_type_support_handle = {
  0,
  &my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_social_nav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_social_nav_interfaces, msg, HumanArray)() {
  my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_social_nav_interfaces, msg, HumanPoseVel)();
  if (!my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_type_support_handle.typesupport_identifier) {
    my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_social_nav_interfaces__msg__HumanArray__rosidl_typesupport_introspection_c__HumanArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
