// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_social_nav_interfaces:msg/HumanPoseVel.idl
// generated code does not contain a copyright notice
#include "my_social_nav_interfaces/msg/detail/human_pose_vel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `human_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
my_social_nav_interfaces__msg__HumanPoseVel__init(my_social_nav_interfaces__msg__HumanPoseVel * msg)
{
  if (!msg) {
    return false;
  }
  // human_id
  if (!rosidl_runtime_c__String__init(&msg->human_id)) {
    my_social_nav_interfaces__msg__HumanPoseVel__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    my_social_nav_interfaces__msg__HumanPoseVel__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    my_social_nav_interfaces__msg__HumanPoseVel__fini(msg);
    return false;
  }
  return true;
}

void
my_social_nav_interfaces__msg__HumanPoseVel__fini(my_social_nav_interfaces__msg__HumanPoseVel * msg)
{
  if (!msg) {
    return;
  }
  // human_id
  rosidl_runtime_c__String__fini(&msg->human_id);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
}

bool
my_social_nav_interfaces__msg__HumanPoseVel__are_equal(const my_social_nav_interfaces__msg__HumanPoseVel * lhs, const my_social_nav_interfaces__msg__HumanPoseVel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // human_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->human_id), &(rhs->human_id)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  return true;
}

bool
my_social_nav_interfaces__msg__HumanPoseVel__copy(
  const my_social_nav_interfaces__msg__HumanPoseVel * input,
  my_social_nav_interfaces__msg__HumanPoseVel * output)
{
  if (!input || !output) {
    return false;
  }
  // human_id
  if (!rosidl_runtime_c__String__copy(
      &(input->human_id), &(output->human_id)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  return true;
}

my_social_nav_interfaces__msg__HumanPoseVel *
my_social_nav_interfaces__msg__HumanPoseVel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_social_nav_interfaces__msg__HumanPoseVel * msg = (my_social_nav_interfaces__msg__HumanPoseVel *)allocator.allocate(sizeof(my_social_nav_interfaces__msg__HumanPoseVel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_social_nav_interfaces__msg__HumanPoseVel));
  bool success = my_social_nav_interfaces__msg__HumanPoseVel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_social_nav_interfaces__msg__HumanPoseVel__destroy(my_social_nav_interfaces__msg__HumanPoseVel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_social_nav_interfaces__msg__HumanPoseVel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_social_nav_interfaces__msg__HumanPoseVel__Sequence__init(my_social_nav_interfaces__msg__HumanPoseVel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_social_nav_interfaces__msg__HumanPoseVel * data = NULL;

  if (size) {
    data = (my_social_nav_interfaces__msg__HumanPoseVel *)allocator.zero_allocate(size, sizeof(my_social_nav_interfaces__msg__HumanPoseVel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_social_nav_interfaces__msg__HumanPoseVel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_social_nav_interfaces__msg__HumanPoseVel__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_social_nav_interfaces__msg__HumanPoseVel__Sequence__fini(my_social_nav_interfaces__msg__HumanPoseVel__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_social_nav_interfaces__msg__HumanPoseVel__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_social_nav_interfaces__msg__HumanPoseVel__Sequence *
my_social_nav_interfaces__msg__HumanPoseVel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_social_nav_interfaces__msg__HumanPoseVel__Sequence * array = (my_social_nav_interfaces__msg__HumanPoseVel__Sequence *)allocator.allocate(sizeof(my_social_nav_interfaces__msg__HumanPoseVel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_social_nav_interfaces__msg__HumanPoseVel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_social_nav_interfaces__msg__HumanPoseVel__Sequence__destroy(my_social_nav_interfaces__msg__HumanPoseVel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_social_nav_interfaces__msg__HumanPoseVel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_social_nav_interfaces__msg__HumanPoseVel__Sequence__are_equal(const my_social_nav_interfaces__msg__HumanPoseVel__Sequence * lhs, const my_social_nav_interfaces__msg__HumanPoseVel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_social_nav_interfaces__msg__HumanPoseVel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_social_nav_interfaces__msg__HumanPoseVel__Sequence__copy(
  const my_social_nav_interfaces__msg__HumanPoseVel__Sequence * input,
  my_social_nav_interfaces__msg__HumanPoseVel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_social_nav_interfaces__msg__HumanPoseVel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_social_nav_interfaces__msg__HumanPoseVel * data =
      (my_social_nav_interfaces__msg__HumanPoseVel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_social_nav_interfaces__msg__HumanPoseVel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_social_nav_interfaces__msg__HumanPoseVel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_social_nav_interfaces__msg__HumanPoseVel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
