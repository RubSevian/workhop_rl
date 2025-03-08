// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_unitree_legged_msgs:msg/Cartesian.idl
// generated code does not contain a copyright notice
#include "ros2_unitree_legged_msgs/msg/detail/cartesian__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros2_unitree_legged_msgs__msg__Cartesian__init(ros2_unitree_legged_msgs__msg__Cartesian * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  return true;
}

void
ros2_unitree_legged_msgs__msg__Cartesian__fini(ros2_unitree_legged_msgs__msg__Cartesian * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
}

bool
ros2_unitree_legged_msgs__msg__Cartesian__are_equal(const ros2_unitree_legged_msgs__msg__Cartesian * lhs, const ros2_unitree_legged_msgs__msg__Cartesian * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__Cartesian__copy(
  const ros2_unitree_legged_msgs__msg__Cartesian * input,
  ros2_unitree_legged_msgs__msg__Cartesian * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

ros2_unitree_legged_msgs__msg__Cartesian *
ros2_unitree_legged_msgs__msg__Cartesian__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__Cartesian * msg = (ros2_unitree_legged_msgs__msg__Cartesian *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__Cartesian), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_unitree_legged_msgs__msg__Cartesian));
  bool success = ros2_unitree_legged_msgs__msg__Cartesian__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_unitree_legged_msgs__msg__Cartesian__destroy(ros2_unitree_legged_msgs__msg__Cartesian * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_unitree_legged_msgs__msg__Cartesian__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_unitree_legged_msgs__msg__Cartesian__Sequence__init(ros2_unitree_legged_msgs__msg__Cartesian__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__Cartesian * data = NULL;

  if (size) {
    data = (ros2_unitree_legged_msgs__msg__Cartesian *)allocator.zero_allocate(size, sizeof(ros2_unitree_legged_msgs__msg__Cartesian), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_unitree_legged_msgs__msg__Cartesian__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_unitree_legged_msgs__msg__Cartesian__fini(&data[i - 1]);
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
ros2_unitree_legged_msgs__msg__Cartesian__Sequence__fini(ros2_unitree_legged_msgs__msg__Cartesian__Sequence * array)
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
      ros2_unitree_legged_msgs__msg__Cartesian__fini(&array->data[i]);
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

ros2_unitree_legged_msgs__msg__Cartesian__Sequence *
ros2_unitree_legged_msgs__msg__Cartesian__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__Cartesian__Sequence * array = (ros2_unitree_legged_msgs__msg__Cartesian__Sequence *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__Cartesian__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_unitree_legged_msgs__msg__Cartesian__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_unitree_legged_msgs__msg__Cartesian__Sequence__destroy(ros2_unitree_legged_msgs__msg__Cartesian__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_unitree_legged_msgs__msg__Cartesian__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_unitree_legged_msgs__msg__Cartesian__Sequence__are_equal(const ros2_unitree_legged_msgs__msg__Cartesian__Sequence * lhs, const ros2_unitree_legged_msgs__msg__Cartesian__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__Cartesian__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__Cartesian__Sequence__copy(
  const ros2_unitree_legged_msgs__msg__Cartesian__Sequence * input,
  ros2_unitree_legged_msgs__msg__Cartesian__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_unitree_legged_msgs__msg__Cartesian);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_unitree_legged_msgs__msg__Cartesian * data =
      (ros2_unitree_legged_msgs__msg__Cartesian *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_unitree_legged_msgs__msg__Cartesian__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_unitree_legged_msgs__msg__Cartesian__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__Cartesian__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
