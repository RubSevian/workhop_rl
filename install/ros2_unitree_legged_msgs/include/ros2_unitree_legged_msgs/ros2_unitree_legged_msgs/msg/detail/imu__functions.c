// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_unitree_legged_msgs:msg/IMU.idl
// generated code does not contain a copyright notice
#include "ros2_unitree_legged_msgs/msg/detail/imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros2_unitree_legged_msgs__msg__IMU__init(ros2_unitree_legged_msgs__msg__IMU * msg)
{
  if (!msg) {
    return false;
  }
  // quaternion
  // gyroscope
  // accelerometer
  // rpy
  // temperature
  return true;
}

void
ros2_unitree_legged_msgs__msg__IMU__fini(ros2_unitree_legged_msgs__msg__IMU * msg)
{
  if (!msg) {
    return;
  }
  // quaternion
  // gyroscope
  // accelerometer
  // rpy
  // temperature
}

bool
ros2_unitree_legged_msgs__msg__IMU__are_equal(const ros2_unitree_legged_msgs__msg__IMU * lhs, const ros2_unitree_legged_msgs__msg__IMU * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // quaternion
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->quaternion[i] != rhs->quaternion[i]) {
      return false;
    }
  }
  // gyroscope
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyroscope[i] != rhs->gyroscope[i]) {
      return false;
    }
  }
  // accelerometer
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->accelerometer[i] != rhs->accelerometer[i]) {
      return false;
    }
  }
  // rpy
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->rpy[i] != rhs->rpy[i]) {
      return false;
    }
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__IMU__copy(
  const ros2_unitree_legged_msgs__msg__IMU * input,
  ros2_unitree_legged_msgs__msg__IMU * output)
{
  if (!input || !output) {
    return false;
  }
  // quaternion
  for (size_t i = 0; i < 4; ++i) {
    output->quaternion[i] = input->quaternion[i];
  }
  // gyroscope
  for (size_t i = 0; i < 3; ++i) {
    output->gyroscope[i] = input->gyroscope[i];
  }
  // accelerometer
  for (size_t i = 0; i < 3; ++i) {
    output->accelerometer[i] = input->accelerometer[i];
  }
  // rpy
  for (size_t i = 0; i < 3; ++i) {
    output->rpy[i] = input->rpy[i];
  }
  // temperature
  output->temperature = input->temperature;
  return true;
}

ros2_unitree_legged_msgs__msg__IMU *
ros2_unitree_legged_msgs__msg__IMU__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__IMU * msg = (ros2_unitree_legged_msgs__msg__IMU *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__IMU), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_unitree_legged_msgs__msg__IMU));
  bool success = ros2_unitree_legged_msgs__msg__IMU__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_unitree_legged_msgs__msg__IMU__destroy(ros2_unitree_legged_msgs__msg__IMU * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_unitree_legged_msgs__msg__IMU__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_unitree_legged_msgs__msg__IMU__Sequence__init(ros2_unitree_legged_msgs__msg__IMU__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__IMU * data = NULL;

  if (size) {
    data = (ros2_unitree_legged_msgs__msg__IMU *)allocator.zero_allocate(size, sizeof(ros2_unitree_legged_msgs__msg__IMU), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_unitree_legged_msgs__msg__IMU__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_unitree_legged_msgs__msg__IMU__fini(&data[i - 1]);
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
ros2_unitree_legged_msgs__msg__IMU__Sequence__fini(ros2_unitree_legged_msgs__msg__IMU__Sequence * array)
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
      ros2_unitree_legged_msgs__msg__IMU__fini(&array->data[i]);
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

ros2_unitree_legged_msgs__msg__IMU__Sequence *
ros2_unitree_legged_msgs__msg__IMU__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__IMU__Sequence * array = (ros2_unitree_legged_msgs__msg__IMU__Sequence *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__IMU__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_unitree_legged_msgs__msg__IMU__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_unitree_legged_msgs__msg__IMU__Sequence__destroy(ros2_unitree_legged_msgs__msg__IMU__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_unitree_legged_msgs__msg__IMU__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_unitree_legged_msgs__msg__IMU__Sequence__are_equal(const ros2_unitree_legged_msgs__msg__IMU__Sequence * lhs, const ros2_unitree_legged_msgs__msg__IMU__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__IMU__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__IMU__Sequence__copy(
  const ros2_unitree_legged_msgs__msg__IMU__Sequence * input,
  ros2_unitree_legged_msgs__msg__IMU__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_unitree_legged_msgs__msg__IMU);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_unitree_legged_msgs__msg__IMU * data =
      (ros2_unitree_legged_msgs__msg__IMU *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_unitree_legged_msgs__msg__IMU__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_unitree_legged_msgs__msg__IMU__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__IMU__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
