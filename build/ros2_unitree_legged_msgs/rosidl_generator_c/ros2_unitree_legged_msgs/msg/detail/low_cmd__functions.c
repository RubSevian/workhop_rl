// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_unitree_legged_msgs:msg/LowCmd.idl
// generated code does not contain a copyright notice
#include "ros2_unitree_legged_msgs/msg/detail/low_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motor_cmd`
#include "ros2_unitree_legged_msgs/msg/detail/motor_cmd__functions.h"
// Member `bms`
#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__functions.h"

bool
ros2_unitree_legged_msgs__msg__LowCmd__init(ros2_unitree_legged_msgs__msg__LowCmd * msg)
{
  if (!msg) {
    return false;
  }
  // head
  // level_flag
  // frame_reserve
  // sn
  // version
  // band_width
  // motor_cmd
  for (size_t i = 0; i < 20; ++i) {
    if (!ros2_unitree_legged_msgs__msg__MotorCmd__init(&msg->motor_cmd[i])) {
      ros2_unitree_legged_msgs__msg__LowCmd__fini(msg);
      return false;
    }
  }
  // bms
  if (!ros2_unitree_legged_msgs__msg__BmsCmd__init(&msg->bms)) {
    ros2_unitree_legged_msgs__msg__LowCmd__fini(msg);
    return false;
  }
  // wireless_remote
  // reserve
  // crc
  return true;
}

void
ros2_unitree_legged_msgs__msg__LowCmd__fini(ros2_unitree_legged_msgs__msg__LowCmd * msg)
{
  if (!msg) {
    return;
  }
  // head
  // level_flag
  // frame_reserve
  // sn
  // version
  // band_width
  // motor_cmd
  for (size_t i = 0; i < 20; ++i) {
    ros2_unitree_legged_msgs__msg__MotorCmd__fini(&msg->motor_cmd[i]);
  }
  // bms
  ros2_unitree_legged_msgs__msg__BmsCmd__fini(&msg->bms);
  // wireless_remote
  // reserve
  // crc
}

bool
ros2_unitree_legged_msgs__msg__LowCmd__are_equal(const ros2_unitree_legged_msgs__msg__LowCmd * lhs, const ros2_unitree_legged_msgs__msg__LowCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // head
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->head[i] != rhs->head[i]) {
      return false;
    }
  }
  // level_flag
  if (lhs->level_flag != rhs->level_flag) {
    return false;
  }
  // frame_reserve
  if (lhs->frame_reserve != rhs->frame_reserve) {
    return false;
  }
  // sn
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->sn[i] != rhs->sn[i]) {
      return false;
    }
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->version[i] != rhs->version[i]) {
      return false;
    }
  }
  // band_width
  if (lhs->band_width != rhs->band_width) {
    return false;
  }
  // motor_cmd
  for (size_t i = 0; i < 20; ++i) {
    if (!ros2_unitree_legged_msgs__msg__MotorCmd__are_equal(
        &(lhs->motor_cmd[i]), &(rhs->motor_cmd[i])))
    {
      return false;
    }
  }
  // bms
  if (!ros2_unitree_legged_msgs__msg__BmsCmd__are_equal(
      &(lhs->bms), &(rhs->bms)))
  {
    return false;
  }
  // wireless_remote
  for (size_t i = 0; i < 40; ++i) {
    if (lhs->wireless_remote[i] != rhs->wireless_remote[i]) {
      return false;
    }
  }
  // reserve
  if (lhs->reserve != rhs->reserve) {
    return false;
  }
  // crc
  if (lhs->crc != rhs->crc) {
    return false;
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__LowCmd__copy(
  const ros2_unitree_legged_msgs__msg__LowCmd * input,
  ros2_unitree_legged_msgs__msg__LowCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // head
  for (size_t i = 0; i < 2; ++i) {
    output->head[i] = input->head[i];
  }
  // level_flag
  output->level_flag = input->level_flag;
  // frame_reserve
  output->frame_reserve = input->frame_reserve;
  // sn
  for (size_t i = 0; i < 2; ++i) {
    output->sn[i] = input->sn[i];
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    output->version[i] = input->version[i];
  }
  // band_width
  output->band_width = input->band_width;
  // motor_cmd
  for (size_t i = 0; i < 20; ++i) {
    if (!ros2_unitree_legged_msgs__msg__MotorCmd__copy(
        &(input->motor_cmd[i]), &(output->motor_cmd[i])))
    {
      return false;
    }
  }
  // bms
  if (!ros2_unitree_legged_msgs__msg__BmsCmd__copy(
      &(input->bms), &(output->bms)))
  {
    return false;
  }
  // wireless_remote
  for (size_t i = 0; i < 40; ++i) {
    output->wireless_remote[i] = input->wireless_remote[i];
  }
  // reserve
  output->reserve = input->reserve;
  // crc
  output->crc = input->crc;
  return true;
}

ros2_unitree_legged_msgs__msg__LowCmd *
ros2_unitree_legged_msgs__msg__LowCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__LowCmd * msg = (ros2_unitree_legged_msgs__msg__LowCmd *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__LowCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_unitree_legged_msgs__msg__LowCmd));
  bool success = ros2_unitree_legged_msgs__msg__LowCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_unitree_legged_msgs__msg__LowCmd__destroy(ros2_unitree_legged_msgs__msg__LowCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_unitree_legged_msgs__msg__LowCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_unitree_legged_msgs__msg__LowCmd__Sequence__init(ros2_unitree_legged_msgs__msg__LowCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__LowCmd * data = NULL;

  if (size) {
    data = (ros2_unitree_legged_msgs__msg__LowCmd *)allocator.zero_allocate(size, sizeof(ros2_unitree_legged_msgs__msg__LowCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_unitree_legged_msgs__msg__LowCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_unitree_legged_msgs__msg__LowCmd__fini(&data[i - 1]);
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
ros2_unitree_legged_msgs__msg__LowCmd__Sequence__fini(ros2_unitree_legged_msgs__msg__LowCmd__Sequence * array)
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
      ros2_unitree_legged_msgs__msg__LowCmd__fini(&array->data[i]);
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

ros2_unitree_legged_msgs__msg__LowCmd__Sequence *
ros2_unitree_legged_msgs__msg__LowCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__LowCmd__Sequence * array = (ros2_unitree_legged_msgs__msg__LowCmd__Sequence *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__LowCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_unitree_legged_msgs__msg__LowCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_unitree_legged_msgs__msg__LowCmd__Sequence__destroy(ros2_unitree_legged_msgs__msg__LowCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_unitree_legged_msgs__msg__LowCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_unitree_legged_msgs__msg__LowCmd__Sequence__are_equal(const ros2_unitree_legged_msgs__msg__LowCmd__Sequence * lhs, const ros2_unitree_legged_msgs__msg__LowCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__LowCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__LowCmd__Sequence__copy(
  const ros2_unitree_legged_msgs__msg__LowCmd__Sequence * input,
  ros2_unitree_legged_msgs__msg__LowCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_unitree_legged_msgs__msg__LowCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_unitree_legged_msgs__msg__LowCmd * data =
      (ros2_unitree_legged_msgs__msg__LowCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_unitree_legged_msgs__msg__LowCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_unitree_legged_msgs__msg__LowCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__LowCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
