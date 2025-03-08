// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_unitree_legged_msgs:msg/BmsState.idl
// generated code does not contain a copyright notice
#include "ros2_unitree_legged_msgs/msg/detail/bms_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros2_unitree_legged_msgs__msg__BmsState__init(ros2_unitree_legged_msgs__msg__BmsState * msg)
{
  if (!msg) {
    return false;
  }
  // version_h
  // version_l
  // bms_status
  // soc
  // current
  // cycle
  // bq_ntc
  // mcu_ntc
  // cell_vol
  return true;
}

void
ros2_unitree_legged_msgs__msg__BmsState__fini(ros2_unitree_legged_msgs__msg__BmsState * msg)
{
  if (!msg) {
    return;
  }
  // version_h
  // version_l
  // bms_status
  // soc
  // current
  // cycle
  // bq_ntc
  // mcu_ntc
  // cell_vol
}

bool
ros2_unitree_legged_msgs__msg__BmsState__are_equal(const ros2_unitree_legged_msgs__msg__BmsState * lhs, const ros2_unitree_legged_msgs__msg__BmsState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version_h
  if (lhs->version_h != rhs->version_h) {
    return false;
  }
  // version_l
  if (lhs->version_l != rhs->version_l) {
    return false;
  }
  // bms_status
  if (lhs->bms_status != rhs->bms_status) {
    return false;
  }
  // soc
  if (lhs->soc != rhs->soc) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // cycle
  if (lhs->cycle != rhs->cycle) {
    return false;
  }
  // bq_ntc
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->bq_ntc[i] != rhs->bq_ntc[i]) {
      return false;
    }
  }
  // mcu_ntc
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->mcu_ntc[i] != rhs->mcu_ntc[i]) {
      return false;
    }
  }
  // cell_vol
  for (size_t i = 0; i < 10; ++i) {
    if (lhs->cell_vol[i] != rhs->cell_vol[i]) {
      return false;
    }
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__BmsState__copy(
  const ros2_unitree_legged_msgs__msg__BmsState * input,
  ros2_unitree_legged_msgs__msg__BmsState * output)
{
  if (!input || !output) {
    return false;
  }
  // version_h
  output->version_h = input->version_h;
  // version_l
  output->version_l = input->version_l;
  // bms_status
  output->bms_status = input->bms_status;
  // soc
  output->soc = input->soc;
  // current
  output->current = input->current;
  // cycle
  output->cycle = input->cycle;
  // bq_ntc
  for (size_t i = 0; i < 2; ++i) {
    output->bq_ntc[i] = input->bq_ntc[i];
  }
  // mcu_ntc
  for (size_t i = 0; i < 2; ++i) {
    output->mcu_ntc[i] = input->mcu_ntc[i];
  }
  // cell_vol
  for (size_t i = 0; i < 10; ++i) {
    output->cell_vol[i] = input->cell_vol[i];
  }
  return true;
}

ros2_unitree_legged_msgs__msg__BmsState *
ros2_unitree_legged_msgs__msg__BmsState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__BmsState * msg = (ros2_unitree_legged_msgs__msg__BmsState *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__BmsState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_unitree_legged_msgs__msg__BmsState));
  bool success = ros2_unitree_legged_msgs__msg__BmsState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_unitree_legged_msgs__msg__BmsState__destroy(ros2_unitree_legged_msgs__msg__BmsState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_unitree_legged_msgs__msg__BmsState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_unitree_legged_msgs__msg__BmsState__Sequence__init(ros2_unitree_legged_msgs__msg__BmsState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__BmsState * data = NULL;

  if (size) {
    data = (ros2_unitree_legged_msgs__msg__BmsState *)allocator.zero_allocate(size, sizeof(ros2_unitree_legged_msgs__msg__BmsState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_unitree_legged_msgs__msg__BmsState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_unitree_legged_msgs__msg__BmsState__fini(&data[i - 1]);
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
ros2_unitree_legged_msgs__msg__BmsState__Sequence__fini(ros2_unitree_legged_msgs__msg__BmsState__Sequence * array)
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
      ros2_unitree_legged_msgs__msg__BmsState__fini(&array->data[i]);
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

ros2_unitree_legged_msgs__msg__BmsState__Sequence *
ros2_unitree_legged_msgs__msg__BmsState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_unitree_legged_msgs__msg__BmsState__Sequence * array = (ros2_unitree_legged_msgs__msg__BmsState__Sequence *)allocator.allocate(sizeof(ros2_unitree_legged_msgs__msg__BmsState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_unitree_legged_msgs__msg__BmsState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_unitree_legged_msgs__msg__BmsState__Sequence__destroy(ros2_unitree_legged_msgs__msg__BmsState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_unitree_legged_msgs__msg__BmsState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_unitree_legged_msgs__msg__BmsState__Sequence__are_equal(const ros2_unitree_legged_msgs__msg__BmsState__Sequence * lhs, const ros2_unitree_legged_msgs__msg__BmsState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__BmsState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_unitree_legged_msgs__msg__BmsState__Sequence__copy(
  const ros2_unitree_legged_msgs__msg__BmsState__Sequence * input,
  ros2_unitree_legged_msgs__msg__BmsState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_unitree_legged_msgs__msg__BmsState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_unitree_legged_msgs__msg__BmsState * data =
      (ros2_unitree_legged_msgs__msg__BmsState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_unitree_legged_msgs__msg__BmsState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_unitree_legged_msgs__msg__BmsState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_unitree_legged_msgs__msg__BmsState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
