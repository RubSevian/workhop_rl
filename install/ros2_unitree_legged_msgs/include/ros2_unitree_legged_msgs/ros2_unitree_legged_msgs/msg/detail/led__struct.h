// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_unitree_legged_msgs:msg/LED.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LED__STRUCT_H_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LED in the package ros2_unitree_legged_msgs.
typedef struct ros2_unitree_legged_msgs__msg__LED
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} ros2_unitree_legged_msgs__msg__LED;

// Struct for a sequence of ros2_unitree_legged_msgs__msg__LED.
typedef struct ros2_unitree_legged_msgs__msg__LED__Sequence
{
  ros2_unitree_legged_msgs__msg__LED * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_unitree_legged_msgs__msg__LED__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LED__STRUCT_H_
