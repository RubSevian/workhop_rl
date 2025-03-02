// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_unitree_legged_msgs:msg/Cartesian.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__CARTESIAN__STRUCT_H_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__CARTESIAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Cartesian in the package ros2_unitree_legged_msgs.
typedef struct ros2_unitree_legged_msgs__msg__Cartesian
{
  float x;
  float y;
  float z;
} ros2_unitree_legged_msgs__msg__Cartesian;

// Struct for a sequence of ros2_unitree_legged_msgs__msg__Cartesian.
typedef struct ros2_unitree_legged_msgs__msg__Cartesian__Sequence
{
  ros2_unitree_legged_msgs__msg__Cartesian * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_unitree_legged_msgs__msg__Cartesian__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__CARTESIAN__STRUCT_H_
