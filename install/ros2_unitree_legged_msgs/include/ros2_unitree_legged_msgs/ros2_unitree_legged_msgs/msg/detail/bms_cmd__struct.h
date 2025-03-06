// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_unitree_legged_msgs:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__STRUCT_H_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BmsCmd in the package ros2_unitree_legged_msgs.
typedef struct ros2_unitree_legged_msgs__msg__BmsCmd
{
  /// off 0xA5
  uint8_t off;
  uint8_t reserve[3];
} ros2_unitree_legged_msgs__msg__BmsCmd;

// Struct for a sequence of ros2_unitree_legged_msgs__msg__BmsCmd.
typedef struct ros2_unitree_legged_msgs__msg__BmsCmd__Sequence
{
  ros2_unitree_legged_msgs__msg__BmsCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_unitree_legged_msgs__msg__BmsCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__STRUCT_H_
