// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_unitree_legged_msgs:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__STRUCT_H_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_cmd'
#include "ros2_unitree_legged_msgs/msg/detail/motor_cmd__struct.h"
// Member 'bms'
#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__struct.h"

/// Struct defined in msg/LowCmd in the package ros2_unitree_legged_msgs.
typedef struct ros2_unitree_legged_msgs__msg__LowCmd
{
  uint8_t head[2];
  uint8_t level_flag;
  uint8_t frame_reserve;
  uint32_t sn[2];
  uint32_t version[2];
  uint16_t band_width;
  ros2_unitree_legged_msgs__msg__MotorCmd motor_cmd[20];
  ros2_unitree_legged_msgs__msg__BmsCmd bms;
  uint8_t wireless_remote[40];
  uint32_t reserve;
  uint32_t crc;
} ros2_unitree_legged_msgs__msg__LowCmd;

// Struct for a sequence of ros2_unitree_legged_msgs__msg__LowCmd.
typedef struct ros2_unitree_legged_msgs__msg__LowCmd__Sequence
{
  ros2_unitree_legged_msgs__msg__LowCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_unitree_legged_msgs__msg__LowCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__STRUCT_H_
