// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_unitree_legged_msgs:msg/HighCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__STRUCT_H_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bms'
#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__struct.h"
// Member 'led'
#include "ros2_unitree_legged_msgs/msg/detail/led__struct.h"

/// Struct defined in msg/HighCmd in the package ros2_unitree_legged_msgs.
typedef struct ros2_unitree_legged_msgs__msg__HighCmd
{
  uint8_t head[2];
  uint8_t level_flag;
  uint8_t frame_reserve;
  uint32_t sn[2];
  uint32_t version[2];
  uint16_t band_width;
  uint8_t mode;
  uint8_t gait_type;
  uint8_t speed_level;
  float foot_raise_height;
  float body_height;
  float position[2];
  float euler[3];
  float velocity[2];
  float yaw_speed;
  ros2_unitree_legged_msgs__msg__BmsCmd bms;
  ros2_unitree_legged_msgs__msg__LED led[4];
  uint8_t wireless_remote[40];
  uint32_t reserve;
  uint32_t crc;
} ros2_unitree_legged_msgs__msg__HighCmd;

// Struct for a sequence of ros2_unitree_legged_msgs__msg__HighCmd.
typedef struct ros2_unitree_legged_msgs__msg__HighCmd__Sequence
{
  ros2_unitree_legged_msgs__msg__HighCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_unitree_legged_msgs__msg__HighCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__STRUCT_H_
