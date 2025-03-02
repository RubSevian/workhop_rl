// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_unitree_legged_msgs:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_STATE__STRUCT_H_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'imu'
#include "ros2_unitree_legged_msgs/msg/detail/imu__struct.h"
// Member 'motor_state'
#include "ros2_unitree_legged_msgs/msg/detail/motor_state__struct.h"
// Member 'bms'
#include "ros2_unitree_legged_msgs/msg/detail/bms_state__struct.h"

/// Struct defined in msg/LowState in the package ros2_unitree_legged_msgs.
typedef struct ros2_unitree_legged_msgs__msg__LowState
{
  uint8_t head[2];
  uint8_t level_flag;
  uint8_t frame_reserve;
  uint32_t sn[2];
  uint32_t version[2];
  uint16_t band_width;
  ros2_unitree_legged_msgs__msg__IMU imu;
  ros2_unitree_legged_msgs__msg__MotorState motor_state[20];
  ros2_unitree_legged_msgs__msg__BmsState bms;
  int16_t foot_force[4];
  int16_t foot_force_est[4];
  uint32_t tick;
  uint8_t wireless_remote[40];
  uint32_t reserve;
  uint32_t crc;
} ros2_unitree_legged_msgs__msg__LowState;

// Struct for a sequence of ros2_unitree_legged_msgs__msg__LowState.
typedef struct ros2_unitree_legged_msgs__msg__LowState__Sequence
{
  ros2_unitree_legged_msgs__msg__LowState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_unitree_legged_msgs__msg__LowState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_STATE__STRUCT_H_
