// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_unitree_legged_msgs:msg/HighState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_unitree_legged_msgs/msg/detail/high_state__rosidl_typesupport_introspection_c.h"
#include "ros2_unitree_legged_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_unitree_legged_msgs/msg/detail/high_state__functions.h"
#include "ros2_unitree_legged_msgs/msg/detail/high_state__struct.h"


// Include directives for member types
// Member `imu`
#include "ros2_unitree_legged_msgs/msg/imu.h"
// Member `imu`
#include "ros2_unitree_legged_msgs/msg/detail/imu__rosidl_typesupport_introspection_c.h"
// Member `motor_state`
#include "ros2_unitree_legged_msgs/msg/motor_state.h"
// Member `motor_state`
#include "ros2_unitree_legged_msgs/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"
// Member `bms`
#include "ros2_unitree_legged_msgs/msg/bms_state.h"
// Member `bms`
#include "ros2_unitree_legged_msgs/msg/detail/bms_state__rosidl_typesupport_introspection_c.h"
// Member `foot_position2body`
// Member `foot_speed2body`
#include "ros2_unitree_legged_msgs/msg/cartesian.h"
// Member `foot_position2body`
// Member `foot_speed2body`
#include "ros2_unitree_legged_msgs/msg/detail/cartesian__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_unitree_legged_msgs__msg__HighState__init(message_memory);
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_fini_function(void * message_memory)
{
  ros2_unitree_legged_msgs__msg__HighState__fini(message_memory);
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__head(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__head(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__head(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__head(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__head(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__head(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__head(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__sn(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__sn(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__sn(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__sn(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__sn(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__sn(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__sn(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__version(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__version(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__version(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__version(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__version(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__version(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__version(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__motor_state(
  const void * untyped_member)
{
  (void)untyped_member;
  return 20;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__motor_state(
  const void * untyped_member, size_t index)
{
  const ros2_unitree_legged_msgs__msg__MotorState * member =
    (const ros2_unitree_legged_msgs__msg__MotorState *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__motor_state(
  void * untyped_member, size_t index)
{
  ros2_unitree_legged_msgs__msg__MotorState * member =
    (ros2_unitree_legged_msgs__msg__MotorState *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__motor_state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_unitree_legged_msgs__msg__MotorState * item =
    ((const ros2_unitree_legged_msgs__msg__MotorState *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__motor_state(untyped_member, index));
  ros2_unitree_legged_msgs__msg__MotorState * value =
    (ros2_unitree_legged_msgs__msg__MotorState *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__motor_state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_unitree_legged_msgs__msg__MotorState * item =
    ((ros2_unitree_legged_msgs__msg__MotorState *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__motor_state(untyped_member, index));
  const ros2_unitree_legged_msgs__msg__MotorState * value =
    (const ros2_unitree_legged_msgs__msg__MotorState *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_force(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_force(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_force(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_force(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_force(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_force(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_force(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_force_est(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_force_est(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_force_est(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_force_est(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_force_est(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_force_est(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_force_est(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__velocity(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__velocity(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__velocity(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__velocity(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__range_obstacle(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__range_obstacle(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__range_obstacle(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__range_obstacle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__range_obstacle(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__range_obstacle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__range_obstacle(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_position2body(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_position2body(
  const void * untyped_member, size_t index)
{
  const ros2_unitree_legged_msgs__msg__Cartesian * member =
    (const ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_position2body(
  void * untyped_member, size_t index)
{
  ros2_unitree_legged_msgs__msg__Cartesian * member =
    (ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_position2body(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_unitree_legged_msgs__msg__Cartesian * item =
    ((const ros2_unitree_legged_msgs__msg__Cartesian *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_position2body(untyped_member, index));
  ros2_unitree_legged_msgs__msg__Cartesian * value =
    (ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_position2body(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_unitree_legged_msgs__msg__Cartesian * item =
    ((ros2_unitree_legged_msgs__msg__Cartesian *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_position2body(untyped_member, index));
  const ros2_unitree_legged_msgs__msg__Cartesian * value =
    (const ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_speed2body(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_speed2body(
  const void * untyped_member, size_t index)
{
  const ros2_unitree_legged_msgs__msg__Cartesian * member =
    (const ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_speed2body(
  void * untyped_member, size_t index)
{
  ros2_unitree_legged_msgs__msg__Cartesian * member =
    (ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_speed2body(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_unitree_legged_msgs__msg__Cartesian * item =
    ((const ros2_unitree_legged_msgs__msg__Cartesian *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_speed2body(untyped_member, index));
  ros2_unitree_legged_msgs__msg__Cartesian * value =
    (ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_speed2body(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_unitree_legged_msgs__msg__Cartesian * item =
    ((ros2_unitree_legged_msgs__msg__Cartesian *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_speed2body(untyped_member, index));
  const ros2_unitree_legged_msgs__msg__Cartesian * value =
    (const ros2_unitree_legged_msgs__msg__Cartesian *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__wireless_remote(
  const void * untyped_member)
{
  (void)untyped_member;
  return 40;
}

const void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__wireless_remote(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__wireless_remote(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__wireless_remote(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__wireless_remote(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__wireless_remote(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__wireless_remote(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array[25] = {
  {
    "head",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, head),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__head,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__head,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__head,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__head,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__head,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "level_flag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, level_flag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame_reserve",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, frame_reserve),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, sn),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__sn,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__sn,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__sn,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__sn,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__sn,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, version),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__version,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__version,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__version,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__version,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__version,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "band_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, band_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, imu),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    20,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, motor_state),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__motor_state,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__motor_state,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__motor_state,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__motor_state,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__motor_state,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, bms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "foot_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, foot_force),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_force,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_force,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_force,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_force,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_force,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "foot_force_est",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, foot_force_est),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_force_est,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_force_est,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_force_est,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_force_est,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_force_est,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gait_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, gait_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "foot_raise_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, foot_raise_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, position),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__position,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__position,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__position,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__position,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, body_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, velocity),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__velocity,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__velocity,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__velocity,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__velocity,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, yaw_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "range_obstacle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, range_obstacle),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__range_obstacle,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__range_obstacle,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__range_obstacle,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__range_obstacle,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__range_obstacle,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "foot_position2body",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, foot_position2body),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_position2body,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_position2body,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_position2body,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_position2body,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_position2body,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "foot_speed2body",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, foot_speed2body),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__foot_speed2body,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__foot_speed2body,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__foot_speed2body,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__foot_speed2body,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__foot_speed2body,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wireless_remote",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    40,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, wireless_remote),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__size_function__HighState__wireless_remote,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_const_function__HighState__wireless_remote,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__get_function__HighState__wireless_remote,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__fetch_function__HighState__wireless_remote,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__assign_function__HighState__wireless_remote,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reserve",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, reserve),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "crc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighState, crc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_members = {
  "ros2_unitree_legged_msgs__msg",  // message namespace
  "HighState",  // message name
  25,  // number of fields
  sizeof(ros2_unitree_legged_msgs__msg__HighState),
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array,  // message members
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_type_support_handle = {
  0,
  &ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_unitree_legged_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, HighState)() {
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, IMU)();
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, MotorState)();
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, BmsState)();
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array[20].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, Cartesian)();
  ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_member_array[21].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, Cartesian)();
  if (!ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_type_support_handle.typesupport_identifier) {
    ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_unitree_legged_msgs__msg__HighState__rosidl_typesupport_introspection_c__HighState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
