// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_unitree_legged_msgs:msg/HighCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__rosidl_typesupport_introspection_c.h"
#include "ros2_unitree_legged_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__functions.h"
#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__struct.h"


// Include directives for member types
// Member `bms`
#include "ros2_unitree_legged_msgs/msg/bms_cmd.h"
// Member `bms`
#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__rosidl_typesupport_introspection_c.h"
// Member `led`
#include "ros2_unitree_legged_msgs/msg/led.h"
// Member `led`
#include "ros2_unitree_legged_msgs/msg/detail/led__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_unitree_legged_msgs__msg__HighCmd__init(message_memory);
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_fini_function(void * message_memory)
{
  ros2_unitree_legged_msgs__msg__HighCmd__fini(message_memory);
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__head(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__head(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__head(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__head(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__head(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__head(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__head(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__sn(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__sn(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__sn(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__sn(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__sn(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__sn(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__sn(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__version(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__version(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__version(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__version(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__version(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__version(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__version(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__euler(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__euler(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__euler(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__euler(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__euler(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__euler(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__euler(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__velocity(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__velocity(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__velocity(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__velocity(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__led(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__led(
  const void * untyped_member, size_t index)
{
  const ros2_unitree_legged_msgs__msg__LED * member =
    (const ros2_unitree_legged_msgs__msg__LED *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__led(
  void * untyped_member, size_t index)
{
  ros2_unitree_legged_msgs__msg__LED * member =
    (ros2_unitree_legged_msgs__msg__LED *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__led(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_unitree_legged_msgs__msg__LED * item =
    ((const ros2_unitree_legged_msgs__msg__LED *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__led(untyped_member, index));
  ros2_unitree_legged_msgs__msg__LED * value =
    (ros2_unitree_legged_msgs__msg__LED *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__led(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_unitree_legged_msgs__msg__LED * item =
    ((ros2_unitree_legged_msgs__msg__LED *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__led(untyped_member, index));
  const ros2_unitree_legged_msgs__msg__LED * value =
    (const ros2_unitree_legged_msgs__msg__LED *)(untyped_value);
  *item = *value;
}

size_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__wireless_remote(
  const void * untyped_member)
{
  (void)untyped_member;
  return 40;
}

const void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__wireless_remote(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__wireless_remote(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__wireless_remote(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__wireless_remote(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__wireless_remote(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__wireless_remote(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_member_array[20] = {
  {
    "head",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, head),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__head,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__head,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__head,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__head,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__head,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, level_flag),  // bytes offset in struct
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, frame_reserve),  // bytes offset in struct
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, sn),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__sn,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__sn,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__sn,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__sn,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__sn,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, version),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__version,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__version,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__version,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__version,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__version,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, band_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, mode),  // bytes offset in struct
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, gait_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, speed_level),  // bytes offset in struct
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, foot_raise_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, body_height),  // bytes offset in struct
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
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, position),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__position,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__position,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__position,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__position,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "euler",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, euler),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__euler,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__euler,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__euler,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__euler,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__euler,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, velocity),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__velocity,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__velocity,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__velocity,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__velocity,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__velocity,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, yaw_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, bms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "led",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, led),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__led,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__led,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__led,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__led,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__led,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, wireless_remote),  // bytes offset in struct
    NULL,  // default value
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__size_function__HighCmd__wireless_remote,  // size() function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_const_function__HighCmd__wireless_remote,  // get_const(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__get_function__HighCmd__wireless_remote,  // get(index) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__fetch_function__HighCmd__wireless_remote,  // fetch(index, &value) function pointer
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__assign_function__HighCmd__wireless_remote,  // assign(index, value) function pointer
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, reserve),  // bytes offset in struct
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
    offsetof(ros2_unitree_legged_msgs__msg__HighCmd, crc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_members = {
  "ros2_unitree_legged_msgs__msg",  // message namespace
  "HighCmd",  // message name
  20,  // number of fields
  sizeof(ros2_unitree_legged_msgs__msg__HighCmd),
  ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_member_array,  // message members
  ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_type_support_handle = {
  0,
  &ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_unitree_legged_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, HighCmd)() {
  ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_member_array[15].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, BmsCmd)();
  ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_member_array[16].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_unitree_legged_msgs, msg, LED)();
  if (!ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_type_support_handle.typesupport_identifier) {
    ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_unitree_legged_msgs__msg__HighCmd__rosidl_typesupport_introspection_c__HighCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
