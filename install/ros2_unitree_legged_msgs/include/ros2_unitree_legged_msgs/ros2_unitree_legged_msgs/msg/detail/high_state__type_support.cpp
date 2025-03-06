// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros2_unitree_legged_msgs:msg/HighState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2_unitree_legged_msgs/msg/detail/high_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void HighState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_unitree_legged_msgs::msg::HighState(_init);
}

void HighState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_unitree_legged_msgs::msg::HighState *>(message_memory);
  typed_message->~HighState();
}

size_t size_function__HighState__head(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__HighState__head(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__head(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__head(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__HighState__head(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__HighState__head(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__HighState__head(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

size_t size_function__HighState__sn(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__HighState__sn(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__sn(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__sn(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__HighState__sn(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__HighState__sn(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__HighState__sn(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

size_t size_function__HighState__version(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__HighState__version(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__version(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__version(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__HighState__version(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__HighState__version(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__HighState__version(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

size_t size_function__HighState__motor_state(const void * untyped_member)
{
  (void)untyped_member;
  return 20;
}

const void * get_const_function__HighState__motor_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<ros2_unitree_legged_msgs::msg::MotorState, 20> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__motor_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<ros2_unitree_legged_msgs::msg::MotorState, 20> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__motor_state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros2_unitree_legged_msgs::msg::MotorState *>(
    get_const_function__HighState__motor_state(untyped_member, index));
  auto & value = *reinterpret_cast<ros2_unitree_legged_msgs::msg::MotorState *>(untyped_value);
  value = item;
}

void assign_function__HighState__motor_state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros2_unitree_legged_msgs::msg::MotorState *>(
    get_function__HighState__motor_state(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros2_unitree_legged_msgs::msg::MotorState *>(untyped_value);
  item = value;
}

size_t size_function__HighState__foot_force(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__HighState__foot_force(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__foot_force(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__foot_force(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__HighState__foot_force(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__HighState__foot_force(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__HighState__foot_force(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

size_t size_function__HighState__foot_force_est(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__HighState__foot_force_est(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__foot_force_est(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__foot_force_est(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__HighState__foot_force_est(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__HighState__foot_force_est(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__HighState__foot_force_est(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

size_t size_function__HighState__position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__HighState__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__HighState__position(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__HighState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__HighState__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__HighState__velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__HighState__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__HighState__velocity(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__HighState__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__HighState__velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__HighState__range_obstacle(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__HighState__range_obstacle(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__range_obstacle(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__range_obstacle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__HighState__range_obstacle(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__HighState__range_obstacle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__HighState__range_obstacle(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__HighState__foot_position2body(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__HighState__foot_position2body(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<ros2_unitree_legged_msgs::msg::Cartesian, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__foot_position2body(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<ros2_unitree_legged_msgs::msg::Cartesian, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__foot_position2body(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros2_unitree_legged_msgs::msg::Cartesian *>(
    get_const_function__HighState__foot_position2body(untyped_member, index));
  auto & value = *reinterpret_cast<ros2_unitree_legged_msgs::msg::Cartesian *>(untyped_value);
  value = item;
}

void assign_function__HighState__foot_position2body(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros2_unitree_legged_msgs::msg::Cartesian *>(
    get_function__HighState__foot_position2body(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros2_unitree_legged_msgs::msg::Cartesian *>(untyped_value);
  item = value;
}

size_t size_function__HighState__foot_speed2body(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__HighState__foot_speed2body(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<ros2_unitree_legged_msgs::msg::Cartesian, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__foot_speed2body(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<ros2_unitree_legged_msgs::msg::Cartesian, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__foot_speed2body(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ros2_unitree_legged_msgs::msg::Cartesian *>(
    get_const_function__HighState__foot_speed2body(untyped_member, index));
  auto & value = *reinterpret_cast<ros2_unitree_legged_msgs::msg::Cartesian *>(untyped_value);
  value = item;
}

void assign_function__HighState__foot_speed2body(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ros2_unitree_legged_msgs::msg::Cartesian *>(
    get_function__HighState__foot_speed2body(untyped_member, index));
  const auto & value = *reinterpret_cast<const ros2_unitree_legged_msgs::msg::Cartesian *>(untyped_value);
  item = value;
}

size_t size_function__HighState__wireless_remote(const void * untyped_member)
{
  (void)untyped_member;
  return 40;
}

const void * get_const_function__HighState__wireless_remote(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 40> *>(untyped_member);
  return &member[index];
}

void * get_function__HighState__wireless_remote(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 40> *>(untyped_member);
  return &member[index];
}

void fetch_function__HighState__wireless_remote(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__HighState__wireless_remote(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__HighState__wireless_remote(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__HighState__wireless_remote(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HighState_message_member_array[25] = {
  {
    "head",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, head),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__head,  // size() function pointer
    get_const_function__HighState__head,  // get_const(index) function pointer
    get_function__HighState__head,  // get(index) function pointer
    fetch_function__HighState__head,  // fetch(index, &value) function pointer
    assign_function__HighState__head,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "level_flag",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, level_flag),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frame_reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, frame_reserve),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sn",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, sn),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__sn,  // size() function pointer
    get_const_function__HighState__sn,  // get_const(index) function pointer
    get_function__HighState__sn,  // get(index) function pointer
    fetch_function__HighState__sn,  // fetch(index, &value) function pointer
    assign_function__HighState__sn,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "version",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, version),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__version,  // size() function pointer
    get_const_function__HighState__version,  // get_const(index) function pointer
    get_function__HighState__version,  // get(index) function pointer
    fetch_function__HighState__version,  // fetch(index, &value) function pointer
    assign_function__HighState__version,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "band_width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, band_width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "imu",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_unitree_legged_msgs::msg::IMU>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, imu),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_unitree_legged_msgs::msg::MotorState>(),  // members of sub message
    true,  // is array
    20,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, motor_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__motor_state,  // size() function pointer
    get_const_function__HighState__motor_state,  // get_const(index) function pointer
    get_function__HighState__motor_state,  // get(index) function pointer
    fetch_function__HighState__motor_state,  // fetch(index, &value) function pointer
    assign_function__HighState__motor_state,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bms",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_unitree_legged_msgs::msg::BmsState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, bms),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "foot_force",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, foot_force),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__foot_force,  // size() function pointer
    get_const_function__HighState__foot_force,  // get_const(index) function pointer
    get_function__HighState__foot_force,  // get(index) function pointer
    fetch_function__HighState__foot_force,  // fetch(index, &value) function pointer
    assign_function__HighState__foot_force,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "foot_force_est",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, foot_force_est),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__foot_force_est,  // size() function pointer
    get_const_function__HighState__foot_force_est,  // get_const(index) function pointer
    get_function__HighState__foot_force_est,  // get(index) function pointer
    fetch_function__HighState__foot_force_est,  // fetch(index, &value) function pointer
    assign_function__HighState__foot_force_est,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "progress",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, progress),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gait_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, gait_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "foot_raise_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, foot_raise_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__position,  // size() function pointer
    get_const_function__HighState__position,  // get_const(index) function pointer
    get_function__HighState__position,  // get(index) function pointer
    fetch_function__HighState__position,  // fetch(index, &value) function pointer
    assign_function__HighState__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "body_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, body_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__velocity,  // size() function pointer
    get_const_function__HighState__velocity,  // get_const(index) function pointer
    get_function__HighState__velocity,  // get(index) function pointer
    fetch_function__HighState__velocity,  // fetch(index, &value) function pointer
    assign_function__HighState__velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, yaw_speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "range_obstacle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, range_obstacle),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__range_obstacle,  // size() function pointer
    get_const_function__HighState__range_obstacle,  // get_const(index) function pointer
    get_function__HighState__range_obstacle,  // get(index) function pointer
    fetch_function__HighState__range_obstacle,  // fetch(index, &value) function pointer
    assign_function__HighState__range_obstacle,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "foot_position2body",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_unitree_legged_msgs::msg::Cartesian>(),  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, foot_position2body),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__foot_position2body,  // size() function pointer
    get_const_function__HighState__foot_position2body,  // get_const(index) function pointer
    get_function__HighState__foot_position2body,  // get(index) function pointer
    fetch_function__HighState__foot_position2body,  // fetch(index, &value) function pointer
    assign_function__HighState__foot_position2body,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "foot_speed2body",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ros2_unitree_legged_msgs::msg::Cartesian>(),  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, foot_speed2body),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__foot_speed2body,  // size() function pointer
    get_const_function__HighState__foot_speed2body,  // get_const(index) function pointer
    get_function__HighState__foot_speed2body,  // get(index) function pointer
    fetch_function__HighState__foot_speed2body,  // fetch(index, &value) function pointer
    assign_function__HighState__foot_speed2body,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wireless_remote",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    40,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, wireless_remote),  // bytes offset in struct
    nullptr,  // default value
    size_function__HighState__wireless_remote,  // size() function pointer
    get_const_function__HighState__wireless_remote,  // get_const(index) function pointer
    get_function__HighState__wireless_remote,  // get(index) function pointer
    fetch_function__HighState__wireless_remote,  // fetch(index, &value) function pointer
    assign_function__HighState__wireless_remote,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, reserve),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "crc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_unitree_legged_msgs::msg::HighState, crc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HighState_message_members = {
  "ros2_unitree_legged_msgs::msg",  // message namespace
  "HighState",  // message name
  25,  // number of fields
  sizeof(ros2_unitree_legged_msgs::msg::HighState),
  HighState_message_member_array,  // message members
  HighState_init_function,  // function to initialize message memory (memory has to be allocated)
  HighState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HighState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HighState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros2_unitree_legged_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_unitree_legged_msgs::msg::HighState>()
{
  return &::ros2_unitree_legged_msgs::msg::rosidl_typesupport_introspection_cpp::HighState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_unitree_legged_msgs, msg, HighState)() {
  return &::ros2_unitree_legged_msgs::msg::rosidl_typesupport_introspection_cpp::HighState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
