// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/HighCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_HighCmd_crc
{
public:
  explicit Init_HighCmd_crc(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::HighCmd crc(::ros2_unitree_legged_msgs::msg::HighCmd::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_reserve
{
public:
  explicit Init_HighCmd_reserve(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_crc reserve(::ros2_unitree_legged_msgs::msg::HighCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_HighCmd_crc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_wireless_remote
{
public:
  explicit Init_HighCmd_wireless_remote(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_reserve wireless_remote(::ros2_unitree_legged_msgs::msg::HighCmd::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_HighCmd_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_led
{
public:
  explicit Init_HighCmd_led(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_wireless_remote led(::ros2_unitree_legged_msgs::msg::HighCmd::_led_type arg)
  {
    msg_.led = std::move(arg);
    return Init_HighCmd_wireless_remote(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_bms
{
public:
  explicit Init_HighCmd_bms(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_led bms(::ros2_unitree_legged_msgs::msg::HighCmd::_bms_type arg)
  {
    msg_.bms = std::move(arg);
    return Init_HighCmd_led(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_yaw_speed
{
public:
  explicit Init_HighCmd_yaw_speed(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_bms yaw_speed(::ros2_unitree_legged_msgs::msg::HighCmd::_yaw_speed_type arg)
  {
    msg_.yaw_speed = std::move(arg);
    return Init_HighCmd_bms(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_velocity
{
public:
  explicit Init_HighCmd_velocity(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_yaw_speed velocity(::ros2_unitree_legged_msgs::msg::HighCmd::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_HighCmd_yaw_speed(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_euler
{
public:
  explicit Init_HighCmd_euler(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_velocity euler(::ros2_unitree_legged_msgs::msg::HighCmd::_euler_type arg)
  {
    msg_.euler = std::move(arg);
    return Init_HighCmd_velocity(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_position
{
public:
  explicit Init_HighCmd_position(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_euler position(::ros2_unitree_legged_msgs::msg::HighCmd::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_HighCmd_euler(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_body_height
{
public:
  explicit Init_HighCmd_body_height(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_position body_height(::ros2_unitree_legged_msgs::msg::HighCmd::_body_height_type arg)
  {
    msg_.body_height = std::move(arg);
    return Init_HighCmd_position(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_foot_raise_height
{
public:
  explicit Init_HighCmd_foot_raise_height(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_body_height foot_raise_height(::ros2_unitree_legged_msgs::msg::HighCmd::_foot_raise_height_type arg)
  {
    msg_.foot_raise_height = std::move(arg);
    return Init_HighCmd_body_height(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_speed_level
{
public:
  explicit Init_HighCmd_speed_level(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_foot_raise_height speed_level(::ros2_unitree_legged_msgs::msg::HighCmd::_speed_level_type arg)
  {
    msg_.speed_level = std::move(arg);
    return Init_HighCmd_foot_raise_height(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_gait_type
{
public:
  explicit Init_HighCmd_gait_type(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_speed_level gait_type(::ros2_unitree_legged_msgs::msg::HighCmd::_gait_type_type arg)
  {
    msg_.gait_type = std::move(arg);
    return Init_HighCmd_speed_level(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_mode
{
public:
  explicit Init_HighCmd_mode(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_gait_type mode(::ros2_unitree_legged_msgs::msg::HighCmd::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_HighCmd_gait_type(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_band_width
{
public:
  explicit Init_HighCmd_band_width(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_mode band_width(::ros2_unitree_legged_msgs::msg::HighCmd::_band_width_type arg)
  {
    msg_.band_width = std::move(arg);
    return Init_HighCmd_mode(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_version
{
public:
  explicit Init_HighCmd_version(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_band_width version(::ros2_unitree_legged_msgs::msg::HighCmd::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_HighCmd_band_width(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_sn
{
public:
  explicit Init_HighCmd_sn(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_version sn(::ros2_unitree_legged_msgs::msg::HighCmd::_sn_type arg)
  {
    msg_.sn = std::move(arg);
    return Init_HighCmd_version(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_frame_reserve
{
public:
  explicit Init_HighCmd_frame_reserve(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_sn frame_reserve(::ros2_unitree_legged_msgs::msg::HighCmd::_frame_reserve_type arg)
  {
    msg_.frame_reserve = std::move(arg);
    return Init_HighCmd_sn(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_level_flag
{
public:
  explicit Init_HighCmd_level_flag(::ros2_unitree_legged_msgs::msg::HighCmd & msg)
  : msg_(msg)
  {}
  Init_HighCmd_frame_reserve level_flag(::ros2_unitree_legged_msgs::msg::HighCmd::_level_flag_type arg)
  {
    msg_.level_flag = std::move(arg);
    return Init_HighCmd_frame_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

class Init_HighCmd_head
{
public:
  Init_HighCmd_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HighCmd_level_flag head(::ros2_unitree_legged_msgs::msg::HighCmd::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_HighCmd_level_flag(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::HighCmd>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_HighCmd_head();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__BUILDER_HPP_
