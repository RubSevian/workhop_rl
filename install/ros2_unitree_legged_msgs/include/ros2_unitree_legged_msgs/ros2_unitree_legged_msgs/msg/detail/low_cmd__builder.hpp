// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/low_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_LowCmd_crc
{
public:
  explicit Init_LowCmd_crc(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::LowCmd crc(::ros2_unitree_legged_msgs::msg::LowCmd::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_reserve
{
public:
  explicit Init_LowCmd_reserve(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_crc reserve(::ros2_unitree_legged_msgs::msg::LowCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_LowCmd_crc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_wireless_remote
{
public:
  explicit Init_LowCmd_wireless_remote(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_reserve wireless_remote(::ros2_unitree_legged_msgs::msg::LowCmd::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_LowCmd_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_bms
{
public:
  explicit Init_LowCmd_bms(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_wireless_remote bms(::ros2_unitree_legged_msgs::msg::LowCmd::_bms_type arg)
  {
    msg_.bms = std::move(arg);
    return Init_LowCmd_wireless_remote(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_motor_cmd
{
public:
  explicit Init_LowCmd_motor_cmd(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_bms motor_cmd(::ros2_unitree_legged_msgs::msg::LowCmd::_motor_cmd_type arg)
  {
    msg_.motor_cmd = std::move(arg);
    return Init_LowCmd_bms(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_band_width
{
public:
  explicit Init_LowCmd_band_width(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_motor_cmd band_width(::ros2_unitree_legged_msgs::msg::LowCmd::_band_width_type arg)
  {
    msg_.band_width = std::move(arg);
    return Init_LowCmd_motor_cmd(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_version
{
public:
  explicit Init_LowCmd_version(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_band_width version(::ros2_unitree_legged_msgs::msg::LowCmd::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_LowCmd_band_width(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_sn
{
public:
  explicit Init_LowCmd_sn(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_version sn(::ros2_unitree_legged_msgs::msg::LowCmd::_sn_type arg)
  {
    msg_.sn = std::move(arg);
    return Init_LowCmd_version(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_frame_reserve
{
public:
  explicit Init_LowCmd_frame_reserve(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_sn frame_reserve(::ros2_unitree_legged_msgs::msg::LowCmd::_frame_reserve_type arg)
  {
    msg_.frame_reserve = std::move(arg);
    return Init_LowCmd_sn(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_level_flag
{
public:
  explicit Init_LowCmd_level_flag(::ros2_unitree_legged_msgs::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_frame_reserve level_flag(::ros2_unitree_legged_msgs::msg::LowCmd::_level_flag_type arg)
  {
    msg_.level_flag = std::move(arg);
    return Init_LowCmd_frame_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

class Init_LowCmd_head
{
public:
  Init_LowCmd_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowCmd_level_flag head(::ros2_unitree_legged_msgs::msg::LowCmd::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_LowCmd_level_flag(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::LowCmd>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_LowCmd_head();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__BUILDER_HPP_
