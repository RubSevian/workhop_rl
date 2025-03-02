// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/low_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_LowState_crc
{
public:
  explicit Init_LowState_crc(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::LowState crc(::ros2_unitree_legged_msgs::msg::LowState::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_reserve
{
public:
  explicit Init_LowState_reserve(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_crc reserve(::ros2_unitree_legged_msgs::msg::LowState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_LowState_crc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_wireless_remote
{
public:
  explicit Init_LowState_wireless_remote(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_reserve wireless_remote(::ros2_unitree_legged_msgs::msg::LowState::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_LowState_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_tick
{
public:
  explicit Init_LowState_tick(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_wireless_remote tick(::ros2_unitree_legged_msgs::msg::LowState::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_LowState_wireless_remote(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_foot_force_est
{
public:
  explicit Init_LowState_foot_force_est(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_tick foot_force_est(::ros2_unitree_legged_msgs::msg::LowState::_foot_force_est_type arg)
  {
    msg_.foot_force_est = std::move(arg);
    return Init_LowState_tick(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_foot_force
{
public:
  explicit Init_LowState_foot_force(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_foot_force_est foot_force(::ros2_unitree_legged_msgs::msg::LowState::_foot_force_type arg)
  {
    msg_.foot_force = std::move(arg);
    return Init_LowState_foot_force_est(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_bms
{
public:
  explicit Init_LowState_bms(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_foot_force bms(::ros2_unitree_legged_msgs::msg::LowState::_bms_type arg)
  {
    msg_.bms = std::move(arg);
    return Init_LowState_foot_force(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_motor_state
{
public:
  explicit Init_LowState_motor_state(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_bms motor_state(::ros2_unitree_legged_msgs::msg::LowState::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return Init_LowState_bms(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_imu
{
public:
  explicit Init_LowState_imu(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_motor_state imu(::ros2_unitree_legged_msgs::msg::LowState::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_LowState_motor_state(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_band_width
{
public:
  explicit Init_LowState_band_width(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_imu band_width(::ros2_unitree_legged_msgs::msg::LowState::_band_width_type arg)
  {
    msg_.band_width = std::move(arg);
    return Init_LowState_imu(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_version
{
public:
  explicit Init_LowState_version(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_band_width version(::ros2_unitree_legged_msgs::msg::LowState::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_LowState_band_width(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_sn
{
public:
  explicit Init_LowState_sn(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_version sn(::ros2_unitree_legged_msgs::msg::LowState::_sn_type arg)
  {
    msg_.sn = std::move(arg);
    return Init_LowState_version(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_frame_reserve
{
public:
  explicit Init_LowState_frame_reserve(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_sn frame_reserve(::ros2_unitree_legged_msgs::msg::LowState::_frame_reserve_type arg)
  {
    msg_.frame_reserve = std::move(arg);
    return Init_LowState_sn(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_level_flag
{
public:
  explicit Init_LowState_level_flag(::ros2_unitree_legged_msgs::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_frame_reserve level_flag(::ros2_unitree_legged_msgs::msg::LowState::_level_flag_type arg)
  {
    msg_.level_flag = std::move(arg);
    return Init_LowState_frame_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

class Init_LowState_head
{
public:
  Init_LowState_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowState_level_flag head(::ros2_unitree_legged_msgs::msg::LowState::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_LowState_level_flag(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::LowState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::LowState>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_LowState_head();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
