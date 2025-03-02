// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/HighState.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_STATE__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/high_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_HighState_crc
{
public:
  explicit Init_HighState_crc(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::HighState crc(::ros2_unitree_legged_msgs::msg::HighState::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_reserve
{
public:
  explicit Init_HighState_reserve(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_crc reserve(::ros2_unitree_legged_msgs::msg::HighState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_HighState_crc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_wireless_remote
{
public:
  explicit Init_HighState_wireless_remote(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_reserve wireless_remote(::ros2_unitree_legged_msgs::msg::HighState::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_HighState_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_foot_speed2body
{
public:
  explicit Init_HighState_foot_speed2body(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_wireless_remote foot_speed2body(::ros2_unitree_legged_msgs::msg::HighState::_foot_speed2body_type arg)
  {
    msg_.foot_speed2body = std::move(arg);
    return Init_HighState_wireless_remote(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_foot_position2body
{
public:
  explicit Init_HighState_foot_position2body(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_foot_speed2body foot_position2body(::ros2_unitree_legged_msgs::msg::HighState::_foot_position2body_type arg)
  {
    msg_.foot_position2body = std::move(arg);
    return Init_HighState_foot_speed2body(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_range_obstacle
{
public:
  explicit Init_HighState_range_obstacle(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_foot_position2body range_obstacle(::ros2_unitree_legged_msgs::msg::HighState::_range_obstacle_type arg)
  {
    msg_.range_obstacle = std::move(arg);
    return Init_HighState_foot_position2body(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_yaw_speed
{
public:
  explicit Init_HighState_yaw_speed(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_range_obstacle yaw_speed(::ros2_unitree_legged_msgs::msg::HighState::_yaw_speed_type arg)
  {
    msg_.yaw_speed = std::move(arg);
    return Init_HighState_range_obstacle(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_velocity
{
public:
  explicit Init_HighState_velocity(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_yaw_speed velocity(::ros2_unitree_legged_msgs::msg::HighState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_HighState_yaw_speed(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_body_height
{
public:
  explicit Init_HighState_body_height(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_velocity body_height(::ros2_unitree_legged_msgs::msg::HighState::_body_height_type arg)
  {
    msg_.body_height = std::move(arg);
    return Init_HighState_velocity(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_position
{
public:
  explicit Init_HighState_position(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_body_height position(::ros2_unitree_legged_msgs::msg::HighState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_HighState_body_height(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_foot_raise_height
{
public:
  explicit Init_HighState_foot_raise_height(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_position foot_raise_height(::ros2_unitree_legged_msgs::msg::HighState::_foot_raise_height_type arg)
  {
    msg_.foot_raise_height = std::move(arg);
    return Init_HighState_position(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_gait_type
{
public:
  explicit Init_HighState_gait_type(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_foot_raise_height gait_type(::ros2_unitree_legged_msgs::msg::HighState::_gait_type_type arg)
  {
    msg_.gait_type = std::move(arg);
    return Init_HighState_foot_raise_height(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_progress
{
public:
  explicit Init_HighState_progress(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_gait_type progress(::ros2_unitree_legged_msgs::msg::HighState::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_HighState_gait_type(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_mode
{
public:
  explicit Init_HighState_mode(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_progress mode(::ros2_unitree_legged_msgs::msg::HighState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_HighState_progress(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_foot_force_est
{
public:
  explicit Init_HighState_foot_force_est(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_mode foot_force_est(::ros2_unitree_legged_msgs::msg::HighState::_foot_force_est_type arg)
  {
    msg_.foot_force_est = std::move(arg);
    return Init_HighState_mode(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_foot_force
{
public:
  explicit Init_HighState_foot_force(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_foot_force_est foot_force(::ros2_unitree_legged_msgs::msg::HighState::_foot_force_type arg)
  {
    msg_.foot_force = std::move(arg);
    return Init_HighState_foot_force_est(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_bms
{
public:
  explicit Init_HighState_bms(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_foot_force bms(::ros2_unitree_legged_msgs::msg::HighState::_bms_type arg)
  {
    msg_.bms = std::move(arg);
    return Init_HighState_foot_force(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_motor_state
{
public:
  explicit Init_HighState_motor_state(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_bms motor_state(::ros2_unitree_legged_msgs::msg::HighState::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return Init_HighState_bms(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_imu
{
public:
  explicit Init_HighState_imu(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_motor_state imu(::ros2_unitree_legged_msgs::msg::HighState::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_HighState_motor_state(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_band_width
{
public:
  explicit Init_HighState_band_width(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_imu band_width(::ros2_unitree_legged_msgs::msg::HighState::_band_width_type arg)
  {
    msg_.band_width = std::move(arg);
    return Init_HighState_imu(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_version
{
public:
  explicit Init_HighState_version(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_band_width version(::ros2_unitree_legged_msgs::msg::HighState::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_HighState_band_width(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_sn
{
public:
  explicit Init_HighState_sn(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_version sn(::ros2_unitree_legged_msgs::msg::HighState::_sn_type arg)
  {
    msg_.sn = std::move(arg);
    return Init_HighState_version(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_frame_reserve
{
public:
  explicit Init_HighState_frame_reserve(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_sn frame_reserve(::ros2_unitree_legged_msgs::msg::HighState::_frame_reserve_type arg)
  {
    msg_.frame_reserve = std::move(arg);
    return Init_HighState_sn(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_level_flag
{
public:
  explicit Init_HighState_level_flag(::ros2_unitree_legged_msgs::msg::HighState & msg)
  : msg_(msg)
  {}
  Init_HighState_frame_reserve level_flag(::ros2_unitree_legged_msgs::msg::HighState::_level_flag_type arg)
  {
    msg_.level_flag = std::move(arg);
    return Init_HighState_frame_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

class Init_HighState_head
{
public:
  Init_HighState_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HighState_level_flag head(::ros2_unitree_legged_msgs::msg::HighState::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_HighState_level_flag(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::HighState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::HighState>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_HighState_head();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_STATE__BUILDER_HPP_
