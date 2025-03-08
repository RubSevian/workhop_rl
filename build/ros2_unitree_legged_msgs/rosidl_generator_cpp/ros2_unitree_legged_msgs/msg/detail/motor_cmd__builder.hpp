// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/MotorCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_CMD__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/motor_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCmd_reserve
{
public:
  explicit Init_MotorCmd_reserve(::ros2_unitree_legged_msgs::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::MotorCmd reserve(::ros2_unitree_legged_msgs::msg::MotorCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

class Init_MotorCmd_kd
{
public:
  explicit Init_MotorCmd_kd(::ros2_unitree_legged_msgs::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_reserve kd(::ros2_unitree_legged_msgs::msg::MotorCmd::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MotorCmd_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

class Init_MotorCmd_kp
{
public:
  explicit Init_MotorCmd_kp(::ros2_unitree_legged_msgs::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_kd kp(::ros2_unitree_legged_msgs::msg::MotorCmd::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MotorCmd_kd(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

class Init_MotorCmd_tau
{
public:
  explicit Init_MotorCmd_tau(::ros2_unitree_legged_msgs::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_kp tau(::ros2_unitree_legged_msgs::msg::MotorCmd::_tau_type arg)
  {
    msg_.tau = std::move(arg);
    return Init_MotorCmd_kp(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

class Init_MotorCmd_dq
{
public:
  explicit Init_MotorCmd_dq(::ros2_unitree_legged_msgs::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_tau dq(::ros2_unitree_legged_msgs::msg::MotorCmd::_dq_type arg)
  {
    msg_.dq = std::move(arg);
    return Init_MotorCmd_tau(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

class Init_MotorCmd_q
{
public:
  explicit Init_MotorCmd_q(::ros2_unitree_legged_msgs::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_dq q(::ros2_unitree_legged_msgs::msg::MotorCmd::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorCmd_dq(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

class Init_MotorCmd_mode
{
public:
  Init_MotorCmd_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCmd_q mode(::ros2_unitree_legged_msgs::msg::MotorCmd::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorCmd_q(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::MotorCmd>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_MotorCmd_mode();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_CMD__BUILDER_HPP_
