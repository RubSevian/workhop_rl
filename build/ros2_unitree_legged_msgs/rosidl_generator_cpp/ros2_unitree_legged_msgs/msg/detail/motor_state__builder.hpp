// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorState_reserve
{
public:
  explicit Init_MotorState_reserve(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::MotorState reserve(::ros2_unitree_legged_msgs::msg::MotorState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_temperature
{
public:
  explicit Init_MotorState_temperature(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_reserve temperature(::ros2_unitree_legged_msgs::msg::MotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_MotorState_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_ddq_raw
{
public:
  explicit Init_MotorState_ddq_raw(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_temperature ddq_raw(::ros2_unitree_legged_msgs::msg::MotorState::_ddq_raw_type arg)
  {
    msg_.ddq_raw = std::move(arg);
    return Init_MotorState_temperature(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_dq_raw
{
public:
  explicit Init_MotorState_dq_raw(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_ddq_raw dq_raw(::ros2_unitree_legged_msgs::msg::MotorState::_dq_raw_type arg)
  {
    msg_.dq_raw = std::move(arg);
    return Init_MotorState_ddq_raw(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_q_raw
{
public:
  explicit Init_MotorState_q_raw(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_dq_raw q_raw(::ros2_unitree_legged_msgs::msg::MotorState::_q_raw_type arg)
  {
    msg_.q_raw = std::move(arg);
    return Init_MotorState_dq_raw(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_tau_est
{
public:
  explicit Init_MotorState_tau_est(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_q_raw tau_est(::ros2_unitree_legged_msgs::msg::MotorState::_tau_est_type arg)
  {
    msg_.tau_est = std::move(arg);
    return Init_MotorState_q_raw(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_ddq
{
public:
  explicit Init_MotorState_ddq(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_tau_est ddq(::ros2_unitree_legged_msgs::msg::MotorState::_ddq_type arg)
  {
    msg_.ddq = std::move(arg);
    return Init_MotorState_tau_est(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_dq
{
public:
  explicit Init_MotorState_dq(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_ddq dq(::ros2_unitree_legged_msgs::msg::MotorState::_dq_type arg)
  {
    msg_.dq = std::move(arg);
    return Init_MotorState_ddq(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_q
{
public:
  explicit Init_MotorState_q(::ros2_unitree_legged_msgs::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_dq q(::ros2_unitree_legged_msgs::msg::MotorState::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorState_dq(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

class Init_MotorState_mode
{
public:
  Init_MotorState_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_q mode(::ros2_unitree_legged_msgs::msg::MotorState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorState_q(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::MotorState>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_MotorState_mode();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
