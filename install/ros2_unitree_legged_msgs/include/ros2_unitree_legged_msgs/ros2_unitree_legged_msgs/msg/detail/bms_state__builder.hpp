// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_STATE__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/bms_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_BmsState_cell_vol
{
public:
  explicit Init_BmsState_cell_vol(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::BmsState cell_vol(::ros2_unitree_legged_msgs::msg::BmsState::_cell_vol_type arg)
  {
    msg_.cell_vol = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_mcu_ntc
{
public:
  explicit Init_BmsState_mcu_ntc(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_cell_vol mcu_ntc(::ros2_unitree_legged_msgs::msg::BmsState::_mcu_ntc_type arg)
  {
    msg_.mcu_ntc = std::move(arg);
    return Init_BmsState_cell_vol(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_bq_ntc
{
public:
  explicit Init_BmsState_bq_ntc(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_mcu_ntc bq_ntc(::ros2_unitree_legged_msgs::msg::BmsState::_bq_ntc_type arg)
  {
    msg_.bq_ntc = std::move(arg);
    return Init_BmsState_mcu_ntc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_cycle
{
public:
  explicit Init_BmsState_cycle(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_bq_ntc cycle(::ros2_unitree_legged_msgs::msg::BmsState::_cycle_type arg)
  {
    msg_.cycle = std::move(arg);
    return Init_BmsState_bq_ntc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_current
{
public:
  explicit Init_BmsState_current(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_cycle current(::ros2_unitree_legged_msgs::msg::BmsState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BmsState_cycle(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_soc
{
public:
  explicit Init_BmsState_soc(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_current soc(::ros2_unitree_legged_msgs::msg::BmsState::_soc_type arg)
  {
    msg_.soc = std::move(arg);
    return Init_BmsState_current(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_bms_status
{
public:
  explicit Init_BmsState_bms_status(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_soc bms_status(::ros2_unitree_legged_msgs::msg::BmsState::_bms_status_type arg)
  {
    msg_.bms_status = std::move(arg);
    return Init_BmsState_soc(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_version_l
{
public:
  explicit Init_BmsState_version_l(::ros2_unitree_legged_msgs::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_bms_status version_l(::ros2_unitree_legged_msgs::msg::BmsState::_version_l_type arg)
  {
    msg_.version_l = std::move(arg);
    return Init_BmsState_bms_status(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

class Init_BmsState_version_h
{
public:
  Init_BmsState_version_h()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BmsState_version_l version_h(::ros2_unitree_legged_msgs::msg::BmsState::_version_h_type arg)
  {
    msg_.version_h = std::move(arg);
    return Init_BmsState_version_l(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::BmsState>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_BmsState_version_h();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_STATE__BUILDER_HPP_
