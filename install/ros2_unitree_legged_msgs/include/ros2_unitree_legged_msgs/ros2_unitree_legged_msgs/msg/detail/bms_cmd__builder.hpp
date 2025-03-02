// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_BmsCmd_reserve
{
public:
  explicit Init_BmsCmd_reserve(::ros2_unitree_legged_msgs::msg::BmsCmd & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::BmsCmd reserve(::ros2_unitree_legged_msgs::msg::BmsCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsCmd msg_;
};

class Init_BmsCmd_off
{
public:
  Init_BmsCmd_off()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BmsCmd_reserve off(::ros2_unitree_legged_msgs::msg::BmsCmd::_off_type arg)
  {
    msg_.off = std::move(arg);
    return Init_BmsCmd_reserve(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::BmsCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::BmsCmd>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_BmsCmd_off();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__BUILDER_HPP_
