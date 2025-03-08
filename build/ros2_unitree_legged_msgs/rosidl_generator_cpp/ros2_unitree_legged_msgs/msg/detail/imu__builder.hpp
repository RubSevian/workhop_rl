// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_unitree_legged_msgs:msg/IMU.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__IMU__BUILDER_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__IMU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_unitree_legged_msgs/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_IMU_temperature
{
public:
  explicit Init_IMU_temperature(::ros2_unitree_legged_msgs::msg::IMU & msg)
  : msg_(msg)
  {}
  ::ros2_unitree_legged_msgs::msg::IMU temperature(::ros2_unitree_legged_msgs::msg::IMU::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::IMU msg_;
};

class Init_IMU_rpy
{
public:
  explicit Init_IMU_rpy(::ros2_unitree_legged_msgs::msg::IMU & msg)
  : msg_(msg)
  {}
  Init_IMU_temperature rpy(::ros2_unitree_legged_msgs::msg::IMU::_rpy_type arg)
  {
    msg_.rpy = std::move(arg);
    return Init_IMU_temperature(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::IMU msg_;
};

class Init_IMU_accelerometer
{
public:
  explicit Init_IMU_accelerometer(::ros2_unitree_legged_msgs::msg::IMU & msg)
  : msg_(msg)
  {}
  Init_IMU_rpy accelerometer(::ros2_unitree_legged_msgs::msg::IMU::_accelerometer_type arg)
  {
    msg_.accelerometer = std::move(arg);
    return Init_IMU_rpy(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::IMU msg_;
};

class Init_IMU_gyroscope
{
public:
  explicit Init_IMU_gyroscope(::ros2_unitree_legged_msgs::msg::IMU & msg)
  : msg_(msg)
  {}
  Init_IMU_accelerometer gyroscope(::ros2_unitree_legged_msgs::msg::IMU::_gyroscope_type arg)
  {
    msg_.gyroscope = std::move(arg);
    return Init_IMU_accelerometer(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::IMU msg_;
};

class Init_IMU_quaternion
{
public:
  Init_IMU_quaternion()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMU_gyroscope quaternion(::ros2_unitree_legged_msgs::msg::IMU::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_IMU_gyroscope(msg_);
  }

private:
  ::ros2_unitree_legged_msgs::msg::IMU msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_unitree_legged_msgs::msg::IMU>()
{
  return ros2_unitree_legged_msgs::msg::builder::Init_IMU_quaternion();
}

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__IMU__BUILDER_HPP_
