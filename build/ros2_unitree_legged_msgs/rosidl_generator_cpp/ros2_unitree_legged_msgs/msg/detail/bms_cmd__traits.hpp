// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_unitree_legged_msgs:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__TRAITS_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_unitree_legged_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BmsCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: off
  {
    out << "off: ";
    rosidl_generator_traits::value_to_yaml(msg.off, out);
    out << ", ";
  }

  // member: reserve
  {
    if (msg.reserve.size() == 0) {
      out << "reserve: []";
    } else {
      out << "reserve: [";
      size_t pending_items = msg.reserve.size();
      for (auto item : msg.reserve) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BmsCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: off
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "off: ";
    rosidl_generator_traits::value_to_yaml(msg.off, out);
    out << "\n";
  }

  // member: reserve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.reserve.size() == 0) {
      out << "reserve: []\n";
    } else {
      out << "reserve:\n";
      for (auto item : msg.reserve) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BmsCmd & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros2_unitree_legged_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ros2_unitree_legged_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_unitree_legged_msgs::msg::BmsCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_unitree_legged_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_unitree_legged_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_unitree_legged_msgs::msg::BmsCmd & msg)
{
  return ros2_unitree_legged_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_unitree_legged_msgs::msg::BmsCmd>()
{
  return "ros2_unitree_legged_msgs::msg::BmsCmd";
}

template<>
inline const char * name<ros2_unitree_legged_msgs::msg::BmsCmd>()
{
  return "ros2_unitree_legged_msgs/msg/BmsCmd";
}

template<>
struct has_fixed_size<ros2_unitree_legged_msgs::msg::BmsCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_unitree_legged_msgs::msg::BmsCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_unitree_legged_msgs::msg::BmsCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__BMS_CMD__TRAITS_HPP_
