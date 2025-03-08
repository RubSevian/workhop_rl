// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_unitree_legged_msgs:msg/IMU.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__IMU__TRAITS_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__IMU__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_unitree_legged_msgs/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_unitree_legged_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const IMU & msg,
  std::ostream & out)
{
  out << "{";
  // member: quaternion
  {
    if (msg.quaternion.size() == 0) {
      out << "quaternion: []";
    } else {
      out << "quaternion: [";
      size_t pending_items = msg.quaternion.size();
      for (auto item : msg.quaternion) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gyroscope
  {
    if (msg.gyroscope.size() == 0) {
      out << "gyroscope: []";
    } else {
      out << "gyroscope: [";
      size_t pending_items = msg.gyroscope.size();
      for (auto item : msg.gyroscope) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: accelerometer
  {
    if (msg.accelerometer.size() == 0) {
      out << "accelerometer: []";
    } else {
      out << "accelerometer: [";
      size_t pending_items = msg.accelerometer.size();
      for (auto item : msg.accelerometer) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: rpy
  {
    if (msg.rpy.size() == 0) {
      out << "rpy: []";
    } else {
      out << "rpy: [";
      size_t pending_items = msg.rpy.size();
      for (auto item : msg.rpy) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IMU & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: quaternion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.quaternion.size() == 0) {
      out << "quaternion: []\n";
    } else {
      out << "quaternion:\n";
      for (auto item : msg.quaternion) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gyroscope
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gyroscope.size() == 0) {
      out << "gyroscope: []\n";
    } else {
      out << "gyroscope:\n";
      for (auto item : msg.gyroscope) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: accelerometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.accelerometer.size() == 0) {
      out << "accelerometer: []\n";
    } else {
      out << "accelerometer:\n";
      for (auto item : msg.accelerometer) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: rpy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rpy.size() == 0) {
      out << "rpy: []\n";
    } else {
      out << "rpy:\n";
      for (auto item : msg.rpy) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IMU & msg, bool use_flow_style = false)
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
  const ros2_unitree_legged_msgs::msg::IMU & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_unitree_legged_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_unitree_legged_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_unitree_legged_msgs::msg::IMU & msg)
{
  return ros2_unitree_legged_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_unitree_legged_msgs::msg::IMU>()
{
  return "ros2_unitree_legged_msgs::msg::IMU";
}

template<>
inline const char * name<ros2_unitree_legged_msgs::msg::IMU>()
{
  return "ros2_unitree_legged_msgs/msg/IMU";
}

template<>
struct has_fixed_size<ros2_unitree_legged_msgs::msg::IMU>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_unitree_legged_msgs::msg::IMU>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_unitree_legged_msgs::msg::IMU>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__IMU__TRAITS_HPP_
