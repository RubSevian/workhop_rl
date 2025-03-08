// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_unitree_legged_msgs:msg/HighCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__TRAITS_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'bms'
#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__traits.hpp"
// Member 'led'
#include "ros2_unitree_legged_msgs/msg/detail/led__traits.hpp"

namespace ros2_unitree_legged_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HighCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: head
  {
    if (msg.head.size() == 0) {
      out << "head: []";
    } else {
      out << "head: [";
      size_t pending_items = msg.head.size();
      for (auto item : msg.head) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: level_flag
  {
    out << "level_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.level_flag, out);
    out << ", ";
  }

  // member: frame_reserve
  {
    out << "frame_reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_reserve, out);
    out << ", ";
  }

  // member: sn
  {
    if (msg.sn.size() == 0) {
      out << "sn: []";
    } else {
      out << "sn: [";
      size_t pending_items = msg.sn.size();
      for (auto item : msg.sn) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: version
  {
    if (msg.version.size() == 0) {
      out << "version: []";
    } else {
      out << "version: [";
      size_t pending_items = msg.version.size();
      for (auto item : msg.version) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: band_width
  {
    out << "band_width: ";
    rosidl_generator_traits::value_to_yaml(msg.band_width, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: gait_type
  {
    out << "gait_type: ";
    rosidl_generator_traits::value_to_yaml(msg.gait_type, out);
    out << ", ";
  }

  // member: speed_level
  {
    out << "speed_level: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_level, out);
    out << ", ";
  }

  // member: foot_raise_height
  {
    out << "foot_raise_height: ";
    rosidl_generator_traits::value_to_yaml(msg.foot_raise_height, out);
    out << ", ";
  }

  // member: body_height
  {
    out << "body_height: ";
    rosidl_generator_traits::value_to_yaml(msg.body_height, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: euler
  {
    if (msg.euler.size() == 0) {
      out << "euler: []";
    } else {
      out << "euler: [";
      size_t pending_items = msg.euler.size();
      for (auto item : msg.euler) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity
  {
    if (msg.velocity.size() == 0) {
      out << "velocity: []";
    } else {
      out << "velocity: [";
      size_t pending_items = msg.velocity.size();
      for (auto item : msg.velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: yaw_speed
  {
    out << "yaw_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_speed, out);
    out << ", ";
  }

  // member: bms
  {
    out << "bms: ";
    to_flow_style_yaml(msg.bms, out);
    out << ", ";
  }

  // member: led
  {
    if (msg.led.size() == 0) {
      out << "led: []";
    } else {
      out << "led: [";
      size_t pending_items = msg.led.size();
      for (auto item : msg.led) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wireless_remote
  {
    if (msg.wireless_remote.size() == 0) {
      out << "wireless_remote: []";
    } else {
      out << "wireless_remote: [";
      size_t pending_items = msg.wireless_remote.size();
      for (auto item : msg.wireless_remote) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: reserve
  {
    out << "reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.reserve, out);
    out << ", ";
  }

  // member: crc
  {
    out << "crc: ";
    rosidl_generator_traits::value_to_yaml(msg.crc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HighCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: head
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.head.size() == 0) {
      out << "head: []\n";
    } else {
      out << "head:\n";
      for (auto item : msg.head) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: level_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.level_flag, out);
    out << "\n";
  }

  // member: frame_reserve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_reserve, out);
    out << "\n";
  }

  // member: sn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sn.size() == 0) {
      out << "sn: []\n";
    } else {
      out << "sn:\n";
      for (auto item : msg.sn) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.version.size() == 0) {
      out << "version: []\n";
    } else {
      out << "version:\n";
      for (auto item : msg.version) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: band_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "band_width: ";
    rosidl_generator_traits::value_to_yaml(msg.band_width, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: gait_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gait_type: ";
    rosidl_generator_traits::value_to_yaml(msg.gait_type, out);
    out << "\n";
  }

  // member: speed_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_level: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_level, out);
    out << "\n";
  }

  // member: foot_raise_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foot_raise_height: ";
    rosidl_generator_traits::value_to_yaml(msg.foot_raise_height, out);
    out << "\n";
  }

  // member: body_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_height: ";
    rosidl_generator_traits::value_to_yaml(msg.body_height, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: euler
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.euler.size() == 0) {
      out << "euler: []\n";
    } else {
      out << "euler:\n";
      for (auto item : msg.euler) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocity.size() == 0) {
      out << "velocity: []\n";
    } else {
      out << "velocity:\n";
      for (auto item : msg.velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: yaw_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_speed, out);
    out << "\n";
  }

  // member: bms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms:\n";
    to_block_style_yaml(msg.bms, out, indentation + 2);
  }

  // member: led
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.led.size() == 0) {
      out << "led: []\n";
    } else {
      out << "led:\n";
      for (auto item : msg.led) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: wireless_remote
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wireless_remote.size() == 0) {
      out << "wireless_remote: []\n";
    } else {
      out << "wireless_remote:\n";
      for (auto item : msg.wireless_remote) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: reserve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.reserve, out);
    out << "\n";
  }

  // member: crc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "crc: ";
    rosidl_generator_traits::value_to_yaml(msg.crc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HighCmd & msg, bool use_flow_style = false)
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
  const ros2_unitree_legged_msgs::msg::HighCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_unitree_legged_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_unitree_legged_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_unitree_legged_msgs::msg::HighCmd & msg)
{
  return ros2_unitree_legged_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_unitree_legged_msgs::msg::HighCmd>()
{
  return "ros2_unitree_legged_msgs::msg::HighCmd";
}

template<>
inline const char * name<ros2_unitree_legged_msgs::msg::HighCmd>()
{
  return "ros2_unitree_legged_msgs/msg/HighCmd";
}

template<>
struct has_fixed_size<ros2_unitree_legged_msgs::msg::HighCmd>
  : std::integral_constant<bool, has_fixed_size<ros2_unitree_legged_msgs::msg::BmsCmd>::value && has_fixed_size<ros2_unitree_legged_msgs::msg::LED>::value> {};

template<>
struct has_bounded_size<ros2_unitree_legged_msgs::msg::HighCmd>
  : std::integral_constant<bool, has_bounded_size<ros2_unitree_legged_msgs::msg::BmsCmd>::value && has_bounded_size<ros2_unitree_legged_msgs::msg::LED>::value> {};

template<>
struct is_message<ros2_unitree_legged_msgs::msg::HighCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__HIGH_CMD__TRAITS_HPP_
