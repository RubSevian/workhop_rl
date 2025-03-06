// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2_unitree_legged_msgs:msg/HighCmd.idl
// generated code does not contain a copyright notice
#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2_unitree_legged_msgs/msg/detail/high_cmd__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace ros2_unitree_legged_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros2_unitree_legged_msgs::msg::BmsCmd &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2_unitree_legged_msgs::msg::BmsCmd &);
size_t get_serialized_size(
  const ros2_unitree_legged_msgs::msg::BmsCmd &,
  size_t current_alignment);
size_t
max_serialized_size_BmsCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2_unitree_legged_msgs

namespace ros2_unitree_legged_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros2_unitree_legged_msgs::msg::LED &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2_unitree_legged_msgs::msg::LED &);
size_t get_serialized_size(
  const ros2_unitree_legged_msgs::msg::LED &,
  size_t current_alignment);
size_t
max_serialized_size_LED(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2_unitree_legged_msgs


namespace ros2_unitree_legged_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_unitree_legged_msgs
cdr_serialize(
  const ros2_unitree_legged_msgs::msg::HighCmd & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: head
  {
    cdr << ros_message.head;
  }
  // Member: level_flag
  cdr << ros_message.level_flag;
  // Member: frame_reserve
  cdr << ros_message.frame_reserve;
  // Member: sn
  {
    cdr << ros_message.sn;
  }
  // Member: version
  {
    cdr << ros_message.version;
  }
  // Member: band_width
  cdr << ros_message.band_width;
  // Member: mode
  cdr << ros_message.mode;
  // Member: gait_type
  cdr << ros_message.gait_type;
  // Member: speed_level
  cdr << ros_message.speed_level;
  // Member: foot_raise_height
  cdr << ros_message.foot_raise_height;
  // Member: body_height
  cdr << ros_message.body_height;
  // Member: position
  {
    cdr << ros_message.position;
  }
  // Member: euler
  {
    cdr << ros_message.euler;
  }
  // Member: velocity
  {
    cdr << ros_message.velocity;
  }
  // Member: yaw_speed
  cdr << ros_message.yaw_speed;
  // Member: bms
  ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bms,
    cdr);
  // Member: led
  {
    for (size_t i = 0; i < 4; i++) {
      ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.led[i],
        cdr);
    }
  }
  // Member: wireless_remote
  {
    cdr << ros_message.wireless_remote;
  }
  // Member: reserve
  cdr << ros_message.reserve;
  // Member: crc
  cdr << ros_message.crc;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_unitree_legged_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2_unitree_legged_msgs::msg::HighCmd & ros_message)
{
  // Member: head
  {
    cdr >> ros_message.head;
  }

  // Member: level_flag
  cdr >> ros_message.level_flag;

  // Member: frame_reserve
  cdr >> ros_message.frame_reserve;

  // Member: sn
  {
    cdr >> ros_message.sn;
  }

  // Member: version
  {
    cdr >> ros_message.version;
  }

  // Member: band_width
  cdr >> ros_message.band_width;

  // Member: mode
  cdr >> ros_message.mode;

  // Member: gait_type
  cdr >> ros_message.gait_type;

  // Member: speed_level
  cdr >> ros_message.speed_level;

  // Member: foot_raise_height
  cdr >> ros_message.foot_raise_height;

  // Member: body_height
  cdr >> ros_message.body_height;

  // Member: position
  {
    cdr >> ros_message.position;
  }

  // Member: euler
  {
    cdr >> ros_message.euler;
  }

  // Member: velocity
  {
    cdr >> ros_message.velocity;
  }

  // Member: yaw_speed
  cdr >> ros_message.yaw_speed;

  // Member: bms
  ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bms);

  // Member: led
  {
    for (size_t i = 0; i < 4; i++) {
      ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.led[i]);
    }
  }

  // Member: wireless_remote
  {
    cdr >> ros_message.wireless_remote;
  }

  // Member: reserve
  cdr >> ros_message.reserve;

  // Member: crc
  cdr >> ros_message.crc;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_unitree_legged_msgs
get_serialized_size(
  const ros2_unitree_legged_msgs::msg::HighCmd & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: head
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.head[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: level_flag
  {
    size_t item_size = sizeof(ros_message.level_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: frame_reserve
  {
    size_t item_size = sizeof(ros_message.frame_reserve);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sn
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.sn[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.version[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: band_width
  {
    size_t item_size = sizeof(ros_message.band_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mode
  {
    size_t item_size = sizeof(ros_message.mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gait_type
  {
    size_t item_size = sizeof(ros_message.gait_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_level
  {
    size_t item_size = sizeof(ros_message.speed_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: foot_raise_height
  {
    size_t item_size = sizeof(ros_message.foot_raise_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: body_height
  {
    size_t item_size = sizeof(ros_message.body_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: euler
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.euler[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: velocity
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.velocity[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: yaw_speed
  {
    size_t item_size = sizeof(ros_message.yaw_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bms

  current_alignment +=
    ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bms, current_alignment);
  // Member: led
  {
    size_t array_size = 4;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.led[index], current_alignment);
    }
  }
  // Member: wireless_remote
  {
    size_t array_size = 40;
    size_t item_size = sizeof(ros_message.wireless_remote[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reserve
  {
    size_t item_size = sizeof(ros_message.reserve);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: crc
  {
    size_t item_size = sizeof(ros_message.crc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2_unitree_legged_msgs
max_serialized_size_HighCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: head
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: level_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: frame_reserve
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sn
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: version
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: band_width
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gait_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: speed_level
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: foot_raise_height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: body_height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: euler
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: velocity
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: yaw_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bms
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_BmsCmd(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: led
  {
    size_t array_size = 4;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_LED(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: wireless_remote
  {
    size_t array_size = 40;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reserve
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: crc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2_unitree_legged_msgs::msg::HighCmd;
    is_plain =
      (
      offsetof(DataType, crc) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _HighCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2_unitree_legged_msgs::msg::HighCmd *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _HighCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2_unitree_legged_msgs::msg::HighCmd *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _HighCmd__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2_unitree_legged_msgs::msg::HighCmd *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _HighCmd__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_HighCmd(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _HighCmd__callbacks = {
  "ros2_unitree_legged_msgs::msg",
  "HighCmd",
  _HighCmd__cdr_serialize,
  _HighCmd__cdr_deserialize,
  _HighCmd__get_serialized_size,
  _HighCmd__max_serialized_size
};

static rosidl_message_type_support_t _HighCmd__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_HighCmd__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros2_unitree_legged_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2_unitree_legged_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_unitree_legged_msgs::msg::HighCmd>()
{
  return &ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::_HighCmd__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2_unitree_legged_msgs, msg, HighCmd)() {
  return &ros2_unitree_legged_msgs::msg::typesupport_fastrtps_cpp::_HighCmd__handle;
}

#ifdef __cplusplus
}
#endif
