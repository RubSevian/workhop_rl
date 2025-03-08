// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_unitree_legged_msgs:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__STRUCT_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'motor_cmd'
#include "ros2_unitree_legged_msgs/msg/detail/motor_cmd__struct.hpp"
// Member 'bms'
#include "ros2_unitree_legged_msgs/msg/detail/bms_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_unitree_legged_msgs__msg__LowCmd __attribute__((deprecated))
#else
# define DEPRECATED__ros2_unitree_legged_msgs__msg__LowCmd __declspec(deprecated)
#endif

namespace ros2_unitree_legged_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LowCmd_
{
  using Type = LowCmd_<ContainerAllocator>;

  explicit LowCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bms(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->head.begin(), this->head.end(), 0);
      this->level_flag = 0;
      this->frame_reserve = 0;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->sn.begin(), this->sn.end(), 0ul);
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->version.begin(), this->version.end(), 0ul);
      this->band_width = 0;
      this->motor_cmd.fill(ros2_unitree_legged_msgs::msg::MotorCmd_<ContainerAllocator>{_init});
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->wireless_remote.begin(), this->wireless_remote.end(), 0);
      this->reserve = 0ul;
      this->crc = 0ul;
    }
  }

  explicit LowCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : head(_alloc),
    sn(_alloc),
    version(_alloc),
    motor_cmd(_alloc),
    bms(_alloc, _init),
    wireless_remote(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->head.begin(), this->head.end(), 0);
      this->level_flag = 0;
      this->frame_reserve = 0;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->sn.begin(), this->sn.end(), 0ul);
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->version.begin(), this->version.end(), 0ul);
      this->band_width = 0;
      this->motor_cmd.fill(ros2_unitree_legged_msgs::msg::MotorCmd_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->wireless_remote.begin(), this->wireless_remote.end(), 0);
      this->reserve = 0ul;
      this->crc = 0ul;
    }
  }

  // field types and members
  using _head_type =
    std::array<uint8_t, 2>;
  _head_type head;
  using _level_flag_type =
    uint8_t;
  _level_flag_type level_flag;
  using _frame_reserve_type =
    uint8_t;
  _frame_reserve_type frame_reserve;
  using _sn_type =
    std::array<uint32_t, 2>;
  _sn_type sn;
  using _version_type =
    std::array<uint32_t, 2>;
  _version_type version;
  using _band_width_type =
    uint16_t;
  _band_width_type band_width;
  using _motor_cmd_type =
    std::array<ros2_unitree_legged_msgs::msg::MotorCmd_<ContainerAllocator>, 20>;
  _motor_cmd_type motor_cmd;
  using _bms_type =
    ros2_unitree_legged_msgs::msg::BmsCmd_<ContainerAllocator>;
  _bms_type bms;
  using _wireless_remote_type =
    std::array<uint8_t, 40>;
  _wireless_remote_type wireless_remote;
  using _reserve_type =
    uint32_t;
  _reserve_type reserve;
  using _crc_type =
    uint32_t;
  _crc_type crc;

  // setters for named parameter idiom
  Type & set__head(
    const std::array<uint8_t, 2> & _arg)
  {
    this->head = _arg;
    return *this;
  }
  Type & set__level_flag(
    const uint8_t & _arg)
  {
    this->level_flag = _arg;
    return *this;
  }
  Type & set__frame_reserve(
    const uint8_t & _arg)
  {
    this->frame_reserve = _arg;
    return *this;
  }
  Type & set__sn(
    const std::array<uint32_t, 2> & _arg)
  {
    this->sn = _arg;
    return *this;
  }
  Type & set__version(
    const std::array<uint32_t, 2> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__band_width(
    const uint16_t & _arg)
  {
    this->band_width = _arg;
    return *this;
  }
  Type & set__motor_cmd(
    const std::array<ros2_unitree_legged_msgs::msg::MotorCmd_<ContainerAllocator>, 20> & _arg)
  {
    this->motor_cmd = _arg;
    return *this;
  }
  Type & set__bms(
    const ros2_unitree_legged_msgs::msg::BmsCmd_<ContainerAllocator> & _arg)
  {
    this->bms = _arg;
    return *this;
  }
  Type & set__wireless_remote(
    const std::array<uint8_t, 40> & _arg)
  {
    this->wireless_remote = _arg;
    return *this;
  }
  Type & set__reserve(
    const uint32_t & _arg)
  {
    this->reserve = _arg;
    return *this;
  }
  Type & set__crc(
    const uint32_t & _arg)
  {
    this->crc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_unitree_legged_msgs__msg__LowCmd
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_unitree_legged_msgs__msg__LowCmd
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LowCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LowCmd_ & other) const
  {
    if (this->head != other.head) {
      return false;
    }
    if (this->level_flag != other.level_flag) {
      return false;
    }
    if (this->frame_reserve != other.frame_reserve) {
      return false;
    }
    if (this->sn != other.sn) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->band_width != other.band_width) {
      return false;
    }
    if (this->motor_cmd != other.motor_cmd) {
      return false;
    }
    if (this->bms != other.bms) {
      return false;
    }
    if (this->wireless_remote != other.wireless_remote) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    if (this->crc != other.crc) {
      return false;
    }
    return true;
  }
  bool operator!=(const LowCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LowCmd_

// alias to use template instance with default allocator
using LowCmd =
  ros2_unitree_legged_msgs::msg::LowCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LOW_CMD__STRUCT_HPP_
