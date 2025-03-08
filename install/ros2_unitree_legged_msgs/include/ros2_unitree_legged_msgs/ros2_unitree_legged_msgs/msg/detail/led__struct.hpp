// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_unitree_legged_msgs:msg/LED.idl
// generated code does not contain a copyright notice

#ifndef ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LED__STRUCT_HPP_
#define ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_unitree_legged_msgs__msg__LED __attribute__((deprecated))
#else
# define DEPRECATED__ros2_unitree_legged_msgs__msg__LED __declspec(deprecated)
#endif

namespace ros2_unitree_legged_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LED_
{
  using Type = LED_<ContainerAllocator>;

  explicit LED_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0;
      this->g = 0;
      this->b = 0;
    }
  }

  explicit LED_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0;
      this->g = 0;
      this->b = 0;
    }
  }

  // field types and members
  using _r_type =
    uint8_t;
  _r_type r;
  using _g_type =
    uint8_t;
  _g_type g;
  using _b_type =
    uint8_t;
  _b_type b;

  // setters for named parameter idiom
  Type & set__r(
    const uint8_t & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__g(
    const uint8_t & _arg)
  {
    this->g = _arg;
    return *this;
  }
  Type & set__b(
    const uint8_t & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_unitree_legged_msgs__msg__LED
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_unitree_legged_msgs__msg__LED
    std::shared_ptr<ros2_unitree_legged_msgs::msg::LED_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LED_ & other) const
  {
    if (this->r != other.r) {
      return false;
    }
    if (this->g != other.g) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const LED_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LED_

// alias to use template instance with default allocator
using LED =
  ros2_unitree_legged_msgs::msg::LED_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_unitree_legged_msgs

#endif  // ROS2_UNITREE_LEGGED_MSGS__MSG__DETAIL__LED__STRUCT_HPP_
