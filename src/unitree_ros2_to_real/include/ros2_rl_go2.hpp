#pragma once
// R2 read-only diagnostic executable. No actuator publisher or serial owner.
#include "real_controller_core.hpp"
#include "navigation_command_adapter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree_go/msg/low_state.hpp>

class InterfaceRos : public rclcpp::Node {
 public:
  InterfaceRos();
 private:
  sim2real::RealControllerCore core_;
  sim2real::LowStateReader lowstate_;
  sim2real::RemoteSafety remote_;
  sim2real::SafetyTime diagnostic_stamp_{};
  NavigationCommandAdapter navigation_;
  double low_state_timeout_=0.5;
  std::array<double,3> limits_{};
  bool received_state_=false;
  NavigationCommandAdapter::TimePoint state_stamp_{};
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
