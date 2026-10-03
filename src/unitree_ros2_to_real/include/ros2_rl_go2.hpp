#pragma once
// R1 controller is diagnostic-only. Actuator transport is deliberately absent.
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
  sim2real::OutputGate output_gate_;
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
