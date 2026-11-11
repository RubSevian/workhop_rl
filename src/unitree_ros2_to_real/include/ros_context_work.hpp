#pragma once
#include <rclcpp/context.hpp>
#include <exception>

namespace sim2real {
// ROS graph/client APIs can throw std::runtime_error (not only RCLError)
// when shutdown invalidates the context between a validity check and a call.
// Never suppress an error while the node's own context is still live.
template<class Work>
bool WithRosContext(const rclcpp::Context::SharedPtr& context, Work&& work) {
  if (!context->is_valid()) return false;
  try { work(); }
  catch (const std::exception&) {
    if (context->is_valid()) throw;
    return false;
  }
  return true;
}
}
