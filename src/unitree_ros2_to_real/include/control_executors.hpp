#pragma once

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <mutex>
#include <thread>

namespace sim2real {
// IO and inference cannot occupy the workers that deliver observations.
// The two explicit groups must be created with automatic association=false.
inline void SpinControlExecutors(const rclcpp::Node::SharedPtr& node,
    const rclcpp::CallbackGroup::SharedPtr& io,
    const rclcpp::CallbackGroup::SharedPtr& policy, bool shared_probe = false) {
  rclcpp::executors::MultiThreadedExecutor inputs(rclcpp::ExecutorOptions(), 2);
  inputs.add_node(node);
  if (shared_probe) {
    inputs.add_callback_group(io, node->get_node_base_interface());
    inputs.add_callback_group(policy, node->get_node_base_interface());
    inputs.spin();
    return;
  }
  rclcpp::executors::SingleThreadedExecutor io_executor, policy_executor;
  io_executor.add_callback_group(io, node->get_node_base_interface());
  policy_executor.add_callback_group(policy, node->get_node_base_interface());
  std::exception_ptr error;
  std::mutex error_mutex;
  auto run = [&](auto& executor) {
    try { executor.spin(); }
    catch (...) {
      { std::lock_guard lock(error_mutex); if (!error) error = std::current_exception(); }
      rclcpp::shutdown();
    }
  };
  // jthread joins on every exit path, including failure to create a worker.
  std::jthread io_thread([&] { run(io_executor); });
  std::jthread policy_thread;
  try {
    policy_thread = std::jthread([&] { run(policy_executor); });
    run(inputs);
  } catch (...) {
    rclcpp::shutdown();
    io_executor.cancel(); policy_executor.cancel(); inputs.cancel();
    throw;
  }
  io_executor.cancel(); policy_executor.cancel(); inputs.cancel();
  io_thread.join(); policy_thread.join();
  if (error) std::rethrow_exception(error);
}
}  // namespace sim2real
