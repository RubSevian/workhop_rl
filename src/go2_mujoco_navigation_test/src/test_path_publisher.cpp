#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class TestPathPublisher final : public rclcpp::Node {
public:
  TestPathPublisher() : Node("test_path_publisher") {
    declare_parameter<std::string>("test_path", "straight");
    declare_parameter<double>("path_step", 0.25);
    declare_parameter<double>("publish_delay_sec", 2.0);
    declare_parameter<double>("goal_tolerance", 0.30);
    declare_parameter<std::string>("csv_path", "/tmp/go2_mujoco_smoke_test.csv");
    declare_parameter<std::string>("frame_id", "vehicle");

    path_type_ = get_parameter("test_path").as_string();
    const double step = get_parameter("path_step").as_double();
    publish_delay_sec_ = get_parameter("publish_delay_sec").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    BuildPath(step);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(1));
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/navigation_active", rclcpp::QoS(1).transient_local());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", rclcpp::QoS(20),
      std::bind(&TestPathPublisher::OnOdom, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(20),
      std::bind(&TestPathPublisher::OnCmd, this, std::placeholders::_1));

    csv_.open(get_parameter("csv_path").as_string(), std::ios::trunc);
    if (csv_) csv_ << "time,x,y,yaw,cmd_vx,cmd_vy,cmd_wz,goal_distance\n";
    std_msgs::msg::Bool inactive;
    inactive.data = false;
    active_pub_->publish(inactive);
    start_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TestPathPublisher::Tick, this));
  }

private:
  void BuildPath(double step) {
    if (!std::isfinite(step) || step <= 0.0) throw std::invalid_argument("path_step must be positive");
    const auto add_line = [this, step](double x0, double y0, double x1, double y1) {
      const double distance = std::hypot(x1 - x0, y1 - y0);
      const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
      for (int index = 0; index <= count; ++index) {
        const double alpha = static_cast<double>(index) / count;
        points_.emplace_back(x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0));
      }
    };
    if (path_type_ == "straight") add_line(0.0, 0.0, 2.0, 0.0);
    else if (path_type_ == "l_shape") { add_line(0.0, 0.0, 1.5, 0.0); add_line(1.5, 0.0, 1.5, 1.5); }
    else if (path_type_ == "lateral") add_line(0.0, 0.0, 0.0, 1.0);
    else if (path_type_ == "diagonal") add_line(0.0, 0.0, 1.5, 1.5);
    else throw std::invalid_argument("test_path must be straight, l_shape, lateral, or diagonal");
    goal_x_ = points_.back().first;
    goal_y_ = points_.back().second;
  }

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = *msg; have_odom_ = true; }
  void OnCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { cmd_ = *msg; }

  void Tick() {
    if (!have_odom_) return;
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (!navigation_armed_ && elapsed >= publish_delay_sec_) {
      std_msgs::msg::Bool active;
      active.data = true;
      active_pub_->publish(active);
      navigation_armed_ = true;
      const auto& q = odom_.pose.pose.orientation;
      const double initial_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double cos_yaw = std::cos(initial_yaw);
      const double sin_yaw = std::sin(initial_yaw);
      const double initial_x = odom_.pose.pose.position.x;
      const double initial_y = odom_.pose.pose.position.y;
      world_points_.reserve(points_.size());
      for (const auto& [x, y] : points_) {
        world_points_.emplace_back(
          initial_x + cos_yaw * x - sin_yaw * y,
          initial_y + sin_yaw * x + cos_yaw * y);
      }
      goal_world_x_ = world_points_.back().first;
      goal_world_y_ = world_points_.back().second;
      return;  // Ensure the follower receives navigation_active before /path.
    }
    if (navigation_armed_ && !finished_) {
      // pathFollower rejects a path older than pathTimeoutSec.  Re-express the
      // fixed world route in the current vehicle frame on every timer tick.
      const auto& position = odom_.pose.pose.position;
      const auto& q = odom_.pose.pose.orientation;
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      nav_msgs::msg::Path path;
      path.header.stamp = now();
      path.header.frame_id = frame_id_;
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose.header = path.header;
      current_pose.pose.orientation.w = 1.0;
      path.poses.push_back(current_pose);

      std::size_t nearest_index = 0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < world_points_.size(); ++index) {
        const double distance = std::hypot(
          world_points_[index].first - position.x,
          world_points_[index].second - position.y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest_index = index;
        }
      }
      const std::size_t first_target = std::min(nearest_index + 1, world_points_.size() - 1);
      for (std::size_t index = first_target; index < world_points_.size(); ++index) {
        const auto& [world_x, world_y] = world_points_[index];
        const double dx = world_x - position.x;
        const double dy = world_y - position.y;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = cos_yaw * dx + sin_yaw * dy;
        pose.pose.position.y = -sin_yaw * dx + cos_yaw * dy;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      path_pub_->publish(path);
      if (!path_sent_) {
        path_sent_ = true;
        RCLCPP_INFO(get_logger(), "Started %s test path; fixed goal=(%.2f, %.2f)",
                    path_type_.c_str(), goal_world_x_, goal_world_y_);
      }
    }
    const auto& position = odom_.pose.pose.position;
    const double distance = std::hypot(position.x - goal_world_x_, position.y - goal_world_y_);
    const auto& q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (csv_) csv_ << elapsed << ',' << position.x << ',' << position.y << ',' << yaw << ','
                   << cmd_.twist.linear.x << ',' << cmd_.twist.linear.y << ',' << cmd_.twist.angular.z << ','
                   << distance << '\n';
    if (path_sent_ && !finished_ && distance <= goal_tolerance_) {
      finished_ = true;
      std_msgs::msg::Bool inactive;
      inactive.data = false;
      active_pub_->publish(inactive);
      RCLCPP_INFO(get_logger(),
        "Smoke test complete: goal_x=%.3f goal_y=%.3f final_x=%.3f final_y=%.3f position_error=%.3f",
        goal_world_x_, goal_world_y_, position.x, position.y, distance);
    }
  }

  std::string path_type_, frame_id_;
  std::vector<std::pair<double, double>> points_;
  std::vector<std::pair<double, double>> world_points_;
  double goal_x_{0.0}, goal_y_{0.0}, goal_world_x_{0.0}, goal_world_y_{0.0};
  double publish_delay_sec_{2.0}, goal_tolerance_{0.30};
  bool have_odom_{false}, navigation_armed_{false}, path_sent_{false}, finished_{false};
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::TwistStamped cmd_;
  std::ofstream csv_;
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
