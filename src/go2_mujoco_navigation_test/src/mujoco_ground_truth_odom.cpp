#include <array>
#include <cmath>
#include <mutex>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

class MujocoGroundTruthOdom final : public rclcpp::Node {
public:
  MujocoGroundTruthOdom() : Node("mujoco_ground_truth_odom") {
    declare_parameter<std::string>("sport_state_topic", "sportmodestate");
    declare_parameter<std::string>("low_state_topic", "lowstate");
    declare_parameter<std::string>("odom_topic", "/state_estimation");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("child_frame_id", "vehicle");
    declare_parameter<double>("publish_rate_hz", 100.0);

    const auto sport_topic = get_parameter("sport_state_topic").as_string();
    const auto low_topic = get_parameter("low_state_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    const auto rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    sport_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_topic, rclcpp::QoS(20),
      std::bind(&MujocoGroundTruthOdom::OnSportState, this, std::placeholders::_1));
    low_sub_ = create_subscription<unitree_go::msg::LowState>(
      low_topic, rclcpp::QoS(20),
      std::bind(&MujocoGroundTruthOdom::OnLowState, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&MujocoGroundTruthOdom::Publish, this));
  }

private:
  void OnSportState(const unitree_go::msg::SportModeState::SharedPtr msg) {
    std::scoped_lock lock(data_mutex_);
    position_ = msg->position;
    linear_velocity_ = msg->velocity;
    have_sport_state_ = true;
  }

  void OnLowState(const unitree_go::msg::LowState::SharedPtr msg) {
    std::scoped_lock lock(data_mutex_);
    // Unitree LowState stores quaternion in [w, x, y, z] order.
    quaternion_wxyz_ = msg->imu_state.quaternion;
    angular_velocity_ = msg->imu_state.gyroscope;
    have_low_state_ = true;
  }

  void Publish() {
    std::scoped_lock lock(data_mutex_);
    if (!have_sport_state_ || !have_low_state_) return;
    const auto& q = quaternion_wxyz_;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm < 1.0e-6) return;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = position_[0];
    odom.pose.pose.position.y = position_[1];
    odom.pose.pose.position.z = position_[2];
    odom.pose.pose.orientation.w = q[0] / norm;
    odom.pose.pose.orientation.x = q[1] / norm;
    odom.pose.pose.orientation.y = q[2] / norm;
    odom.pose.pose.orientation.z = q[3] / norm;
    odom.twist.twist.linear.x = linear_velocity_[0];
    odom.twist.twist.linear.y = linear_velocity_[1];
    odom.twist.twist.linear.z = linear_velocity_[2];
    odom.twist.twist.angular.x = angular_velocity_[0];
    odom.twist.twist.angular.y = angular_velocity_[1];
    odom.twist.twist.angular.z = angular_velocity_[2];
    odom_pub_->publish(odom);
  }

  std::mutex data_mutex_;
  std::array<float, 3> position_{};
  std::array<float, 3> linear_velocity_{};
  std::array<float, 4> quaternion_wxyz_{};
  std::array<float, 3> angular_velocity_{};
  bool have_sport_state_{false};
  bool have_low_state_{false};
  std::string odom_topic_, frame_id_, child_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MujocoGroundTruthOdom>());
  rclcpp::shutdown();
  return 0;
}
