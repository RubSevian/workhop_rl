// Collision gate for simulation-only manual arm trajectories. Never changes live physics.
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <cstdio>
#include <stdexcept>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr std::array<const char*, 8> kArmJoints = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "gripper_left_joint", "gripper_right_joint"};
class ArmShadowChecker final : public rclcpp::Node {
 public:
  ArmShadowChecker() : Node("stage4d_arm_shadow_checker") {
    const std::string path = std::string(mujoco_dir) + "/../unitree_robots/go2_rars01/scene_stage4d.xml";
    char error[1024] = {};
    model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error(std::string("SHADOW_MODEL_LOAD_FAILED: ") + error);
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("SHADOW_DATA_ALLOCATION_FAILED");
    const int base = mj_name2id(model_.get(), mjOBJ_BODY, "base_link");
    mount_body_id_ = mj_name2id(model_.get(), mjOBJ_BODY, "arm_mount_link");
    first_body_id_ = mj_name2id(model_.get(), mjOBJ_BODY, "link1_m0");
    const int freejoint = mj_name2id(model_.get(), mjOBJ_JOINT, "base_freejoint");
    if (base < 0 || mount_body_id_ < 0 || first_body_id_ < 0 || freejoint < 0) throw std::runtime_error("SHADOW_BODY_CONTRACT_FAILED");
    free_qpos_ = model_->jnt_qposadr[freejoint];
    for (const char* name : kArmJoints) {
      const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, name);
      if (joint < 0) throw std::runtime_error(std::string("SHADOW_JOINT_MISSING: ") + name);
      arm_qpos_.push_back(model_->jnt_qposadr[joint]);
    }
    for (int geom = 0; geom < model_->ngeom; ++geom) {
      int body = model_->geom_bodyid[geom];
      int ancestor = body;
      while (ancestor != 0 && ancestor != base && ancestor != mount_body_id_)
        ancestor = model_->body_parentid[ancestor];
      if (ancestor == base && body != base) arm_geoms_.push_back(geom);
    }
    if (arm_geoms_.empty()) throw std::runtime_error("SHADOW_ARM_GEOMETRY_MISSING");
    margin_ = declare_parameter<double>("margin_m", 0.01);
    monitor_self_collisions_ = declare_parameter<bool>("monitor_self_collisions", false);
    if (!std::isfinite(margin_) || margin_ < 0.0)
      throw std::runtime_error("SHADOW_INVALID_COLLISION_MARGIN");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/sim/ground_truth_odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          odom_ = *message; odom_at_ = now();
        });
    motor_sub_ = create_subscription<sensor_msgs::msg::JointState>("/go2/motor_state", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr message) {
          motor_.clear();
          for (size_t i = 0; i < std::min(message->name.size(), message->position.size()); ++i)
            motor_[message->name[i]] = message->position[i];
          motor_at_ = now();
        });
    result_pub_ = create_publisher<std_msgs::msg::String>("/stage4d/arm_trajectory_check_result", 10);
    request_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/stage4d/arm_trajectory_check_request", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr request) { Check(request->data); });
  }
 private:
  struct ModelDeleter { void operator()(mjModel* p) const { if (p) mj_deleteModel(p); } };
  struct DataDeleter { void operator()(mjData* p) const { if (p) mj_deleteData(p); } };
  void Reply(int id, bool ok, const std::string& reason, double distance,
             int first, int second, int sample, const std::string& kind) {
    auto name = [this](int geom) {
      if (geom < 0) return std::string("-");
      const char* value = mj_id2name(model_.get(), mjOBJ_GEOM, geom);
      const int body = model_->geom_bodyid[geom];
      const char* body_name = mj_id2name(model_.get(), mjOBJ_BODY, body);
      return (value ? std::string(value) : std::string("geom_") + std::to_string(geom)) +
             "@" + (body_name ? std::string(body_name) : std::string("world"));
    };
    std::ostringstream stream;
    stream << "{\"id\":" << id << ",\"ok\":" << (ok ? "true" : "false")
           << ",\"reason\":\"" << reason << "\",\"minimum_distance_m\":" << distance
           << ",\"closest_pair\":[\"" << name(first) << "\",\"" << name(second)
           << "\"],\"sample_index\":" << sample << ",\"sample_time_s\":" << (sample < 0 ? -1.0 : sample / 100.0)
           << ",\"collision_class\":\"" << kind << "\",\"proximity_scan_cutoff_m\":" << std::max(0.01, margin_)
           << ",\"joint_vector\":[";
    for (size_t i = 0; i < arm_qpos_.size(); ++i) {
      if (i) stream << ',';
      stream << data_->qpos[arm_qpos_[i]];
    }
    stream << "],\"self_collision_monitor\":{"
           << "\"enabled\":" << (monitor_self_collisions_ ? "true" : "false")
           << ",\"minimum_distance_m\":" << (self_closest_a_ < 0 ? -1.0 : self_minimum_)
           << ",\"closest_pair\":[\"" << name(self_closest_a_) << "\",\"" << name(self_closest_b_)
           << "\"],\"sample_index\":" << self_closest_sample_
           << ",\"below_margin\":" << (self_closest_a_ >= 0 && self_minimum_ < margin_ ? "true" : "false")
           << ",\"pairs\":[";
    bool first_pair = true;
    for (const auto& entry : self_pairs_) {
      if (!first_pair) stream << ',';
      first_pair = false;
      stream << "{\"pair\":[\"" << name(entry.first.first) << "\",\""
             << name(entry.first.second) << "\"],\"minimum_distance_m\":" << entry.second.distance
             << ",\"sample_index\":" << entry.second.sample
             << ",\"below_margin\":" << (entry.second.distance < margin_ ? "true" : "false") << '}';
    }
    stream << "]}}";
    if (monitor_self_collisions_) for (const auto& entry : self_pairs_) {
      if (entry.second.distance >= margin_) continue;
      const char* first_name = mj_id2name(model_.get(), mjOBJ_BODY, model_->geom_bodyid[entry.first.first]);
      const char* second_name = mj_id2name(model_.get(), mjOBJ_BODY, model_->geom_bodyid[entry.first.second]);
      RCLCPP_WARN(get_logger(), "SELF_COLLISION_MONITORED pair=%s-%s distance_m=%.6f sample=%d",
                  first_name ? first_name : "?", second_name ? second_name : "?",
                  entry.second.distance, entry.second.sample);
    }
    std_msgs::msg::String result; result.data = stream.str(); result_pub_->publish(result);
    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - check_started_at_).count();
    RCLCPP_INFO(get_logger(), "SHADOW_CHECK_DONE id=%d ok=%d reason=%s elapsed_s=%.3f",
                id, ok ? 1 : 0, reason.c_str(), elapsed_s);
  }
  bool IsArm(int geom) const {
    return std::find(arm_geoms_.begin(), arm_geoms_.end(), geom) != arm_geoms_.end();
  }
  bool Adjacent(int a, int b) const {
    const int ba = model_->geom_bodyid[a], bb = model_->geom_bodyid[b];
    if (ba == bb || model_->body_parentid[ba] == bb || model_->body_parentid[bb] == ba)
      return true;
    // The stationary RARS base_link lies between link1_m0 and arm_mount_link.
    // Their meshes overlap by about 1.2 mm at HOME; whitelist this one
    // physically adjacent pair, not every mount or arm self-pair.
    return (ba == first_body_id_ && bb == mount_body_id_) ||
           (bb == first_body_id_ && ba == mount_body_id_);
  }
  std::string ClassOf(int geom) const {
    const int body = model_->geom_bodyid[geom];
    if (IsArm(geom)) return "SELF";
    const char* geom_name = mj_id2name(model_.get(), mjOBJ_GEOM, geom);
    if (geom_name && std::string(geom_name) == "floor") return "FLOOR";
    if (body == 0 || std::string(mj_id2name(model_.get(), mjOBJ_BODY, body) ?: "") == "stage4d_landmarks")
      return "ENVIRONMENT";
    int ancestor = body;
    while (ancestor != 0 && ancestor != mount_body_id_) ancestor = model_->body_parentid[ancestor];
    return ancestor == mount_body_id_ ? "MOUNT" : "ROBOT_BODY";
  }
  void Check(const std::vector<double>& request) {
    if (request.size() < 2 || !std::isfinite(request[0]) || !std::isfinite(request[1])) return;
    self_minimum_ = std::numeric_limits<double>::infinity();
    self_closest_a_ = self_closest_b_ = self_closest_sample_ = -1;
    self_pairs_.clear();
    check_started_at_ = std::chrono::steady_clock::now();
    const int id = static_cast<int>(request[0]);
    const int count = static_cast<int>(request[1]);
    if (count <= 0 || count > 10000 || request.size() != static_cast<size_t>(2 + count * 8)) {
      Reply(id, false, "INVALID_TRAJECTORY", -1, -1, -1, -1, "INPUT"); return;
    }
    const double odom_age = (now() - odom_at_).seconds();
    const double motor_age = (now() - motor_at_).seconds();
    if (!odom_ || motor_.empty() || odom_age < 0.0 || motor_age < 0.0 ||
        odom_age > 0.25 || motor_age > 0.25) {
      Reply(id, false, "SHADOW_STATE_STALE", -1, -1, -1, -1, "STATE"); return;
    }
    for (int i = 0; i < model_->nq; ++i) data_->qpos[i] = model_->qpos0[i];
    const auto& p = odom_->pose.pose.position; const auto& q = odom_->pose.pose.orientation;
    data_->qpos[free_qpos_ + 0] = p.x; data_->qpos[free_qpos_ + 1] = p.y; data_->qpos[free_qpos_ + 2] = p.z;
    data_->qpos[free_qpos_ + 3] = q.w; data_->qpos[free_qpos_ + 4] = q.x;
    data_->qpos[free_qpos_ + 5] = q.y; data_->qpos[free_qpos_ + 6] = q.z;
    for (const auto& item : motor_) {
      const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, item.first.c_str());
      if (joint >= 0 && model_->jnt_type[joint] != mjJNT_FREE)
        data_->qpos[model_->jnt_qposadr[joint]] = item.second;
    }
    double minimum = std::numeric_limits<double>::infinity();
    int closest_a = -1, closest_b = -1, closest_sample = -1;
    std::string closest_kind = "NONE";
    for (int sample = 0; sample < count; ++sample) {
      for (int joint = 0; joint < 8; ++joint) {
        const double value = request[2 + sample * 8 + joint];
        if (!std::isfinite(value)) { Reply(id, false, "INVALID_TRAJECTORY", -1, -1, -1, sample, "INPUT"); return; }
        const int joint_id = mj_name2id(model_.get(), mjOBJ_JOINT, kArmJoints[joint]);
        if (model_->jnt_limited[joint_id] &&
            (value < model_->jnt_range[2 * joint_id] - 1e-6 ||
             value > model_->jnt_range[2 * joint_id + 1] + 1e-6)) {
          Reply(id, false, "JOINT_LIMIT_REJECTED", -1, -1, -1, sample, "JOINT");
          return;
        }
        data_->qpos[arm_qpos_[joint]] = value;
      }
      mj_kinematics(model_.get(), data_.get());
      for (int a : arm_geoms_) for (int b = 0; b < model_->ngeom; ++b) {
        if (a == b || (IsArm(b) && b < a) || Adjacent(a, b)) continue;
        const double dx = data_->geom_xpos[3*a] - data_->geom_xpos[3*b];
        const double dy = data_->geom_xpos[3*a+1] - data_->geom_xpos[3*b+1];
        const double dz = data_->geom_xpos[3*a+2] - data_->geom_xpos[3*b+2];
        const bool self_pair = IsArm(b);
        // Bounding spheres give a conservative lower bound on separation.
        // Any pair able to violate margin_ is inside this 10 mm proximity
        // window; report near self pairs at every 100 Hz sample.
        const double bound = model_->geom_rbound[a] + model_->geom_rbound[b]
                           + std::max(0.01, margin_);
        if (dx*dx + dy*dy + dz*dz > bound*bound) continue;
        mjtNum fromto[6] = {};
        const double distance = mj_geomDistance(model_.get(), data_.get(), a, b, 1.0, fromto);
        if (self_pair) {
          auto& record = self_pairs_[{a, b}];
          if (distance < record.distance) { record.distance = distance; record.sample = sample; }
          if (distance < self_minimum_) {
            self_minimum_ = distance;
            self_closest_a_ = a; self_closest_b_ = b; self_closest_sample_ = sample;
          }
        }
        if (self_pair && monitor_self_collisions_) continue;
        if (distance < minimum) {
          minimum = distance; closest_a = a; closest_b = b;
          closest_sample = sample; closest_kind = ClassOf(b);
        }
        if (distance < margin_) {
          const std::string kind = ClassOf(b);
          Reply(id, false, kind == "FLOOR" ? "FLOOR_CLEARANCE_REJECTED" : kind + "_COLLISION_REJECTED",
                distance, a, b, sample, kind); return;
        }
      }
    }
    Reply(id, true, "CLEAR", closest_a < 0 ? -1.0 : minimum,
          closest_a, closest_b, closest_sample, closest_kind);
  }
  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  int free_qpos_ = -1;
  int mount_body_id_ = -1, first_body_id_ = -1;
  double margin_ = 0.01;
  bool monitor_self_collisions_ = false;
  struct SelfPairRecord {
    double distance = std::numeric_limits<double>::infinity();
    int sample = -1;
  };
  double self_minimum_ = std::numeric_limits<double>::infinity();
  int self_closest_a_ = -1, self_closest_b_ = -1, self_closest_sample_ = -1;
  std::map<std::pair<int, int>, SelfPairRecord> self_pairs_;
  std::chrono::steady_clock::time_point check_started_at_ = std::chrono::steady_clock::now();
  std::vector<int> arm_qpos_, arm_geoms_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  rclcpp::Time odom_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time motor_at_{0, 0, RCL_ROS_TIME};
  std::unordered_map<std::string, double> motor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr request_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
};
}  // namespace
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<ArmShadowChecker>()); }
  catch (const std::exception& error) { fprintf(stderr, "%s\n", error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
