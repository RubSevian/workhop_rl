// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <array>
#include <vector>
#include <limits>
#include <atomic>
#include <sstream>

#include <filesystem>

#include <mujoco/mujoco.h>
#include "mujoco/glfw_adapter.h"
#include "mujoco/simulate.h"
#include "mujoco/array_safety.h"
#include "unitree_sdk2_bridge/unitree_sdk2_bridge.h"
#include <pthread.h>
#include "yaml-cpp/yaml.h"
#include "rars01_arm_sim_gains.hpp"
#include "arm_release_hold.hpp"
#include "virtual_payload_bridge.hpp"
#include "go2_torque_hud.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/set_bool.hpp>

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;
  std::atomic<bool> mujoco_ready{false};

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  struct SimulationConfig
  {
    std::string robot = "go2";
    std::string robot_scene = "scene_terrain.xml";
    std::string base_body = "base_link";

    int domain_id = 1;
    std::string interface = "lo";
    int enable_unitree_bridge = 1;
    int enable_ros_bridge = 0;

    int use_joystick = 0;
    std::string joystick_type = "xbox";
    std::string joystick_device = "/dev/input/js0";
    int joystick_bits = 16;

    int print_scene_information = 1;

    std::string odom_topic = "/state_estimation";
    std::string world_frame = "map";

    double lidar_rate = 10.0;
    int lidar_horizontal_samples = 360;
    int lidar_vertical_lines = 18;
    double lidar_min_range = 0.5;
    double lidar_max_range = 100.0;
    // Do not initialize the estimator while the robot is still settling onto
    // its nominal stance.  This is simulation time, not wall-clock time.
    double sensor_start_delay = 12.0;

    int enable_elastic_band = 0;
    int band_attached_link = 0;

    // Optional Stage4D-only mouse goal control.  Disabled for every generic
    // simulation config unless explicitly enabled below.
    int enable_interactive_goal_click = 0;
    double goal_click_drag_threshold_px = 5.0;
    std::vector<std::string> goal_click_walkable_geoms = {"floor"};
    int enable_manual_manip_target = 0;
    int enable_mujoco_hud = 1;
    int allow_manual_manip_during_navigation = 0;

  } config;

  rars01_sim::ArmGains rars01_arm_gains;

  builtin_interfaces::msg::Time SimStamp(mjtNum seconds) {
    const int64_t total_ns = static_cast<int64_t>(seconds * 1.0e9);
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(total_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
    return stamp;
  }

  class MujocoClockPublisher {
  public:
    MujocoClockPublisher() : node_(std::make_shared<rclcpp::Node>("mujoco_clock")) {
      pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    }
    void Publish(const mjData* data) {
      rosgraph_msgs::msg::Clock msg;
      msg.clock = SimStamp(data->time);
      pub_->publish(msg);
    }
  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  };
  std::unique_ptr<MujocoClockPublisher> mujoco_clock;
  std::unique_ptr<MujocoVirtualPayloadBridge> virtual_payload_bridge;

  // Stage4D application bridge layered on top of the generic viewer click
  // event.  It is intentionally the only component that knows ROS topics.
  class MujocoNavigationGoalBridge {
  public:
    explicit MujocoNavigationGoalBridge(const SimulationConfig& config)
        : world_frame_(config.world_frame), walkable_geoms_(config.goal_click_walkable_geoms),
          node_(std::make_shared<rclcpp::Node>("mujoco_click_goal_bridge")) {
      goal_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 10);
      cancel_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/navigation_cancel", 10);
      diagnostic_pub_ = node_->create_publisher<std_msgs::msg::String>("/mujoco/click_goal_diagnostics", 10);
      navigation_active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/navigation_active", rclcpp::QoS(1).transient_local(),
          [this](const std_msgs::msg::Bool::SharedPtr msg) {
            navigation_active_.store(msg->data);
            if (!msg->data) {
              std::lock_guard<std::mutex> lock(marker_mutex_);
              marker_active_ = false;
            }
          });
      spin_thread_ = std::thread([this] { rclcpp::spin(node_); });
    }

    ~MujocoNavigationGoalBridge() {
      if (spin_thread_.joinable()) spin_thread_.join();
    }

    void HandleSceneClick(const mj::Simulate::SceneClickEvent& event) {
      if (event.action == mj::Simulate::SceneClickEvent::Action::kPrimaryClick) {
        CancelNavigation();
        return;
      }
      if (event.action != mj::Simulate::SceneClickEvent::Action::kCtrlPrimaryClick) return;

      if (!event.hit) {
        Reject("NO_SCENE_HIT", "<background>");
        return;
      }
      if (std::find(walkable_geoms_.begin(), walkable_geoms_.end(), event.geom_name) ==
          walkable_geoms_.end()) {
        Reject("NON_WALKABLE_GEOM", event.geom_name.empty() ? "<unnamed>" : event.geom_name);
        return;
      }

      geometry_msgs::msg::PointStamped goal;
      goal.header.stamp = node_->now();
      goal.header.frame_id = world_frame_;
      goal.point.x = event.world[0];
      goal.point.y = event.world[1];
      goal.point.z = event.world[2];
      goal_pub_->publish(goal);
      navigation_active_.store(true);
      {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        marker_active_ = true;
        marker_point_ = event.world;
      }
      PublishDiagnostic("SET", true, event.world, event.geom_name);
      RCLCPP_INFO(node_->get_logger(), "[CLICK_GOAL][SET] frame=%s point=(%.3f, %.3f, %.3f) geom=%s",
                  world_frame_.c_str(), event.world[0], event.world[1], event.world[2],
                  event.geom_name.c_str());
    }

    void DrawMarker(mjvScene& scene) const {
      std::array<double, 3> point;
      {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        if (!marker_active_) return;
        point = marker_point_;
      }
      if (scene.ngeom >= scene.maxgeom) return;
      mjtNum size[3] = {0.10, 0.0, 0.0};
      mjtNum pos[3] = {point[0], point[1], point[2] + 0.05};
      mjtNum mat[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      float rgba[4] = {1.0F, 0.82F, 0.08F, 1.0F};  // yellow: distinct from FAR/ground-truth paths
      mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, size, pos, mat, rgba);
    }

  private:
    void CancelNavigation() {
      if (!navigation_active_.exchange(false)) {
        RCLCPP_DEBUG(node_->get_logger(), "[CLICK_GOAL][CANCEL] ignored: navigation is already inactive");
        return;
      }
      std_msgs::msg::Empty cancel;
      cancel_pub_->publish(cancel);
      {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        marker_active_ = false;
      }
      PublishDiagnostic("CANCEL", false, {0.0, 0.0, 0.0}, "");
      RCLCPP_INFO(node_->get_logger(), "[CLICK_GOAL][CANCEL] navigation cancelled by plain left click");
    }

    void Reject(const char* reason, const std::string& geom) {
      PublishDiagnostic(reason, navigation_active_.load(), {0.0, 0.0, 0.0}, geom);
      RCLCPP_WARN(node_->get_logger(), "[CLICK_GOAL][REJECT] reason=%s geom=%s", reason, geom.c_str());
    }

    void PublishDiagnostic(const std::string& action, bool active,
                           const std::array<double, 3>& point, const std::string& geom) {
      std_msgs::msg::String message;
      std::ostringstream stream;
      stream << "{\"active\":" << (active ? "true" : "false")
             << ",\"last_action\":\"" << action << "\""
             << ",\"x\":" << point[0] << ",\"y\":" << point[1]
             << ",\"z\":" << point[2];
      if (!geom.empty()) stream << ",\"geom\":\"" << geom << "\"";
      stream << "}";
      message.data = stream.str();
      diagnostic_pub_->publish(message);
    }

    std::string world_frame_;
    std::vector<std::string> walkable_geoms_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_active_sub_;
    std::thread spin_thread_;
    std::atomic<bool> navigation_active_{false};
    mutable std::mutex marker_mutex_;
    bool marker_active_ = false;
    std::array<double, 3> marker_point_ = {0.0, 0.0, 0.0};
  };

  // M0.1 manual UI is a transport-only bridge: it publishes a picked world
  // point and renders status supplied by the manipulation state machine.
  class MujocoManualManipTargetBridge {
  public:
    explicit MujocoManualManipTargetBridge(const SimulationConfig& config)
        : world_frame_(config.world_frame), hud_enabled_(config.enable_mujoco_hud != 0),
          allow_during_navigation_(config.allow_manual_manip_during_navigation != 0),
          target_radius_m_([] { const char* value = std::getenv("STAGE4D_MANUAL_TARGET_DIAMETER_M");
            const double diameter = value ? std::atof(value) : 0.10;
            return std::isfinite(diameter) && diameter > 0.0 ? diameter * 0.5 : 0.05; }()),
          node_(std::make_shared<rclcpp::Node>("mujoco_manual_manip_target_bridge")) {
      target_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/stage4d/manual_manip_target", 10);
      cancel_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
          "/stage4d/manual_manip_cancel", 10);
      navigation_active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/navigation_active", rclcpp::QoS(1).transient_local(),
          [this](const std_msgs::msg::Bool::SharedPtr msg) { navigation_active_.store(msg->data); });
      manual_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
          "/stage4d/manual_manip_status", 10,
          [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            manual_status_ = msg->data;
          });
      spin_thread_ = std::thread([this] { rclcpp::spin(node_); });
    }

    ~MujocoManualManipTargetBridge() {
      if (spin_thread_.joinable()) spin_thread_.join();
    }

    void HandleSceneClick(const mj::Simulate::SceneClickEvent& event) {
      if (event.action == mj::Simulate::SceneClickEvent::Action::kCancelManualTarget) {
        Cancel();
        return;
      }
      if (event.action != mj::Simulate::SceneClickEvent::Action::kAltPrimaryClick) return;
      if (navigation_active_.load() && !allow_during_navigation_) {
        SetMessage("MANUAL TARGET REJECTED — NAV ACTIVE");
        return;
      }
      if (!event.hit) {
        SetMessage("MANUAL TARGET REJECTED — NO SCENE HIT");
        return;
      }
      // body 0 is world terrain; named stage4d_landmarks are static obstacles.
      // Every Go2/RARS01 body has another id, so clicks on robot links reject.
      if (event.body_id != 0 && event.body_name != "stage4d_landmarks") {
        SetMessage("MANUAL TARGET REJECTED — ROBOT/LINK");
        return;
      }
      const std::array<double, 3> center = {event.world[0], event.world[1], event.world[2] + target_radius_m_};
      geometry_msgs::msg::PoseStamped target;
      target.header.stamp = node_->now();
      target.header.frame_id = world_frame_;
      target.pose.position.x = center[0];
      target.pose.position.y = center[1];
      target.pose.position.z = center[2];
      target.pose.orientation.w = 1.0;
      target_pub_->publish(target);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target_active_ = true;
        target_world_ = center;
        manual_status_ = "MANUAL_TARGET_SET; IK=PENDING";
      }
      RCLCPP_INFO(node_->get_logger(), "[MANUAL_TARGET][SET] world=(%.3f, %.3f, %.3f) geom=%s",
                  center[0], center[1], center[2], event.geom_name.c_str());
    }

    void DrawMarker(mjvScene& scene) const {
      std::array<double, 3> target;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!target_active_) return;
        target = target_world_;
      }
      if (scene.ngeom >= scene.maxgeom) return;
      const mjtNum size[3] = {target_radius_m_, 0.0, 0.0};
      const mjtNum pos[3] = {target[0], target[1], target[2]};
      const mjtNum mat[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      const float rgba[4] = {0.05F, 0.95F, 0.85F, 1.0F};
      mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, size, pos, mat, rgba);
    }

    void DrawHud(const mjrRect& rect, mjrContext& context) const {
      if (!hud_enabled_) return;
      std::string status;
      std::array<double, 3> target;
      bool active = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status = manual_status_;
        target = target_world_;
        active = target_active_;
      }
      const char* profile = std::getenv("STAGE4D_PROFILE");
      std::ostringstream left, right;
      // Keep diagnostics inside the HUD instead of one clipped horizontal line.
      for (std::size_t index = 0; (index = status.find("; ", index)) != std::string::npos;) {
        status.replace(index, 2, "\n");
        ++index;
      }
      left << "STAGE4D M0.1\n"
           << "NAV: " << (navigation_active_.load() ? "ACTIVE" : "IDLE / SETTLED") << "\n"
           << "PROFILE: " << (profile ? profile : "baseline") << "\n"
           << "MANIP: " << (status.empty() ? "MANUAL_IDLE" : status) << "\n";
      if (active) left << "TARGET WORLD: " << std::fixed << std::setprecision(2)
                       << target[0] << " " << target[1] << " " << target[2] << "\n";
      const char* oa_exact = std::getenv("STAGE4D_ENABLE_ORIENTATION_AWARE_CHECK");
      const char* narrow_selection = std::getenv("STAGE4D_ENABLE_NARROW_RECOVERED_SELECTION");
      right << "searchRadius: " << (std::getenv("STAGE4D_SEARCH_RADIUS") ? std::getenv("STAGE4D_SEARCH_RADIUS") : "0.55") << "\n"
            << "collision: BROAD\n"
            << "OA exact: " << (oa_exact ? oa_exact : "false") << "; narrow: "
            << (narrow_selection ? narrow_selection : "false") << "\n"
            << "Ctrl+LMB: yellow nav goal; LMB: cancel nav\n"
            << "LAlt+LMB: cyan arm target (+5 cm)\n"
            << "RAlt: cancel arm + HOME";
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, left.str().c_str(), right.str().c_str(), &context);
    }

  private:
    void Cancel() {
      cancel_pub_->publish(std_msgs::msg::Empty());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target_active_ = false;
        manual_status_ = "CANCEL -> RETURN HOME";
      }
      RCLCPP_INFO(node_->get_logger(), "[MANUAL_TARGET][CANCEL] requested safe return home");
    }

    void SetMessage(const std::string& message) {
      std::lock_guard<std::mutex> lock(mutex_);
      manual_status_ = message;
      RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    }

    std::string world_frame_;
    bool hud_enabled_ = true;
    double target_radius_m_ = 0.05;
    bool allow_during_navigation_ = false;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_active_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_status_sub_;
    std::thread spin_thread_;
    std::atomic<bool> navigation_active_{false};
    mutable std::mutex mutex_;
    bool target_active_ = false;
    std::array<double, 3> target_world_ = {0.0, 0.0, 0.0};
    std::string manual_status_ = "MANUAL_IDLE";
  };

  // Stage-4 uses the native ROS message transport, not SDK2's private DDS
  // channel.  It prevents the SDK2 allocator crash on localhost and exactly
  // matches the topics used by the unified policy node.
  class MujocoRosLowLevelBridge {
  public:
    MujocoRosLowLevelBridge()
        : node_(std::make_shared<rclcpp::Node>("mujoco_ros_low_level_bridge")) {
      lowstate_pub_ = node_->create_publisher<unitree_go::msg::LowState>("lowstate", 10);
      physics_dt_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
          "/mujoco/physics_dt", rclcpp::QoS(1).transient_local());
      rl_ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/stage4d/rl_ready", rclcpp::QoS(1).transient_local(),
          [this](std_msgs::msg::Bool::SharedPtr msg) { rl_ready_.store(msg->data); });
      navigation_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/navigation_active", rclcpp::QoS(1).transient_local(),
          [this](std_msgs::msg::Bool::SharedPtr msg) { navigation_active_.store(msg->data); });
      lowcmd_sub_ = node_->create_subscription<unitree_go::msg::LowCmd>(
          "lowcmd", 10,
          [this](const unitree_go::msg::LowCmd::SharedPtr msg) {
            for (int i = 0; i < 20; ++i) {
              const auto& motor = msg->motor_cmd[i];
              if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
                  !std::isfinite(motor.kp) || !std::isfinite(motor.kd) ||
                  !std::isfinite(motor.tau)) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "Ignoring non-finite /lowcmd generation");
                return;
              }
            }
            std::lock_guard<std::mutex> lock(command_mutex_);
            latest_command_ = *msg;
            have_command_ = true;
            last_command_time_ = std::chrono::steady_clock::now();
            ++command_generation_;
          });
      spin_thread_ = std::thread([this] { rclcpp::spin(node_); });
    }

    ~MujocoRosLowLevelBridge() {
      if (spin_thread_.joinable()) spin_thread_.join();
    }

    // Called by the physics owner immediately before every mj_step.  The policy
    // holds q_des at 50 Hz; this method recomputes tau from the *current* q,dq
    // at the 500 Hz physics rate, rather than holding a stale torque.
    void Apply(const mjModel* model, mjData* data) {
      if (!Resolve(model)) return;
      unitree_go::msg::LowCmd command;
      bool timed_out = false;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        timed_out = !have_command_ ||
            (std::chrono::steady_clock::now() - last_command_time_ > std::chrono::milliseconds(100));
        if (!timed_out) command = latest_command_;
      }
      if (timed_out) command = SafeStandingCommand();
      command_timed_out_ = timed_out;
      // Arm PD is evaluated inside mjcb_control; legs are updated here.
      for (int i = 0; i < 12; ++i) {
        const int actuator = leg_actuator_ids_[i];
        const auto& motor = command.motor_cmd[i];
        const double raw = motor.tau + motor.kp * (motor.q - data->sensordata[leg_pos_adr_[i]]) +
            motor.kd * (motor.dq - data->sensordata[leg_vel_adr_[i]]);
        const double limit = (i % 3 == 2) ? 35.55 : 23.7;
        requested_torque_[i] = raw;
        data->ctrl[actuator] = std::clamp(raw, -limit, limit);
        if (raw != data->ctrl[actuator]) ++torque_saturation_count_;
      }
    }

    // This is called by MuJoCo at 500 Hz.  A complete recent command owns
    // the arm; otherwise the existing home hold remains active.
    bool ApplyArm(const mjModel* model, mjData* data) {
      if (!Resolve(model)) return false;
      if (arm_model_ != model || data->time < last_arm_time_) {
        arm_release_hold_.Reset();
        std::lock_guard<std::mutex> lock(command_mutex_);
        have_command_ = false;
      }
      arm_model_ = model;
      last_arm_time_ = data->time;
      unitree_go::msg::LowCmd command;
      bool fresh_command = false;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        fresh_command = have_command_ &&
            std::chrono::steady_clock::now() - last_command_time_ <= std::chrono::milliseconds(100);
        if (fresh_command) command = latest_command_;
      }
      if (!fresh_command) return HoldReleasedArm(model, data);
      for (int i = 0; i < 8; ++i) {
        const auto& motor = command.motor_cmd[12 + i];
        const int joint = rars_joint_ids_[i];
        if (motor.kp <= 0.0F || motor.kd < 0.0F ||
            motor.q < model->jnt_range[2 * joint] ||
            motor.q > model->jnt_range[2 * joint + 1]) {
          return HoldReleasedArm(model, data);
        }
      }
      arm_release_hold_.CommandAccepted();
      for (int i = 0; i < 8; ++i) {
        const int joint = rars_joint_ids_[i];
        const int actuator = rars_actuator_ids_[i];
        const auto& motor = command.motor_cmd[12 + i];
        const double q = data->qpos[model->jnt_qposadr[joint]];
        const double dq = data->qvel[model->jnt_dofadr[joint]];
        const double gravity_and_coriolis = rars01_arm_gains.enable_bias_compensation
            ? data->qfrc_bias[model->jnt_dofadr[joint]] : 0.0;
        const double torque = motor.tau + gravity_and_coriolis +
                              motor.kp * (motor.q - q) +
                              motor.kd * (motor.dq - dq);
        data->ctrl[actuator] = std::clamp(
            torque, model->actuator_ctrlrange[2 * actuator],
            model->actuator_ctrlrange[2 * actuator + 1]);
      }
      return true;
    }

    void Publish(const mjModel* model, const mjData* data) {
      if (!Resolve(model)) return;
      unitree_go::msg::LowState state;
      state.tick = static_cast<uint32_t>(++physics_tick_);
      for (int i = 0; i < 12; ++i) {
        state.motor_state[i].q = data->sensordata[leg_pos_adr_[i]];
        state.motor_state[i].dq = data->sensordata[leg_vel_adr_[i]];
        state.motor_state[i].tau_est = data->sensordata[leg_force_adr_[i]];
      }
      for (int i = 0; i < 8; ++i) {
        const int joint = rars_joint_ids_[i];
        const int slot = 12 + i;
        state.motor_state[slot].q = data->qpos[model->jnt_qposadr[joint]];
        state.motor_state[slot].dq = data->qvel[model->jnt_dofadr[joint]];
        state.motor_state[slot].tau_est = data->qfrc_actuator[model->jnt_dofadr[joint]];
      }
      for (int i = 0; i < 4; ++i) state.imu_state.quaternion[i] = data->sensordata[imu_quat_adr_ + i];
      for (int i = 0; i < 3; ++i) {
        state.imu_state.gyroscope[i] = data->sensordata[imu_gyro_adr_ + i];
        state.imu_state.accelerometer[i] = data->sensordata[imu_acc_adr_ + i];
      }
      const double w = state.imu_state.quaternion[0];
      const double x = state.imu_state.quaternion[1];
      const double y = state.imu_state.quaternion[2];
      const double z = state.imu_state.quaternion[3];
      state.imu_state.rpy[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
      state.imu_state.rpy[1] = std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0));
      state.imu_state.rpy[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
      lowstate_pub_->publish(state);
      // Formatting and UI transfer at 20 Hz; no per-step console/file output.
      if (data->time < last_hud_time_ || data->time + 1e-9 >= next_hud_time_) {
        next_hud_time_ = data->time + 0.05;
        const std::string snapshot = stage4d::Go2TorqueHud(model, data, requested_torque_,
            command_timed_out_ ? "SAFE_TIMEOUT" : rl_ready_.load() ? "RL" : "PRE-RL",
            navigation_active_.load());
        std::lock_guard<std::mutex> lock(hud_mutex_);
        hud_text_ = snapshot;
      }
      last_hud_time_ = data->time;
    }

    void DrawHud(const mjrRect& rect, mjrContext& context) const {
      std::string text;
      { std::lock_guard<std::mutex> lock(hud_mutex_); text = hud_text_; }
      if (!text.empty()) mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect,
                                    text.c_str(), "", &context);
    }

  private:
    bool HoldReleasedArm(const mjModel* model, mjData* data) {
      std::array<double, 8> measured{};
      for (int i = 0; i < 8; ++i)
        measured[i] = data->qpos[model->jnt_qposadr[rars_joint_ids_[i]]];
      const bool already_holding = arm_release_hold_.holding();
      if (!arm_release_hold_.Capture(measured)) return false;
      if (!already_holding)
        RCLCPP_WARN(node_->get_logger(),
                    "Arm command released/invalid: holding measured pose; automatic HOME disabled");
      for (int i = 0; i < 8; ++i) {
        const int joint = rars_joint_ids_[i], actuator = rars_actuator_ids_[i];
        const int dof = model->jnt_dofadr[joint];
        const double kp = i < 6 ? rars01_arm_gains.position_kp[i] : rars01_sim::kGripperKp;
        const double kd = i < 6 ? rars01_arm_gains.position_kd[i] : rars01_sim::kGripperKd;
        const double bias = rars01_arm_gains.enable_bias_compensation ? data->qfrc_bias[dof] : 0.0;
        const double torque = kp * (arm_release_hold_.target()[i] - measured[i]) -
                              kd * data->qvel[dof] + bias;
        data->ctrl[actuator] = std::clamp(torque, model->actuator_ctrlrange[2*actuator],
                                        model->actuator_ctrlrange[2*actuator+1]);
      }
      return true;
    }

    static int SensorAddress(const mjModel* model, const char* name, int dimension) {
      const int sensor = mj_name2id(model, mjOBJ_SENSOR, name);
      if (sensor < 0 || model->sensor_dim[sensor] != dimension) {
        throw std::runtime_error(std::string("Missing or invalid MuJoCo sensor: ") + name);
      }
      return model->sensor_adr[sensor];
    }

    static unitree_go::msg::LowCmd SafeStandingCommand() {
      unitree_go::msg::LowCmd command;
      // FR, FL, RR, RL; a bounded pose avoids a limp free-fall after /lowcmd
      // disappears, while kd damps residual motion.
      constexpr std::array<float, 12> kSafeQ = {
          -0.1F, 0.8F, -1.5F, 0.1F, 0.8F, -1.5F,
          -0.1F, 0.8F, -1.5F, 0.1F, 0.8F, -1.5F};
      for (int i = 0; i < 12; ++i) {
        command.motor_cmd[i].q = kSafeQ[i];
        command.motor_cmd[i].dq = 0.0F;
        command.motor_cmd[i].kp = 30.0F;
        command.motor_cmd[i].kd = 2.0F;
        command.motor_cmd[i].tau = 0.0F;
      }
      return command;
    }

    bool Resolve(const mjModel* model) {
      if (resolved_model_ == model) return ready_;
      resolved_model_ = model;
      ready_ = false;
      const char* actuator_names[] = {"FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
                                      "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};
      const char* joints[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                              "gripper_left_joint", "gripper_right_joint"};
      const char* arm_actuators[] = {
          "joint1_motor", "joint2_motor", "joint3_motor", "joint4_motor",
          "joint5_motor", "joint6_motor", "gripper_left_motor",
          "gripper_right_motor"};
      try {
        for (int i = 0; i < 12; ++i) {
          leg_actuator_ids_[i] = mj_name2id(model, mjOBJ_ACTUATOR, actuator_names[i]);
          if (leg_actuator_ids_[i] < 0) throw std::runtime_error("Missing leg actuator");
          const std::string prefix(actuator_names[i]);
          leg_pos_adr_[i] = SensorAddress(model, (prefix + "_pos").c_str(), 1);
          leg_vel_adr_[i] = SensorAddress(model, (prefix + "_vel").c_str(), 1);
          leg_force_adr_[i] = SensorAddress(model, (prefix + "_torque").c_str(), 1);
        }
        for (int i = 0; i < 8; ++i) {
          rars_joint_ids_[i] = mj_name2id(model, mjOBJ_JOINT, joints[i]);
          rars_actuator_ids_[i] = mj_name2id(model, mjOBJ_ACTUATOR, arm_actuators[i]);
          if (rars_joint_ids_[i] < 0 || rars_actuator_ids_[i] != 12 + i)
            throw std::runtime_error("Missing or reordered RARS01 joint/actuator");
        }
        imu_quat_adr_ = SensorAddress(model, "imu_quat", 4);
        imu_gyro_adr_ = SensorAddress(model, "imu_gyro", 3);
        imu_acc_adr_ = SensorAddress(model, "imu_acc", 3);
        ready_ = true;
        std_msgs::msg::Float64 physics_dt;
        physics_dt.data = model->opt.timestep;
        physics_dt_pub_->publish(physics_dt);
        RCLCPP_INFO(node_->get_logger(), "ROS low-level bridge ready: /lowcmd -> 12 Go2 legs + 8 RARS01 joints, /lowstate <- 20 motors");
      } catch (const std::exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "ROS low-level bridge disabled: %s", error.what());
      }
      return ready_;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr physics_dt_pub_;
    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr lowcmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rl_ready_sub_, navigation_sub_;
    std::atomic<bool> rl_ready_{false}, navigation_active_{false};
    std::array<double, 12> requested_torque_{};
    bool command_timed_out_ = true;
    double next_hud_time_ = 0, last_hud_time_ = -1;
    mutable std::mutex hud_mutex_;
    std::string hud_text_;
    std::thread spin_thread_;
    std::mutex command_mutex_;
    unitree_go::msg::LowCmd latest_command_;
    bool have_command_ = false;
    std::chrono::steady_clock::time_point last_command_time_{};
    std::atomic<uint64_t> command_generation_{0};
    std::atomic<uint64_t> torque_saturation_count_{0};
    uint64_t physics_tick_ = 0;
    const mjModel* resolved_model_ = nullptr;
    bool ready_ = false;
    std::array<int, 12> leg_actuator_ids_{};
    std::array<int, 12> leg_pos_adr_{};
    std::array<int, 12> leg_vel_adr_{};
    std::array<int, 12> leg_force_adr_{};
    std::array<int, 8> rars_joint_ids_{};
    std::array<int, 8> rars_actuator_ids_{};
    rars01_sim::ArmReleaseHold arm_release_hold_;
    const mjModel* arm_model_ = nullptr;
    double last_arm_time_ = -1.0;
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
  };

  std::unique_ptr<MujocoRosLowLevelBridge> ros_low_level_bridge;

  // Publishes only while the physics thread owns the MuJoCo lock, so the pose
  // and velocity describe one consistent physics state.  Twist is explicitly
  // body-frame: mj_objectVelocity(..., flg_local=1) returns [angular, linear].
  class MujocoGroundTruthOdom {
  public:
    MujocoGroundTruthOdom()
        : node_(std::make_shared<rclcpp::Node>("mujoco_ground_truth_odom")) {
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(config.odom_topic, 10);
      arm_base_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/rars_base_pose", 10);
    }

    void Publish(const mjModel* model, const mjData* data) {
      if (!Resolve(model)) return;
      // Reset only on an actual simulation-time rollback.  Ordinary 2 ms
      // physics ticks must remain rate-limited to 50 Hz for planner odometry.
      if (data->time + 1.0e-9 < last_sim_time_) next_publish_time_ = data->time;
      last_sim_time_ = data->time;
      if (data->time + 1.0e-9 < next_publish_time_) return;
      next_publish_time_ = data->time + 0.02;  // 50 Hz, 10 x 0.002 s physics ticks

      const int qpos_adr = model->jnt_qposadr[freejoint_id_];
      const auto finite = [](mjtNum value) { return std::isfinite(static_cast<double>(value)); };
      for (int i = 0; i < 7; ++i) {
        if (!finite(data->qpos[qpos_adr + i])) return;
      }

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = SimStamp(data->time);
      odom.header.frame_id = config.world_frame;
      odom.child_frame_id = config.base_body;
      odom.pose.pose.position.x = data->qpos[qpos_adr + 0];
      odom.pose.pose.position.y = data->qpos[qpos_adr + 1];
      odom.pose.pose.position.z = data->qpos[qpos_adr + 2];
      // MuJoCo freejoint quaternion is w,x,y,z; ROS is x,y,z,w.
      odom.pose.pose.orientation.w = data->qpos[qpos_adr + 3];
      odom.pose.pose.orientation.x = data->qpos[qpos_adr + 4];
      odom.pose.pose.orientation.y = data->qpos[qpos_adr + 5];
      odom.pose.pose.orientation.z = data->qpos[qpos_adr + 6];

      std::array<mjtNum, 6> body_velocity{};
      mj_objectVelocity(model, data, mjOBJ_BODY, base_body_id_, body_velocity.data(), 1);
      odom.twist.twist.angular.x = body_velocity[0];
      odom.twist.twist.angular.y = body_velocity[1];
      odom.twist.twist.angular.z = body_velocity[2];
      odom.twist.twist.linear.x = body_velocity[3];
      odom.twist.twist.linear.y = body_velocity[4];
      odom.twist.twist.linear.z = body_velocity[5];
      odom_pub_->publish(odom);
      // Authoritative arm-base pose from the same MuJoCo state and timestamp.
      geometry_msgs::msg::PoseStamped arm_pose;
      arm_pose.header = odom.header;
      arm_pose.pose.position.x = data->xpos[3 * arm_base_body_id_];
      arm_pose.pose.position.y = data->xpos[3 * arm_base_body_id_ + 1];
      arm_pose.pose.position.z = data->xpos[3 * arm_base_body_id_ + 2];
      arm_pose.pose.orientation.w = data->xquat[4 * arm_base_body_id_];
      arm_pose.pose.orientation.x = data->xquat[4 * arm_base_body_id_ + 1];
      arm_pose.pose.orientation.y = data->xquat[4 * arm_base_body_id_ + 2];
      arm_pose.pose.orientation.z = data->xquat[4 * arm_base_body_id_ + 3];
      arm_base_pub_->publish(arm_pose);

    }

  private:
    bool Resolve(const mjModel* model) {
      if (resolved_model_ == model) return freejoint_id_ >= 0 && base_body_id_ >= 0 && arm_base_body_id_ >= 0;
      resolved_model_ = model;
      base_body_id_ = mj_name2id(model, mjOBJ_BODY, config.base_body.c_str());
      arm_base_body_id_ = mj_name2id(model, mjOBJ_BODY, "base_link");
      freejoint_id_ = mj_name2id(model, mjOBJ_JOINT, "base_freejoint");
      next_publish_time_ = 0.0;
      last_sim_time_ = -1.0e30;
      if (base_body_id_ < 0 || arm_base_body_id_ < 0 || freejoint_id_ < 0 || model->jnt_type[freejoint_id_] != mjJNT_FREE) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Cannot publish ground-truth odometry: need body '%s' and freejoint 'base_freejoint'",
                     config.base_body.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "Publishing MuJoCo ground truth: %s (%s -> %s, body-frame twist)",
                  config.odom_topic.c_str(), config.world_frame.c_str(), config.base_body.c_str());
      return true;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_base_pub_;
    int arm_base_body_id_ = -1;
    const mjModel* resolved_model_ = nullptr;
    int base_body_id_ = -1;
    int freejoint_id_ = -1;
    mjtNum next_publish_time_ = 0.0;
    mjtNum last_sim_time_ = -1.0e30;
  };

  std::unique_ptr<MujocoGroundTruthOdom> ground_truth_odom;

  // Publishes cumulative contact diagnostics at 50 Hz.  MuJoCo's world body
  // (body id 0) contains environment geometry; robot geometry is attached to
  // descendant bodies.  Floor/foot contacts are expected and excluded from
  // the obstacle counter, while robot/world contacts are reported explicitly.
  class MujocoCollisionDiagnostics {
  public:
    MujocoCollisionDiagnostics()
        : node_(std::make_shared<rclcpp::Node>("mujoco_collision_diagnostics")) {
      auto qos = rclcpp::QoS(1).transient_local();
      count_pub_ = node_->create_publisher<std_msgs::msg::Int64>(
          "/mujoco/non_floor_contact_count", qos);
      unexpected_floor_pub_ = node_->create_publisher<std_msgs::msg::Int64>(
          "/mujoco/unexpected_floor_contact_count", qos);
      diagnostics_pub_ = node_->create_publisher<std_msgs::msg::String>(
          "/mujoco/collision_diagnostics", qos);
    }

    void Observe(const mjModel* model, const mjData* data) {
      if (!Resolve(model)) return;
      bool environment_contact = false;
      for (int i = 0; i < data->ncon; ++i) {
        const int geom1 = data->contact[i].geom1;
        const int geom2 = data->contact[i].geom2;
        const bool floor1 = geom1 == floor_geom_id_;
        const bool floor2 = geom2 == floor_geom_id_;
        const bool robot1 = IsRobotGeom(model, geom1);
        const bool robot2 = IsRobotGeom(model, geom2);
        if (floor1 || floor2) {
          const int other = floor1 ? geom2 : geom1;
          if (IsRobotGeom(model, other) && !IsExpectedFoot(model, other)) {
            ++unexpected_floor_contact_count_;
            if (first_unexpected_floor_geoms_.empty()) {
              first_unexpected_floor_geoms_ = GeomName(model, other) + " <-> floor";
            }
          }
          continue;
        }
        if ((robot1 && !robot2) || (robot2 && !robot1)) {
          ++non_floor_contact_count_;
          environment_contact = true;
          if (first_contact_time_ < 0.0) {
            first_contact_time_ = data->time;
            first_contact_geoms_ = GeomName(model, geom1) + " <-> " + GeomName(model, geom2);
          }
        }
      }
      if (environment_contact) ++non_floor_contact_steps_;
      if (data->time + 1.0e-9 < next_publish_time_) return;
      next_publish_time_ = data->time + 0.02;
      std_msgs::msg::Int64 count;
      count.data = non_floor_contact_count_;
      count_pub_->publish(count);
      count.data = unexpected_floor_contact_count_;
      unexpected_floor_pub_->publish(count);
      std_msgs::msg::String diagnostics;
      std::ostringstream json;
      json << "{\"non_floor_environment_contact_count\":" << non_floor_contact_count_
           << ",\"non_floor_environment_contact_steps\":" << non_floor_contact_steps_
           << ",\"unexpected_robot_floor_contact_count\":" << unexpected_floor_contact_count_
           << ",\"first_non_floor_contact_time_s\":";
      if (first_contact_time_ < 0.0) json << "null";
      else json << first_contact_time_;
      json << ",\"first_non_floor_contact_geoms\":";
      if (first_contact_geoms_.empty()) json << "null";
      else json << "\"" << first_contact_geoms_ << "\"";
      json << ",\"first_unexpected_floor_contact_geoms\":";
      if (first_unexpected_floor_geoms_.empty()) json << "null";
      else json << "\"" << first_unexpected_floor_geoms_ << "\"";
      json << "}";
      diagnostics.data = json.str();
      diagnostics_pub_->publish(diagnostics);
    }

  private:
    bool Resolve(const mjModel* model) {
      if (resolved_model_ == model) return floor_geom_id_ >= 0;
      resolved_model_ = model;
      floor_geom_id_ = mj_name2id(model, mjOBJ_GEOM, "floor");
      next_publish_time_ = 0.0;
      return floor_geom_id_ >= 0;
    }

    static bool IsRobotGeom(const mjModel* model, int geom_id) {
      return geom_id >= 0 && geom_id < model->ngeom && model->geom_bodyid[geom_id] > 0;
    }

    static std::string GeomName(const mjModel* model, int geom_id) {
      const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
      const char* body = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom_id]);
      std::string result = name ? std::string(name) : ("geom_" + std::to_string(geom_id));
      return result + "@" + (body ? std::string(body) : "body_" + std::to_string(model->geom_bodyid[geom_id]));
    }

    static bool IsExpectedFoot(const mjModel* model, int geom_id) {
      const std::string geom = GeomName(model, geom_id);
      const int body_id = model->geom_bodyid[geom_id];
      const char* body_name = mj_id2name(model, mjOBJ_BODY, body_id);
      const std::string body = body_name ? std::string(body_name) : "";
      auto has_foot_token = [](const std::string& value) {
        return value.find("foot") != std::string::npos || value.find("toe") != std::string::npos;
      };
      return has_foot_token(geom) || has_foot_token(body);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr unexpected_floor_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    const mjModel* resolved_model_ = nullptr;
    int floor_geom_id_ = -1;
    mjtNum next_publish_time_ = 0.0;
    int64_t non_floor_contact_count_ = 0;
    int64_t non_floor_contact_steps_ = 0;
    int64_t unexpected_floor_contact_count_ = 0;
    mjtNum first_contact_time_ = -1.0;
    std::string first_contact_geoms_;
    std::string first_unexpected_floor_geoms_;
  };

  std::unique_ptr<MujocoCollisionDiagnostics> collision_diagnostics;

  // Minimal deterministic simulated UNILIDAR + IMU bridge.  It samples the
  // existing MuJoCo imu sensors and raycasts from the model's radar body; no
  // ground-truth pose is used to fabricate points or estimator output.
  class MujocoPointLioSensorBridge {
  public:
    explicit MujocoPointLioSensorBridge(const SimulationConfig& cfg)
        : node_(std::make_shared<rclcpp::Node>("mujoco_pointlio_sensor_bridge")),
          lidar_rate_(cfg.lidar_rate), horizontal_samples_(cfg.lidar_horizontal_samples),
          vertical_lines_(cfg.lidar_vertical_lines), min_range_(cfg.lidar_min_range),
          max_range_(cfg.lidar_max_range), sensor_start_delay_(cfg.sensor_start_delay) {
      imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/utlidar/imu", 20);
      cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/cloud", 5);
      diagnostics_pub_ = node_->create_publisher<std_msgs::msg::String>("/mujoco/lidar_diagnostics", 10);
      lidar_toggle_ = node_->create_service<std_srvs::srv::SetBool>(
          "/mujoco/lidar_enable", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            lidar_enabled_.store(req->data); res->success = true;
            res->message = req->data ? "simulated LiDAR enabled" : "simulated LiDAR disabled";
          });
      imu_toggle_ = node_->create_service<std_srvs::srv::SetBool>(
          "/mujoco/imu_enable", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            imu_enabled_.store(req->data); res->success = true;
            res->message = req->data ? "simulated IMU enabled" : "simulated IMU disabled";
          });
      spin_thread_ = std::thread([this] { rclcpp::spin(node_); });
    }

    ~MujocoPointLioSensorBridge() {
      if (spin_thread_.joinable()) spin_thread_.join();
    }

    void Publish(const mjModel* model, const mjData* data) {
      if (!Resolve(model)) return;
      if (data->time + 1.0e-9 < last_sim_time_) {
        next_lidar_time_ = data->time;
        next_imu_time_ = data->time;
      }
      last_sim_time_ = data->time;
      // The Go2 starts from the floor and spends its initial seconds settling
      // and entering the RL stance.  Feeding that transient to Point-LIO makes
      // its stationary gravity initialization invalid.
      if (data->time + 1.0e-9 < sensor_start_delay_) {
        scan_active_ = false;
        next_imu_time_ = sensor_start_delay_;
        next_lidar_time_ = sensor_start_delay_;
        return;
      }
      if (imu_enabled_.load() && data->time + 1.0e-9 >= next_imu_time_) {
        PublishImu(data);
        next_imu_time_ = data->time + 0.01;  // original UTLidar Point-LIO contract: 100 Hz
      }
      // A cloud is assembled over one scan period from physics-time raycasts,
      // so each point's time field is truthful rather than a synthetic zero.
      if (lidar_enabled_.load()) PublishLidar(model, data);
    }

  private:
    static int SensorAddress(const mjModel* model, const char* name, int dimension) {
      const int sensor = mj_name2id(model, mjOBJ_SENSOR, name);
      if (sensor < 0 || model->sensor_dim[sensor] != dimension) return -1;
      return model->sensor_adr[sensor];
    }
    static std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& a,
                                                      const std::array<double, 4>& b) {
      // MuJoCo sensor order is w,x,y,z.
      return {a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
              a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
              a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
              a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]};
    }
    static std::array<double, 3> BodyToRawVector(const std::array<double, 3>& body) {
      // Inverse of transform_everything.py: Ry(15.1 deg) * diag(1,-1,-1).
      constexpr double theta = 15.1 * M_PI / 180.0;
      const double x = std::cos(theta) * body[0] + std::sin(theta) * body[2];
      const double y = body[1];
      const double z = -std::sin(theta) * body[0] + std::cos(theta) * body[2];
      return {x, -y, -z};
    }
    bool Resolve(const mjModel* model) {
      if (resolved_model_ == model) return ready_;
      resolved_model_ = model;
      radar_body_id_ = mj_name2id(model, mjOBJ_BODY, "radar");
      floor_geom_id_ = mj_name2id(model, mjOBJ_GEOM, "floor");
      imu_quat_adr_ = SensorAddress(model, "imu_quat", 4);
      imu_gyro_adr_ = SensorAddress(model, "imu_gyro", 3);
      imu_acc_adr_ = SensorAddress(model, "imu_acc", 3);
      ready_ = radar_body_id_ >= 0 && floor_geom_id_ >= 0 && imu_quat_adr_ >= 0 &&
               imu_gyro_adr_ >= 0 && imu_acc_adr_ >= 0;
      next_lidar_time_ = sensor_start_delay_;
      next_imu_time_ = sensor_start_delay_;
      last_sim_time_ = -1.0e30;
      scan_active_ = false;
      if (ready_) {
        RCLCPP_INFO(node_->get_logger(),
                    "L1 emulation ready: /utlidar/imu=100Hz, /utlidar/cloud=%.1fHz, scan=%dx%d, start_delay=%.1fs, environment group=0",
                    lidar_rate_, vertical_lines_, horizontal_samples_, sensor_start_delay_);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "L1 emulation missing radar, floor, or IMU sensors");
      }
      return ready_;
    }
    void PublishImu(const mjData* data) {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = SimStamp(data->time);
      msg.header.frame_id = "utlidar_imu_1";
      // The original transformer applies mount R(0, 2.878202585, pi) to raw.
      // Publish its inverse here so its existing raw->body conversion happens exactly once.
      constexpr double pitch = 2.8782025850555556;
      constexpr double yaw = M_PI;
      const std::array<double, 4> mount = {std::cos(pitch/2)*std::cos(yaw/2),
                                            -std::sin(pitch/2)*std::sin(yaw/2),
                                             std::sin(pitch/2)*std::cos(yaw/2),
                                             std::cos(pitch/2)*std::sin(yaw/2)};
      const std::array<double, 4> inv_mount = {mount[0], -mount[1], -mount[2], -mount[3]};
      const std::array<double, 4> body_q = {data->sensordata[imu_quat_adr_], data->sensordata[imu_quat_adr_ + 1],
                                             data->sensordata[imu_quat_adr_ + 2], data->sensordata[imu_quat_adr_ + 3]};
      const auto raw_q = MultiplyQuaternion(inv_mount, body_q);
      msg.orientation.w = raw_q[0]; msg.orientation.x = raw_q[1];
      msg.orientation.y = raw_q[2]; msg.orientation.z = raw_q[3];
      const auto raw_w = BodyToRawVector({data->sensordata[imu_gyro_adr_], data->sensordata[imu_gyro_adr_ + 1], data->sensordata[imu_gyro_adr_ + 2]});
      const auto raw_a = BodyToRawVector({data->sensordata[imu_acc_adr_], data->sensordata[imu_acc_adr_ + 1], data->sensordata[imu_acc_adr_ + 2]});
      msg.angular_velocity.x = raw_w[0]; msg.angular_velocity.y = raw_w[1]; msg.angular_velocity.z = raw_w[2];
      msg.linear_acceleration.x = raw_a[0]; msg.linear_acceleration.y = raw_a[1]; msg.linear_acceleration.z = raw_a[2];
      imu_pub_->publish(msg);
    }
    struct LidarHit { float x, y, z, intensity, time; uint16_t ring; };

    void PublishLidar(const mjModel* model, const mjData* data) {
      const int rows = std::max(1, vertical_lines_);
      const int cols = std::max(1, horizontal_samples_);
      const int rays = rows * cols;
      const double scan_period = 1.0 / std::max(1.0, lidar_rate_);
      if (!scan_active_) {
        scan_active_ = true;
        scan_start_time_ = data->time;
        scan_next_ray_ = 0;
        scan_hits_.clear();
        scan_floor_hits_ = 0;
        scan_non_floor_hits_ = 0;
        scan_min_hit_ = std::numeric_limits<double>::infinity();
        scan_max_hit_ = 0.0;
      }

      // One azimuth column per instant: all vertical channels share that
      // instant.  This produces the exact Point-LIO `time` contract while
      // retaining actual MuJoCo poses/ray intersections for every slice.
      const double elapsed = std::max(0.0, static_cast<double>(data->time - scan_start_time_));
      const int target = std::min(rays, std::max(scan_next_ray_,
          static_cast<int>(std::floor((elapsed / scan_period) * rays + 1.0e-9))));
      const int count = target - scan_next_ray_;
      if (count > 0) {
        std::vector<mjtNum> world_dirs(3 * count);
        std::vector<std::array<float, 3>> local_dirs(count);
        std::vector<uint16_t> rings(count);
        std::vector<float> offsets(count);
        const mjtNum* rotation = data->xmat + 9 * radar_body_id_;
        for (int j = 0; j < count; ++j) {
          const int ray = scan_next_ray_ + j;
          const int col = ray / rows;
          const int row = ray % rows;
          const double vertical = (-15.0 + 30.0 * row / std::max(1, rows - 1)) * M_PI / 180.0;
          const double horizontal = 2.0 * M_PI * col / cols;
          const std::array<float, 3> local = {static_cast<float>(std::cos(vertical) * std::cos(horizontal)),
                                               static_cast<float>(std::cos(vertical) * std::sin(horizontal)),
                                               static_cast<float>(std::sin(vertical))};
          local_dirs[j] = local;
          rings[j] = static_cast<uint16_t>(row);
          offsets[j] = static_cast<float>(scan_period * col / cols);
          world_dirs[3*j+0] = rotation[0]*local[0] + rotation[1]*local[1] + rotation[2]*local[2];
          world_dirs[3*j+1] = rotation[3]*local[0] + rotation[4]*local[1] + rotation[5]*local[2];
          world_dirs[3*j+2] = rotation[6]*local[0] + rotation[7]*local[1] + rotation[8]*local[2];
        }
        std::array<mjtByte, mjNGROUP> environment_mask{};
        environment_mask[0] = 1;
        std::vector<int> geom_ids(count, -1);
        std::vector<mjtNum> distances(count, -1.0);
        const mjtNum* origin = data->xpos + 3 * radar_body_id_;
        mj_multiRay(model, const_cast<mjData*>(data), origin, world_dirs.data(), environment_mask.data(), 1,
                    -1, geom_ids.data(), distances.data(), count, static_cast<mjtNum>(max_range_));
        for (int j = 0; j < count; ++j) {
          const mjtNum distance = distances[j];
          if (geom_ids[j] < 0 || !std::isfinite(static_cast<double>(distance)) ||
              distance < min_range_ || distance > max_range_) continue;
          const auto& dir = local_dirs[j];
          scan_hits_.push_back({static_cast<float>(distance)*dir[0], static_cast<float>(distance)*dir[1],
                                static_cast<float>(distance)*dir[2], 1.0F, offsets[j], rings[j]});
          scan_min_hit_ = std::min(scan_min_hit_, static_cast<double>(distance));
          scan_max_hit_ = std::max(scan_max_hit_, static_cast<double>(distance));
          if (geom_ids[j] == floor_geom_id_) ++scan_floor_hits_; else ++scan_non_floor_hits_;
        }
        scan_next_ray_ = target;
      }
      if (scan_next_ray_ < rays) return;

      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.stamp = SimStamp(scan_start_time_);
      cloud.header.frame_id = "utlidar_lidar_1";
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(6, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, "time", 1, sensor_msgs::msg::PointField::FLOAT32,
          "ring", 1, sensor_msgs::msg::PointField::UINT16);
      modifier.resize(scan_hits_.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), intensity(cloud, "intensity"), time(cloud, "time");
      sensor_msgs::PointCloud2Iterator<uint16_t> ring(cloud, "ring");
      for (const auto& hit : scan_hits_) { *x=hit.x; *y=hit.y; *z=hit.z; *intensity=hit.intensity; *time=hit.time; *ring=hit.ring; ++x; ++y; ++z; ++intensity; ++time; ++ring; }
      cloud_pub_->publish(cloud);
      if (data->time + 1.0e-9 >= next_diagnostics_time_) {
        next_diagnostics_time_ = data->time + 1.0;
        std_msgs::msg::String diagnostic;
        std::ostringstream out;
        out << "{\"total_rays\":" << rays << ",\"valid_hits\":" << scan_hits_.size()
            << ",\"hit_ratio\":" << (static_cast<double>(scan_hits_.size())/rays)
            << ",\"min_valid_range\":" << (scan_hits_.empty() ? 0.0 : scan_min_hit_)
            << ",\"max_valid_range\":" << scan_max_hit_ << ",\"floor_hits\":" << scan_floor_hits_
            << ",\"non_floor_hits\":" << scan_non_floor_hits_ << "}";
        diagnostic.data = out.str();
        diagnostics_pub_->publish(diagnostic);
      }
      scan_active_ = false;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lidar_toggle_, imu_toggle_;
    std::thread spin_thread_;
    std::atomic<bool> lidar_enabled_{true}, imu_enabled_{true};
    const mjModel* resolved_model_ = nullptr;
    int radar_body_id_ = -1, floor_geom_id_ = -1, imu_quat_adr_ = -1, imu_gyro_adr_ = -1, imu_acc_adr_ = -1;
    bool ready_ = false;
    mjtNum next_lidar_time_ = 0.0, next_imu_time_ = 0.0, next_diagnostics_time_ = 0.0, last_sim_time_ = -1.0e30;
    double lidar_rate_; int horizontal_samples_, vertical_lines_; double min_range_, max_range_, sensor_start_delay_;
    bool scan_active_ = false;
    mjtNum scan_start_time_ = 0.0;
    int scan_next_ray_ = 0, scan_floor_hits_ = 0, scan_non_floor_hits_ = 0;
    double scan_min_hit_ = std::numeric_limits<double>::infinity(), scan_max_hit_ = 0.0;
    std::vector<LidarHit> scan_hits_;
  };

  std::unique_ptr<MujocoPointLioSensorBridge> pointlio_sensor_bridge;

  using Seconds = std::chrono::duration<double>;

  // The Unitree bridge owns ctrl[0:12]. This callback owns only the dedicated
  // arm/gripper actuators in the combined model at every MuJoCo physics step.
  void Go2Rars01HomeHold(const mjModel* model, mjData* data) {
    if (model->nu < 20) return;
    if (ros_low_level_bridge && ros_low_level_bridge->ApplyArm(model, data)) return;
    const char* joints[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_left_joint", "gripper_right_joint"};
    const char* actuators[] = {"joint1_motor", "joint2_motor", "joint3_motor", "joint4_motor", "joint5_motor", "joint6_motor", "gripper_left_motor", "gripper_right_motor"};
    // Arm gains come from the dedicated simulation YAML; gripper gains stay unchanged.
    for (int i = 0; i < 8; ++i) {
      int jid = mj_name2id(model, mjOBJ_JOINT, joints[i]);
      int aid = mj_name2id(model, mjOBJ_ACTUATOR, actuators[i]);
      if (jid < 0 || aid != 12 + i) return;
      const double kp = i < 6 ? rars01_arm_gains.position_kp[i] : rars01_sim::kGripperKp;
      const double kd = i < 6 ? rars01_arm_gains.position_kd[i] : rars01_sim::kGripperKd;
      double tau = -kp * data->qpos[model->jnt_qposadr[jid]] - kd * data->qvel[model->jnt_dofadr[jid]];
      const double lower = model->actuator_ctrlrange[2 * aid];
      const double upper = model->actuator_ctrlrange[2 * aid + 1];
      data->ctrl[aid] = std::max(lower, std::min(upper, tau));
    }
  }

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    try {
      if (virtual_payload_bridge) virtual_payload_bridge->Initialize(mnew);
    } catch (const std::exception& error) {
      std::snprintf(sim.load_error, sizeof(sim.load_error), "Payload initialization failed: %s", error.what());
      std::cerr << sim.load_error << std::endl;
      mj_deleteModel(mnew);
      return nullptr;
    }
    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // Servo ownership is explicit: legs are updated at every physics
              // tick before mj_step; arm/gripper remain in mjcb_control.
              if (virtual_payload_bridge) virtual_payload_bridge->Apply(m, d);
              if (ros_low_level_bridge) ros_low_level_bridge->Apply(m, d);
              // run single step, let next iteration deal with timing
              mj_step(m, d);
              if (mujoco_clock) mujoco_clock->Publish(d);
              if (ros_low_level_bridge) ros_low_level_bridge->Publish(m, d);
              if (ground_truth_odom) ground_truth_odom->Publish(m, d);
              if (collision_diagnostics) collision_diagnostics->Observe(m, d);
              if (pointlio_sensor_bridge) pointlio_sensor_bridge->Publish(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (sim.use_elastic_band_ == 1)
                {
                  if (sim.elastic_band_.enable_)
                  {
                    vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    sim.elastic_band_.Advance(x, dx);

                    d->xfrc_applied[config.band_attached_link] = sim.elastic_band_.f_[0];
                    d->xfrc_applied[config.band_attached_link + 1] = sim.elastic_band_.f_[1];
                    d->xfrc_applied[config.band_attached_link + 2] = sim.elastic_band_.f_[2];
                  }
                }

                // Recalculate leg PD from the latest held q_des before every
                // 0.002 s physics step; do not hold a 50 Hz torque.
                if (virtual_payload_bridge) virtual_payload_bridge->Apply(m, d);
                if (ros_low_level_bridge) ros_low_level_bridge->Apply(m, d);
                // call mj_step
                mj_step(m, d);
                if (mujoco_clock) mujoco_clock->Publish(d);
                if (ros_low_level_bridge) ros_low_level_bridge->Publish(m, d);
                if (ground_truth_odom) ground_truth_odom->Publish(m, d);
                if (collision_diagnostics) collision_diagnostics->Observe(m, d);
              if (pointlio_sensor_bridge) pointlio_sensor_bridge->Publish(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            if (virtual_payload_bridge) virtual_payload_bridge->Apply(m, d);
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      // A model-provided `home` keyframe is its physically valid reset pose.
      // mj_makeData alone uses qpos0 (zero free-base translation for this
      // URDF), which begins the robot inside the floor and causes a launch.
      const int home_key = mj_name2id(m, mjOBJ_KEY, "home");
      if (home_key >= 0)
        mj_resetDataKeyframe(m, d, home_key);
      sim->Load(m, d, filename);
      mj_forward(m, d);
      mujoco_ready.store(true);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  ctrlnoise = nullptr;
  d = nullptr;
  m = nullptr;
  mujoco_ready.store(false);
  return;
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (1)
  {
    if (mujoco_ready.load())
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  const int base_body_id = mj_name2id(m, mjOBJ_BODY, config.base_body.c_str());
  if (base_body_id < 0) throw std::runtime_error("Configured base body does not exist: " + config.base_body);
  config.band_attached_link = 6 * base_body_id;

  ChannelFactory::Instance()->Init(config.domain_id, config.interface);
  UnitreeSdk2Bridge unitree_interface(m, d);

  if (config.use_joystick == 1)
  {
    unitree_interface.SetupJoystick(config.joystick_device, config.joystick_type, config.joystick_bits);
  }

  if (config.print_scene_information == 1)
  {
    unitree_interface.PrintSceneInformation();
  }

  unitree_interface.Run();

  return nullptr;
}
//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false);

  // Load simulation configuration
  string path_mujoco = mujoco_dir ;
  std::cout << "Path to main.cc: " << mujoco_dir << std::endl;
  string config_name = "config.yaml";
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      config_name = argv[i + 1];
      break;
    }
  }
  const std::filesystem::path requested_config(config_name);
  const string path_config = requested_config.is_absolute() ? requested_config.string() :
      (std::filesystem::path(mujoco_dir) / requested_config).string();
  std::cout << "Path to main.cc: new " << path_config << std::endl;
  YAML::Node yaml_node = YAML::LoadFile(path_config);
  config.robot = yaml_node["robot"].as<std::string>();
  if (config.robot == "go2_rars01") {
    const char* override_path = std::getenv("STAGE4D_RARS01_ARM_SIM_CONFIG");
    const std::string gains_path = override_path && *override_path
        ? override_path : (std::filesystem::path(mujoco_dir) / "rars01_arm_sim.yaml").string();
    try {
      rars01_arm_gains = rars01_sim::LoadArmGains(gains_path);
      std::cout << "RARS01 simulator bias compensation: "
                << (rars01_arm_gains.enable_bias_compensation ? "ON (simulation oracle)" : "OFF (PD + command feedforward)")
                << std::endl;
    } catch (const std::exception& error) {
      std::cerr << error.what() << std::endl;
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    std::cout << "RARS01 arm sim gains loaded from " << gains_path << std::endl;
  }
  config.robot_scene = yaml_node["robot_scene"].as<std::string>();
  config.base_body = yaml_node["base_body"] ? yaml_node["base_body"].as<std::string>() : "base_link";
  config.domain_id = yaml_node["domain_id"].as<int>();
  config.interface = yaml_node["interface"].as<std::string>();
  config.enable_unitree_bridge = yaml_node["enable_unitree_bridge"] ? yaml_node["enable_unitree_bridge"].as<int>() : 1;
  config.enable_ros_bridge = yaml_node["enable_ros_bridge"] ? yaml_node["enable_ros_bridge"].as<int>() : 0;
  config.print_scene_information = yaml_node["print_scene_information"].as<int>();
  config.odom_topic = yaml_node["odom_topic"] ? yaml_node["odom_topic"].as<std::string>() : "/state_estimation";
  config.world_frame = yaml_node["world_frame"] ? yaml_node["world_frame"].as<std::string>() : "map";
  config.lidar_rate = yaml_node["lidar_rate"] ? yaml_node["lidar_rate"].as<double>() : 10.0;
  config.lidar_horizontal_samples = yaml_node["lidar_horizontal_samples"] ? yaml_node["lidar_horizontal_samples"].as<int>() : 360;
  config.lidar_vertical_lines = yaml_node["lidar_vertical_lines"] ? yaml_node["lidar_vertical_lines"].as<int>() : 18;
  config.lidar_min_range = yaml_node["lidar_min_range"] ? yaml_node["lidar_min_range"].as<double>() : 0.5;
  config.lidar_max_range = yaml_node["lidar_max_range"] ? yaml_node["lidar_max_range"].as<double>() : 100.0;
  config.sensor_start_delay = yaml_node["sensor_start_delay"] ? yaml_node["sensor_start_delay"].as<double>() : 12.0;
  config.enable_interactive_goal_click = yaml_node["enable_interactive_goal_click"]
      ? yaml_node["enable_interactive_goal_click"].as<int>() : 0;
  config.goal_click_drag_threshold_px = yaml_node["goal_click_drag_threshold_px"]
      ? yaml_node["goal_click_drag_threshold_px"].as<double>() : 5.0;
  if (yaml_node["goal_click_walkable_geoms"]) {
    config.goal_click_walkable_geoms.clear();
    for (const auto& geom : yaml_node["goal_click_walkable_geoms"]) {
      config.goal_click_walkable_geoms.push_back(geom.as<std::string>());
    }
  }
  config.enable_manual_manip_target = yaml_node["enable_manual_manip_target"]
      ? yaml_node["enable_manual_manip_target"].as<int>() : 0;
  config.enable_mujoco_hud = yaml_node["enable_mujoco_hud"]
      ? yaml_node["enable_mujoco_hud"].as<int>() : 1;
  const char* hud_env = std::getenv("STAGE4D_MUJOCO_HUD");
  if (hud_env != nullptr) {
    config.enable_mujoco_hud = (std::strcmp(hud_env, "0") != 0 &&
                                std::strcmp(hud_env, "false") != 0 &&
                                std::strcmp(hud_env, "FALSE") != 0);
  }
  config.allow_manual_manip_during_navigation = yaml_node["allow_manual_manip_during_navigation"]
      ? yaml_node["allow_manual_manip_during_navigation"].as<int>() : 0;
  config.enable_elastic_band = yaml_node["enable_elastic_band"].as<int>();
  config.use_joystick = yaml_node["use_joystick"].as<int>();
  config.joystick_type = yaml_node["joystick_type"].as<std::string>();
  config.joystick_device = yaml_node["joystick_device"].as<std::string>();
  config.joystick_bits = yaml_node["joystick_bits"].as<int>();

  sim->use_elastic_band_ = config.enable_elastic_band;

  std::filesystem::path fs_path(path_mujoco);

  std::filesystem::path parent_path = fs_path.parent_path();

  std::cout << "Path to main.cc: parent " << parent_path << std::endl;
  string scene_path = (parent_path / "unitree_robots" / config.robot / config.robot_scene).string();
  const char *filename = nullptr;
  if (argc > 1 && std::string(argv[1]) != "--config")
  {
    filename = argv[1];
  }
  else
  {
    filename = scene_path.c_str();
  }

  if (config.enable_unitree_bridge) {
    pthread_t unitree_thread;
    int rc = pthread_create(&unitree_thread, NULL, UnitreeSdk2BridgeThread, NULL);
    if (rc != 0) {
      std::cout << "Error:unable to create thread," << rc << std::endl;
      exit(-1);
    }
  } else {
    std::cout << "Unitree DDS bridge disabled by config; running MuJoCo physics/viewer only." << std::endl;
  }

  std::unique_ptr<MujocoNavigationGoalBridge> navigation_goal_bridge;
  std::unique_ptr<MujocoManualManipTargetBridge> manual_manip_bridge;
  if (config.enable_interactive_goal_click)
    navigation_goal_bridge = std::make_unique<MujocoNavigationGoalBridge>(config);
  if (config.enable_manual_manip_target)
    manual_manip_bridge = std::make_unique<MujocoManualManipTargetBridge>(config);
  if (navigation_goal_bridge || manual_manip_bridge) {
    sim->ConfigureSceneClick(true, config.goal_click_drag_threshold_px);
    sim->SetSceneClickCallback([nav = navigation_goal_bridge.get(), manual = manual_manip_bridge.get()](
        const mj::Simulate::SceneClickEvent& event) {
      if (nav) nav->HandleSceneClick(event);
      if (manual) manual->HandleSceneClick(event);
    });
    sim->SetSceneOverlayCallback([nav = navigation_goal_bridge.get(), manual = manual_manip_bridge.get()](mjvScene& scene) {
      if (nav) nav->DrawMarker(scene);
      if (manual) manual->DrawMarker(scene);
    });
    std::cout << "Interactive Stage4D: Ctrl+LMB goal, LMB cancel goal, LeftAlt+LMB manual target, RightAlt cancel manual target"
              << std::endl;
  }
  if (config.enable_mujoco_hud) {
    sim->SetSceneHudCallback([manual = manual_manip_bridge.get()](const mjrRect& rect, mjrContext& context) {
      if (manual) manual->DrawHud(rect, context);
      if (virtual_payload_bridge) virtual_payload_bridge->DrawHud(rect, context);
      if (ros_low_level_bridge) ros_low_level_bridge->DrawHud(rect, context);
    });
  }

  if (config.robot == "go2_rars01") mjcb_control = Go2Rars01HomeHold;
  if (config.enable_ros_bridge) ros_low_level_bridge = std::make_unique<MujocoRosLowLevelBridge>();
  mujoco_clock = std::make_unique<MujocoClockPublisher>();
  ground_truth_odom = std::make_unique<MujocoGroundTruthOdom>();
  collision_diagnostics = std::make_unique<MujocoCollisionDiagnostics>();
  pointlio_sensor_bridge = std::make_unique<MujocoPointLioSensorBridge>(config);

  virtual_payload_bridge = std::make_unique<MujocoVirtualPayloadBridge>();
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  sim->RenderLoop();
  sim->exitrequest.store(true);
  physicsthreadhandle.join();

  // Viewer callbacks hold raw bridge pointers, so clear them first.
  sim->SetSceneClickCallback({});
  sim->SetSceneOverlayCallback({});
  sim->SetSceneHudCallback({});
  rclcpp::shutdown();
  virtual_payload_bridge.reset();
  manual_manip_bridge.reset();
  navigation_goal_bridge.reset();
  pointlio_sensor_bridge.reset();
  collision_diagnostics.reset();
  ground_truth_odom.reset();
  ros_low_level_bridge.reset();
  mujoco_clock.reset();
  return 0;
}
