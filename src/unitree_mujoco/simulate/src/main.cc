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
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <array>
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
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
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

    int enable_elastic_band = 0;
    int band_attached_link = 0;

  } config;

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
      lowcmd_sub_ = node_->create_subscription<unitree_go::msg::LowCmd>(
          "lowcmd", 10,
          [this](const unitree_go::msg::LowCmd::SharedPtr msg) {
            for (int i = 0; i < 12; ++i) {
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
      // Only the 12 Go2 leg actuators are commandable through this interface.
      // RARS01 remains exclusively under Go2Rars01HomeHold at ctrl[12:20].
      for (int i = 0; i < 12; ++i) {
        const int actuator = leg_actuator_ids_[i];
        const auto& motor = command.motor_cmd[i];
        const double raw = motor.tau + motor.kp * (motor.q - data->sensordata[leg_pos_adr_[i]]) +
            motor.kd * (motor.dq - data->sensordata[leg_vel_adr_[i]]);
        const double limit = (i % 3 == 2) ? 35.55 : 23.7;
        data->ctrl[actuator] = std::clamp(raw, -limit, limit);
        if (raw != data->ctrl[actuator]) ++torque_saturation_count_;
      }
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
    }

  private:
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
          if (rars_joint_ids_[i] < 0) throw std::runtime_error("Missing RARS01 joint");
        }
        imu_quat_adr_ = SensorAddress(model, "imu_quat", 4);
        imu_gyro_adr_ = SensorAddress(model, "imu_gyro", 3);
        imu_acc_adr_ = SensorAddress(model, "imu_acc", 3);
        ready_ = true;
        std_msgs::msg::Float64 physics_dt;
        physics_dt.data = model->opt.timestep;
        physics_dt_pub_->publish(physics_dt);
        RCLCPP_INFO(node_->get_logger(), "ROS low-level bridge ready: /lowcmd -> 12 Go2 legs, /lowstate <- 20 motors");
      } catch (const std::exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "ROS low-level bridge disabled: %s", error.what());
      }
      return ready_;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr physics_dt_pub_;
    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr lowcmd_sub_;
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
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
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
      odom.header.stamp = node_->now();
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

      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = odom.child_frame_id;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

  private:
    bool Resolve(const mjModel* model) {
      if (resolved_model_ == model) return freejoint_id_ >= 0 && base_body_id_ >= 0;
      resolved_model_ = model;
      base_body_id_ = mj_name2id(model, mjOBJ_BODY, config.base_body.c_str());
      freejoint_id_ = mj_name2id(model, mjOBJ_JOINT, "base_freejoint");
      next_publish_time_ = 0.0;
      last_sim_time_ = -1.0e30;
      if (base_body_id_ < 0 || freejoint_id_ < 0 || model->jnt_type[freejoint_id_] != mjJNT_FREE) {
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
          max_range_(cfg.lidar_max_range) {
      imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/unilidar/imu", 20);
      cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/unilidar/cloud", 5);
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
        physics_ticks_ = 0;
      }
      last_sim_time_ = data->time;
      ++physics_ticks_;
      if (imu_enabled_.load() && (physics_ticks_ % 2) == 0) PublishImu(model, data);
      if (lidar_enabled_.load() && data->time + 1.0e-9 >= next_lidar_time_) {
        PublishLidar(model, data);
        next_lidar_time_ = data->time + 1.0 / std::max(1.0, lidar_rate_);
      }
    }

  private:
    static int SensorAddress(const mjModel* model, const char* name, int dimension) {
      const int sensor = mj_name2id(model, mjOBJ_SENSOR, name);
      if (sensor < 0 || model->sensor_dim[sensor] != dimension) return -1;
      return model->sensor_adr[sensor];
    }

    bool Resolve(const mjModel* model) {
      if (resolved_model_ == model) return ready_;
      resolved_model_ = model;
      radar_body_id_ = mj_name2id(model, mjOBJ_BODY, "radar");
      base_body_id_ = mj_name2id(model, mjOBJ_BODY, "base");
      imu_quat_adr_ = SensorAddress(model, "imu_quat", 4);
      imu_gyro_adr_ = SensorAddress(model, "imu_gyro", 3);
      imu_acc_adr_ = SensorAddress(model, "imu_acc", 3);
      ready_ = radar_body_id_ >= 0 && base_body_id_ >= 0 && imu_quat_adr_ >= 0 &&
               imu_gyro_adr_ >= 0 && imu_acc_adr_ >= 0;
      next_lidar_time_ = 0.0;
      last_sim_time_ = -1.0e30;
      if (ready_) {
        RCLCPP_INFO(node_->get_logger(),
                    "Point-LIO sensors ready: /unilidar/imu=250Hz, /unilidar/cloud=%.1fHz, frame=unilidar, scan=%dx%d",
                    lidar_rate_, vertical_lines_, horizontal_samples_);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Point-LIO sensor bridge missing radar/body or IMU sensors");
      }
      return ready_;
    }

    builtin_interfaces::msg::Time SimStamp(mjtNum seconds) const {
      builtin_interfaces::msg::Time stamp;
      const int64_t total_ns = static_cast<int64_t>(seconds * 1.0e9);
      stamp.sec = static_cast<int32_t>(total_ns / 1000000000LL);
      stamp.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
      return stamp;
    }

    void PublishImu(const mjModel*, const mjData* data) {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = SimStamp(data->time);
      msg.header.frame_id = "imu";
      msg.orientation.w = data->sensordata[imu_quat_adr_ + 0];
      msg.orientation.x = data->sensordata[imu_quat_adr_ + 1];
      msg.orientation.y = data->sensordata[imu_quat_adr_ + 2];
      msg.orientation.z = data->sensordata[imu_quat_adr_ + 3];
      msg.angular_velocity.x = data->sensordata[imu_gyro_adr_ + 0];
      msg.angular_velocity.y = data->sensordata[imu_gyro_adr_ + 1];
      msg.angular_velocity.z = data->sensordata[imu_gyro_adr_ + 2];
      msg.linear_acceleration.x = data->sensordata[imu_acc_adr_ + 0];
      msg.linear_acceleration.y = data->sensordata[imu_acc_adr_ + 1];
      msg.linear_acceleration.z = data->sensordata[imu_acc_adr_ + 2];
      imu_pub_->publish(msg);
      if (!logged_imu_) {
        logged_imu_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Stationary IMU sample frame=%s gyro=[%.3f %.3f %.3f] acc=[%.3f %.3f %.3f]",
                    msg.header.frame_id.c_str(), msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z);
      }
    }

    void PublishLidar(const mjModel* model, const mjData* data) {
      const int count = std::max(1, vertical_lines_) * std::max(1, horizontal_samples_);
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.stamp = SimStamp(data->time);
      cloud.header.frame_id = "unilidar";
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(6,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
          "time", 1, sensor_msgs::msg::PointField::FLOAT32,
          "ring", 1, sensor_msgs::msg::PointField::UINT16);
      modifier.resize(count);
      sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
      sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");
      sensor_msgs::PointCloud2Iterator<float> time_it(cloud, "time");
      sensor_msgs::PointCloud2Iterator<uint16_t> ring_it(cloud, "ring");
      const mjtNum* origin = data->xpos + 3 * radar_body_id_;
      const mjtNum* rotation = data->xmat + 9 * radar_body_id_;
      const double scan_period = 1.0 / std::max(1.0, lidar_rate_);
      for (int row = 0; row < std::max(1, vertical_lines_); ++row) {
        const double vertical = (-15.0 + 30.0 * row / std::max(1, vertical_lines_ - 1)) * M_PI / 180.0;
        for (int col = 0; col < std::max(1, horizontal_samples_); ++col) {
          const double horizontal = 2.0 * M_PI * col / std::max(1, horizontal_samples_);
          const double local_dir[3] = {std::cos(vertical) * std::cos(horizontal),
                                       std::cos(vertical) * std::sin(horizontal), std::sin(vertical)};
          mjtNum world_dir[3] = {
              rotation[0] * local_dir[0] + rotation[1] * local_dir[1] + rotation[2] * local_dir[2],
              rotation[3] * local_dir[0] + rotation[4] * local_dir[1] + rotation[5] * local_dir[2],
              rotation[6] * local_dir[0] + rotation[7] * local_dir[1] + rotation[8] * local_dir[2]};
          int geom_id[1] = {-1};
          const mjtNum distance = mj_ray(model, data, origin, world_dir, nullptr, 1, base_body_id_, geom_id);
          const bool valid = geom_id[0] >= 0 && distance >= min_range_ && distance <= max_range_;
          const float range = static_cast<float>(valid ? distance : max_range_);
          *x_it = range * static_cast<float>(local_dir[0]);
          *y_it = range * static_cast<float>(local_dir[1]);
          *z_it = range * static_cast<float>(local_dir[2]);
          *intensity_it = valid ? 1.0F : 0.0F;
          *time_it = static_cast<float>(scan_period * col / std::max(1, horizontal_samples_));
          *ring_it = static_cast<uint16_t>(row);
          ++x_it; ++y_it; ++z_it; ++intensity_it; ++time_it; ++ring_it;
        }
      }
      cloud_pub_->publish(cloud);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lidar_toggle_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imu_toggle_;
    std::thread spin_thread_;
    std::atomic<bool> lidar_enabled_{true};
    std::atomic<bool> imu_enabled_{true};
    const mjModel* resolved_model_ = nullptr;
    int radar_body_id_ = -1;
    int base_body_id_ = -1;
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    bool ready_ = false;
    bool logged_imu_ = false;
    uint64_t physics_ticks_ = 0;
    mjtNum next_lidar_time_ = 0.0;
    mjtNum last_sim_time_ = -1.0e30;
    double lidar_rate_;
    int horizontal_samples_;
    int vertical_lines_;
    double min_range_;
    double max_range_;
  };

  std::unique_ptr<MujocoPointLioSensorBridge> pointlio_sensor_bridge;

  using Seconds = std::chrono::duration<double>;

  // The Unitree bridge owns ctrl[0:12]. This callback owns only the dedicated
  // arm/gripper actuators in the combined model at every MuJoCo physics step.
  void Go2Rars01HomeHold(const mjModel* model, mjData* data) {
    if (model->nu < 20) return;
    const char* joints[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_left_joint", "gripper_right_joint"};
    const char* actuators[] = {"joint1_motor", "joint2_motor", "joint3_motor", "joint4_motor", "joint5_motor", "joint6_motor", "gripper_left_motor", "gripper_right_motor"};
    const double kp[] = {20, 20, 20, 6, 6, 6, 20, 20};
    const double kd[] = {1, 1, 1, .4, .4, .4, .2, .2};
    for (int i = 0; i < 8; ++i) {
      int jid = mj_name2id(model, mjOBJ_JOINT, joints[i]);
      int aid = mj_name2id(model, mjOBJ_ACTUATOR, actuators[i]);
      if (jid < 0 || aid != 12 + i) return;
      double tau = -kp[i] * data->qpos[model->jnt_qposadr[jid]] - kd[i] * data->qvel[model->jnt_dofadr[jid]];
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
              if (ros_low_level_bridge) ros_low_level_bridge->Apply(m, d);
              // run single step, let next iteration deal with timing
              mj_step(m, d);
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
                if (ros_low_level_bridge) ros_low_level_bridge->Apply(m, d);
                // call mj_step
                mj_step(m, d);
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

  exit(0);
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

  pthread_exit(NULL);
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

  if (config.robot == "go2_rars01") mjcb_control = Go2Rars01HomeHold;
  if (config.enable_ros_bridge) ros_low_level_bridge = std::make_unique<MujocoRosLowLevelBridge>();
  ground_truth_odom = std::make_unique<MujocoGroundTruthOdom>();
  collision_diagnostics = std::make_unique<MujocoCollisionDiagnostics>();
  pointlio_sensor_bridge = std::make_unique<MujocoPointLioSensorBridge>(config);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  rclcpp::shutdown();
  pthread_exit(NULL);
  return 0;
}
