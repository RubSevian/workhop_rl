#pragma once
#include "virtual_payload.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <thread>

class MujocoVirtualPayloadBridge {
 public:
  MujocoVirtualPayloadBridge() : payload_(Settings()),
      node_(std::make_shared<rclcpp::Node>("mujoco_virtual_payload")) {
    pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/stage4d/payload_status", rclcpp::QoS(1).transient_local());
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/stage4d/payload_attach", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
          request_.store(msg->data ? 1 : 0);
        });
    thread_ = std::thread([this] { rclcpp::spin(node_); });
  }
  ~MujocoVirtualPayloadBridge() { if (thread_.joinable()) thread_.join(); }
  void Initialize(mjModel* model) {
    payload_.Initialize(model); request_.store(-1); attached_.store(false);
    next_publish_=0; error_.clear();
  }
  // Physics owner only, inside sim.mtx, before every mj_step.
  void Apply(mjModel* model, mjData* data) {
    if (payload_.Update(model, data)) {
      request_.store(-1); error_.clear(); next_publish_=0;
      RCLCPP_INFO(node_->get_logger(), "[PAYLOAD][DETACHED] reason=reset");
    }
    const int request = request_.exchange(-1);
    if (request >= 0) {
      try {
        if (request) {
          const bool previous = payload_.attached();
          payload_.Attach(model, data);
          if (!previous) RCLCPP_INFO(node_->get_logger(),
              "[PAYLOAD][ATTACHED] mass=%.3f kg offset_end_link=(%.3f, %.3f, %.3f) m sim_time=%.3f s",
              payload_.settings().mass, payload_.settings().offset[0],
              payload_.settings().offset[1], payload_.settings().offset[2], data->time);
        } else {
          payload_.Detach(model, data);
          RCLCPP_INFO(node_->get_logger(), "[PAYLOAD][DETACHED] reason=request");
        }
        error_.clear();
      } catch (const std::exception& error) {
        error_ = error.what();
        RCLCPP_ERROR(node_->get_logger(), "[PAYLOAD][ATTACH_FAILED] reason=%s", error_.c_str());
      }
      next_publish_=0;
    }
    attached_.store(payload_.attached());
    if (data->time + 1e-9 >= next_publish_) {
      next_publish_ = (std::floor(data->time / 0.02 + 1e-8) + 1) * 0.02;
      std::ostringstream json;
      json << "{\"sim_time_s\":" << data->time
           << ",\"payload_attached\":" << (payload_.attached() ? "true" : "false")
           << ",\"payload_mass_kg\":" << payload_.settings().mass
           << ",\"enabled\":" << (payload_.settings().enabled ? "true" : "false")
           << ",\"reset_count\":" << payload_.reset_count()
           << ",\"error\":\"" << error_ << "\"}";
      std_msgs::msg::String msg; msg.data=json.str(); pub_->publish(msg);
    }
  }
  void DrawHud(const mjrRect& rect, mjrContext& context) const {
    std::ostringstream text;
    text << "PAYLOAD: " << (attached_.load() ? "ON" : "OFF") << "\nMASS: "
         << std::fixed << std::setprecision(2) << payload_.settings().mass << " kg";
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, text.str().c_str(), "", &context);
  }
 private:
  static stage4d::PayloadSettings Settings() {
    stage4d::PayloadSettings settings;
    const char* enabled=std::getenv("STAGE4D_VIRTUAL_PAYLOAD_ENABLED");
    settings.enabled=enabled && std::string(enabled)=="true";
    if (const char* mass=std::getenv("STAGE4D_PAYLOAD_MASS_KG")) {
      size_t consumed=0; settings.mass=std::stod(mass, &consumed);
      if (std::string(mass).size()!=consumed) throw std::invalid_argument("invalid payload mass");
    }
    if (const char* offset=std::getenv("STAGE4D_PAYLOAD_OFFSET_M")) {
      std::istringstream values(offset);
      for (double& value : settings.offset)
        if (!(values >> value)) throw std::invalid_argument("invalid payload offset");
      std::string extra;
      if (values >> extra) throw std::invalid_argument("invalid payload offset length");
    }
    return settings;
  }
  stage4d::VirtualPayload payload_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::thread thread_;
  std::atomic<int> request_{-1};
  std::atomic<bool> attached_{false};
  double next_publish_=0;
  std::string error_;
};
