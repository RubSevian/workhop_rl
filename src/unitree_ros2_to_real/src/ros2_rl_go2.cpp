#include "ros2_rl_go2.hpp"
#include <ATen/Parallel.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

InterfaceRos::InterfaceRos() : Node("rl_locomotion_disarmed") {
  // Match the measured CPU baseline and leave CPU capacity for Point-LIO.
  at::set_num_threads(1);
  at::set_num_interop_threads(1);
  const auto config=declare_parameter<std::string>("config_path", "");
  const auto model=declare_parameter<std::string>("model_path", "");
  if(declare_parameter<bool>("enable_actuator_output", false))
    throw std::runtime_error("Phase R2 read-only executable cannot enable actuator output");
  core_.Load(config,model); // throws on missing/legacy config or failed model; no fallback
  const auto deployment=YAML::LoadFile(config)["real_deployment"];
  low_state_timeout_=deployment["lowstate"]["stale_timeout_s"].as<double>();
  lowstate_=sim2real::LowStateReader(low_state_timeout_);
  remote_=sim2real::RemoteSafety(deployment["remote"]["takeover_hold_s"].as<double>(), deployment["remote"]["stale_timeout_s"].as<double>());
  core_.safety().SetSportTimeout(deployment["sport_mode"]["observation_timeout_s"].as<double>());
  if(!deployment["safety"]["startup_disarmed"].as<bool>() || deployment["safety"]["actuator_output_default"].as<bool>() ||
     !deployment["sport_mode"]["require_release_verified"].as<bool>() ||
     deployment["remote"]["takeover_chord"].as<std::vector<std::string>>() != std::vector<std::string>{"L1","L2","A"} ||
     deployment["remote"]["emergency_chord"].size()!=0)
    throw std::runtime_error("R2 safe configuration invariants violated");
  limits_={deployment["max_linear_x"].as<double>(),deployment["max_linear_y"].as<double>(),deployment["max_yaw_rate"].as<double>()};
  navigation_.SetTimeout(deployment["cmd_vel_timeout_sec"].as<double>());
  const auto cmd_topic=declare_parameter<std::string>("cmd_vel_topic","/cmd_vel");
  state_sub_=create_subscription<unitree_go::msg::LowState>("/lowstate",rclcpp::SensorDataQoS(),[this](unitree_go::msg::LowState::ConstSharedPtr msg) {
    try {
      const auto now=sim2real::SafetyClock::now();
      const bool remote_valid=remote_.Receive(msg->wireless_remote,now);
      if(!lowstate_.Receive(*msg,now)) throw std::runtime_error(lowstate_.snapshot().rejection);
      const auto& state=lowstate_.snapshot();
      core_.SetMeasuredLegs(state.motor_q,state.motor_dq);
      core_.agent().obs.base_quat=torch::tensor(std::vector<float>(state.quaternion_xyzw.begin(),state.quaternion_xyzw.end()));
      core_.agent().obs.ang_vel=torch::tensor(std::vector<float>(state.gyro.begin(),state.gyro.end()));
      // Invalid remote cancels the hold and blocks readiness; no takeover here.
      (void)remote_valid;
      state_stamp_=NavigationCommandAdapter::Clock::now(); received_state_=true;
    } catch(const std::exception& e) {
      received_state_=false; core_.safety().Fault(); core_.RequestMode(sim2real::Mode::FAULT,{});
      RCLCPP_ERROR(get_logger(),"Feedback rejected: %s",e.what());
    }
  });
  cmd_sub_=create_subscription<geometry_msgs::msg::TwistStamped>(cmd_topic,10,[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
    const std::array<double,3> v{msg->twist.linear.x,msg->twist.linear.y,msg->twist.angular.z};
    for(double x:v) if(!std::isfinite(x)) return; // invalid packets never refresh watchdog
    navigation_.Accept(std::clamp(v[0],-limits_[0],limits_[0]),std::clamp(v[1],-limits_[1],limits_[1]),std::clamp(v[2],-limits_[2],limits_[2]),NavigationCommandAdapter::Clock::now());
  });
  nav_sub_=create_subscription<std_msgs::msg::Bool>("/navigation_active",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::Bool::ConstSharedPtr msg) { navigation_.SetNavigationActive(msg->data); });
  status_pub_=create_publisher<std_msgs::msg::String>("/go2/locomotion_status",10);
  timer_=create_wall_timer(std::chrono::milliseconds(20),[this]() {
    const auto now=NavigationCommandAdapter::Clock::now();
    const bool fresh=received_state_ && std::chrono::duration<double>(now-state_stamp_).count()<=low_state_timeout_;
    const auto cmd=navigation_.GetSafeCommand(now);
    core_.agent().obs.command=torch::tensor({cmd.value[0],cmd.value[1],cmd.value[2]});
    if(remote_.Poll(now)) core_.safety().TakeoverRequest();
    sim2real::SafetyReadiness ready;
    ready.model_loaded=core_.loaded();ready.config_valid=true;
    ready.lowstate_fresh=lowstate_.Fresh(now);ready.motor_state_valid=lowstate_.snapshot().valid;
    ready.remote_fresh=remote_.status().remote_valid;
    // No connected arm owner or Sport Mode status IPC exists in this read-only
    // executable. Keep those readiness inputs false/UNKNOWN, never fabricate them.
    core_.safety().Update(ready,now);
    if(core_.safety().AllowsOutput(ready,now)) throw std::logic_error("R2 output-disabled invariant violated");
    const auto blockers=core_.safety().Blockers(ready,now);
    std_msgs::msg::String status;
    status.data=std::string(sim2real::StateName(core_.safety().state()))+" output_disabled blockers=";
    for(const auto& blocker:blockers)status.data+=blocker+",";
    const auto& remote=remote_.status();
    status.data+=" remote_valid="+std::to_string(remote.remote_valid)+" remote_age_ms="+std::to_string(remote.remote_age_ms)+
      " button_mask="+std::to_string(remote.button_mask)+" buttons="+remote.decoded_buttons+
      " takeover_hold_active="+std::to_string(remote.takeover_hold_active)+
      " takeover_request_latched="+std::to_string(remote.takeover_request_latched);
    if(std::chrono::duration<double>(now-diagnostic_stamp_).count()>=1) {
      RCLCPP_INFO(get_logger(),"%s\n%s",status.data.c_str(),lowstate_.Diagnostic(now).c_str());diagnostic_stamp_=now;
    }
    status_pub_->publish(status);
  });
  RCLCPP_INFO(get_logger(),"DISARMED: R2 read-only diagnostics, no LowCmd publisher, SDK2 channel, serial open or output-enable API");
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<InterfaceRos>()); }
  catch(const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("sim2real"),"Startup failed closed: %s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
