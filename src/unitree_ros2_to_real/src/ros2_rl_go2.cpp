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
    throw std::runtime_error("Phase R1 cannot enable actuator output: no motor transport is compiled");
  core_.Load(config,model); // throws on missing/legacy config or failed model; no fallback
  const auto deployment=YAML::LoadFile(config)["real_deployment"];
  low_state_timeout_=deployment["low_state_timeout_sec"].as<double>();
  limits_={deployment["max_linear_x"].as<double>(),deployment["max_linear_y"].as<double>(),deployment["max_yaw_rate"].as<double>()};
  navigation_.SetTimeout(deployment["cmd_vel_timeout_sec"].as<double>());
  const auto cmd_topic=declare_parameter<std::string>("cmd_vel_topic","/cmd_vel");
  state_sub_=create_subscription<unitree_go::msg::LowState>("/lowstate",rclcpp::SensorDataQoS(),[this](unitree_go::msg::LowState::ConstSharedPtr msg) {
    try {
      sim2real::Legs q{},dq{};
      for(int i=0;i<12;++i) { q[i]=msg->motor_state[i].q; dq[i]=msg->motor_state[i].dq; }
      double norm=0;
      for(float v:msg->imu_state.quaternion) { if(!std::isfinite(v)) throw std::runtime_error("Invalid quaternion"); norm+=v*v; }
      if(norm<1e-16) throw std::runtime_error("Zero quaternion");
      for(float v:msg->imu_state.gyroscope) if(!std::isfinite(v)) throw std::runtime_error("Invalid gyro");
      core_.SetMeasuredLegs(q,dq);
      core_.agent().obs.base_quat=torch::tensor({msg->imu_state.quaternion[1],msg->imu_state.quaternion[2],msg->imu_state.quaternion[3],msg->imu_state.quaternion[0]});
      core_.agent().obs.ang_vel=torch::tensor({msg->imu_state.gyroscope[0],msg->imu_state.gyroscope[1],msg->imu_state.gyroscope[2]});
      state_stamp_=NavigationCommandAdapter::Clock::now(); received_state_=true;
    } catch(const std::exception& e) {
      received_state_=false; output_gate_.Fault(); core_.RequestMode(sim2real::Mode::FAULT,{});
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
    // RARS bridge and verified ownership belong to R2. Never invent readiness.
    const sim2real::Readiness ready{core_.loaded(),fresh,false,false};
    if(output_gate_.Allows(ready)) throw std::logic_error("R1 gate invariant violated");
    std_msgs::msg::String status;
    status.data=core_.mode()==sim2real::Mode::FAULT?"FAULT output_disabled":"DISARMED output_disabled";
    status.data+=fresh?" lowstate_fresh":" lowstate_unavailable";
    status_pub_->publish(status);
  });
  RCLCPP_INFO(get_logger(),"DISARMED: Phase R1 has no LowCmd publisher, SDK2 channel, serial transport or arming API");
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<InterfaceRos>()); }
  catch(const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("sim2real"),"Startup failed closed: %s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
