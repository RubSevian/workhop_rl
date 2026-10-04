#include "rars_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cmath>
#include <algorithm>
using namespace sim2real;
namespace {
int64_t Ns(SafetyTime t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
}
class CapturingTransport final:public RarsTransport {
 public:
 explicit CapturingTransport(SdkRarsReadOnlyTransport& t):source(t){}
 std::optional<RarsFrame> Read(SafetyTime now) override {auto f=source.Read(now);if(f)last=f;return f;}
 std::optional<AcceptedArmTarget> LastAcceptedTarget() const override{return source.LastAcceptedTarget();}
 const rars_arm::ArmConfiguration& Configuration() const override{return source.Configuration();}
 SdkRarsReadOnlyTransport& source;std::optional<RarsFrame> last;
};
class ArmOwner final:public rclcpp::Node {
 public:
 ArmOwner():Node("rars01_r3_serial_owner") {
  const auto path=declare_parameter<std::string>("sdk_config_path","");
  if(path.empty())throw std::runtime_error("Existing GraspNet SDK configuration path required; no invented offsets");
  const auto file=YAML::LoadFile(path);const auto cfg=file["robot"]["rars01"];
  if(!cfg)throw std::runtime_error("Expected existing robot.rars01 SDK config");
  rars_arm::ArmConfiguration config;config.port_name=cfg["port"].as<std::string>();config.baud_rate=cfg["baud_rate"].as<unsigned>();
  const auto directions=cfg["joint_directions"].as<std::vector<float>>();if(directions.size()!=7)throw std::runtime_error("Seven SDK directions required");
  for(int i=0;i<7;++i)config.motors[i].direction=directions[i];
  // Existing SDK defaults are retained unless the operator's existing file
  // explicitly provides software offsets. Physical GUI setZero stays in motors.
  if(cfg["joint_zero_offsets"]){auto z=cfg["joint_zero_offsets"].as<std::vector<float>>();if(z.size()!=7)throw std::runtime_error("Seven existing offsets required");for(int i=0;i<7;++i)config.motors[i].zero_offset=z[i];}
  auto seven=[&](const char* key,auto& out){auto v=cfg[key].as<std::vector<float>>();if(v.size()!=7)throw std::runtime_error("Seven SDK control values required");std::copy(v.begin(),v.end(),out.begin());};
  seven("position_kp",config.default_kp);seven("position_kd",config.default_kd);seven("position_velocity_limits_rad_s",config.position_velocity_limits);
  auto modes=cfg["control_modes"].as<std::vector<std::string>>();if(modes.size()!=7)throw std::runtime_error("Seven SDK modes required");
  for(int i=0;i<7;++i){if(modes[i]=="pos_vel")config.control_modes[i]=rars_arm::ArmControlMode::PositionVelocity;else if(modes[i]=="mit")config.control_modes[i]=rars_arm::ArmControlMode::MIT;else throw std::runtime_error("Unknown SDK mode");}
  config.feedback_watchdog_enabled=true;config.feedback_timeout=std::chrono::milliseconds(250);
  allow_hold_=declare_parameter<bool>("allow_static_hold",false);
  const bool connect=declare_parameter<bool>("connect_serial",false);
  const auto lockdir=declare_parameter<std::string>("lock_directory","");
  if(lockdir.empty())throw std::runtime_error("Shared serial lease directory required");
  lease_=std::make_unique<SerialOwnerLease>(lockdir,config.port_name);if(!lease_->acquired())throw std::runtime_error("Serial device owner already exists / lock unavailable");
  sdk_=std::make_unique<rars_arm::RarsArm>(config);
  transport_=std::make_unique<SdkRarsReadOnlyTransport>(*sdk_,*lease_);capture_=std::make_unique<CapturingTransport>(*transport_);bridge_=std::make_unique<RarsBridge>(*capture_);
  RCLCPP_INFO(get_logger(),"Existing config: %s\n%s",path.c_str(),bridge_->CalibrationDiagnostic().c_str());
  if(connect&&!sdk_->connect())throw std::runtime_error("Read-only serial connect failed");
  state_=create_publisher<std_msgs::msg::String>("/rars01/commissioning/state",10);
  hold_=create_service<std_srvs::srv::Trigger>("/rars01/commissioning/hold_current",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){auto r=HoldCurrent(true);res->success=r.first;res->message=r.second;});
  // Abort/emergency may refresh HOLD_CURRENT only if motors already enabled;
  // this channel cannot perform the first physical motor enable.
  hold_sub_=create_subscription<std_msgs::msg::String>("/rars01/commissioning/hold_request",1,[this](std_msgs::msg::String::ConstSharedPtr m){if(m->data=="hold_current"&&sdk_->isEnabled())HoldCurrent(false);});
  timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{Poll();});
 }
 private:
 std::pair<bool,std::string> HoldCurrent(bool operator_service) {
  if(!allow_hold_)return {false,"allow_static_hold=false; explicit operator gate required"};
  const auto now=SafetyClock::now();const auto& s=bridge_->Poll(now);
  if(!s.feedback_ready)return {false,"fresh measured six-joint feedback required"};
  // SDK enables all seven motors; require the separate gripper's measured q too.
  // Do not substitute a zero gripper target or run open/close.
  if(!capture_->last)return {false,"complete measured frame required"};
  const auto& g=capture_->last->joints;
  if(!g.valid[6]||g.motor_id[6]!=7||!std::isfinite(g.position[6]))return {false,"separate gripper feedback required before all-motor enable"};
  rars_arm::RarsArm::MotorValues target;for(int i=0;i<6;++i)target[i]=s.q[i];target[6]=g.position[6];
  if(!sdk_->isEnabled()) {
   if(!operator_service)return {false,"abort channel cannot enable motors"};
   if(!sdk_->enable())return {false,"SDK motor enable failed"};
  }
  if(!sdk_->sendPositionTargets(target))return {false,"SDK control path rejected static target"};
  held_=target;holding_=true;std::array<float,6> accepted;std::copy_n(target.begin(),6,accepted.begin());
  transport_->RecordAcceptedTarget(accepted,SafetyClock::now());return {true,"static measured HOLD_CURRENT accepted by SDK; no trajectory"};
 }
 std::vector<int> Ids() const {
  std::vector<int> ids;if(capture_->last)for(int i=0;i<6;++i)ids.push_back(capture_->last->joints.motor_id[i]);return ids;
 }
 void Poll() {
  const auto now=SafetyClock::now();const auto& s=bridge_->Poll(now);
  if(holding_&&s.feedback_ready) {
   if(sdk_->sendPositionTargets(held_)){std::array<float,6> a;std::copy_n(held_.begin(),6,a.begin());transport_->RecordAcceptedTarget(a,SafetyClock::now());}
   else holding_=false;
  } else if(holding_)holding_=false; // SDK/STM watchdog handles absence; no fabricated feedback.
  const auto& updated=bridge_->Poll(SafetyClock::now());const auto status=sdk_->communicationStatus();
  YAML::Emitter e;e<<YAML::Flow<<YAML::BeginMap<<YAML::Key<<"observed_ns"<<YAML::Value<<Ns(SafetyClock::now())
   <<YAML::Key<<"frame_received_ns"<<YAML::Value<<Ns(updated.received)
   <<YAML::Key<<"common_feedback_age_s"<<YAML::Value<<updated.feedback_age_ms/1000
   <<YAML::Key<<"measured_q"<<YAML::Value<<std::vector<float>(updated.q.begin(),updated.q.end())
   <<YAML::Key<<"measured_dq"<<YAML::Value<<std::vector<float>(updated.dq.begin(),updated.dq.end())
   <<YAML::Key<<"motor_ids"<<YAML::Value<<Ids()
   <<YAML::Key<<"valid"<<YAML::Value<<std::vector<bool>(updated.valid.begin(),updated.valid.end())
   <<YAML::Key<<"feedback_ready"<<YAML::Value<<updated.feedback_ready
   <<YAML::Key<<"static_hold"<<YAML::Value<<(holding_&&status.enabled)
   <<YAML::Key<<"per_joint_freshness_proven"<<YAML::Value<<false
   <<YAML::Key<<"target_valid"<<YAML::Value<<updated.target_ready;
  if(updated.target)e<<YAML::Key<<"accepted_target"<<YAML::Value<<std::vector<float>(updated.target->q.begin(),updated.target->q.end())<<YAML::Key<<"accepted_ns"<<YAML::Value<<Ns(updated.target->accepted);
  e<<YAML::EndMap;std_msgs::msg::String msg;msg.data=e.c_str();state_->publish(msg);
 }
 bool allow_hold_=false,holding_=false;rars_arm::RarsArm::MotorValues held_{};
 std::unique_ptr<SerialOwnerLease> lease_;std::unique_ptr<rars_arm::RarsArm> sdk_;
 std::unique_ptr<SdkRarsReadOnlyTransport> transport_;std::unique_ptr<RarsBridge> bridge_;
 std::unique_ptr<CapturingTransport> capture_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hold_sub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<ArmOwner>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("rars_r3"),"Failed closed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
