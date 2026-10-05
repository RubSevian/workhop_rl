#include "rars_auto_home.hpp"
#include "rars_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cmath>
using namespace sim2real;
namespace {
int64_t Ns(ArmTime t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
template<class T,size_t N> std::vector<T> Values(const std::array<T,N>& a){return std::vector<T>(a.begin(),a.end());}
std::string BootId(){std::ifstream file("/proc/sys/kernel/random/boot_id");std::string id;std::getline(file,id);if(id.empty())throw std::runtime_error("Cannot read boot ID");return id;}
class SdkHomeTransport final:public AutoHomeTransport {
 public:
 explicit SdkHomeTransport(rars_arm::RarsArm& sdk):sdk_(sdk){}
 bool Connect() override{return sdk_.connect();}
 bool Enable() override{return sdk_.enable();}
 bool Disable() override{return sdk_.disable();}
 bool Send(const rars_arm::RarsArm::MotorValues& q) override{return sdk_.sendPositionTargets(q);}
 bool Read(rars_arm::JointState& q) override{return sdk_.tryReadJointState(q);}
 rars_arm::CommunicationStatus Status() const override{return sdk_.communicationStatus();}
 std::string Error() const override{return sdk_.lastError();}
 private:rars_arm::RarsArm& sdk_;
};
}
class ArmOwner final:public rclcpp::Node {
 public:
 ArmOwner():Node("rars01_r3_serial_owner") {
  const auto path=declare_parameter<std::string>("sdk_config_path","");
  const auto deployment_path=declare_parameter<std::string>("config_path","");
  if(path.empty()||deployment_path.empty())throw std::runtime_error("Existing SDK and deployment config paths required");
  const auto cfg=YAML::LoadFile(path)["robot"]["rars01"];
  const auto deployment=YAML::LoadFile(deployment_path)["real_deployment"]["rars01"];
  const auto auto_cfg=deployment["auto_home"];
  if(!cfg||!auto_cfg)throw std::runtime_error("Expected robot.rars01 and real_deployment.rars01.auto_home");
  rars_arm::ArmConfiguration config;
  config.port_name=declare_parameter<std::string>("device_path",cfg["port"].as<std::string>());
  config.baud_rate=cfg["baud_rate"].as<unsigned>();
  const auto directions=cfg["joint_directions"].as<std::vector<float>>();if(directions.size()!=7)throw std::runtime_error("Seven SDK directions required");
  for(int i=0;i<7;++i)config.motors[i].direction=directions[i];
  // Load saved calibration exactly once; conversion remains inside the SDK.
  if(cfg["joint_zero_offsets"]){auto z=cfg["joint_zero_offsets"].as<std::vector<float>>();if(z.size()!=7)throw std::runtime_error("Seven existing offsets required");for(int i=0;i<7;++i)config.motors[i].zero_offset=z[i];}
  auto seven=[&](const char* key,auto& out){auto v=cfg[key].as<std::vector<float>>();if(v.size()!=7)throw std::runtime_error("Seven SDK control values required");std::copy(v.begin(),v.end(),out.begin());};
  seven("position_kp",config.default_kp);seven("position_kd",config.default_kd);seven("position_velocity_limits_rad_s",config.position_velocity_limits);
  auto modes=cfg["control_modes"].as<std::vector<std::string>>();if(modes.size()!=7)throw std::runtime_error("Seven SDK modes required");
  for(int i=0;i<7;++i){if(modes[i]=="pos_vel")config.control_modes[i]=rars_arm::ArmControlMode::PositionVelocity;else if(modes[i]=="mit")config.control_modes[i]=rars_arm::ArmControlMode::MIT;else throw std::runtime_error("Unknown SDK mode");}
  config.command_rate_hz=cfg["command_rate_hz"].as<float>();
  if(auto_cfg["command_rate_hz"])config.command_rate_hz=auto_cfg["command_rate_hz"].as<float>();
  config.feedback_watchdog_enabled=true;
  config.feedback_timeout=std::chrono::milliseconds(int(deployment["feedback_timeout_s"].as<double>()*1000));
  if(cfg["initial_feedback_grace_ms"])config.initial_feedback_grace=std::chrono::milliseconds(cfg["initial_feedback_grace_ms"].as<int>());
  connect_=declare_parameter<bool>("connect_serial",false);
  const bool read_only=declare_parameter<bool>("read_only",true);
  AutoHomeConfig home;home.enabled=auto_cfg["enabled"].as<bool>()&&!read_only;
  home.emergency_disable_validated=deployment["emergency_disable_validated"]&&deployment["emergency_disable_validated"].as<bool>();
  home.startup_delay_s=auto_cfg["startup_delay_s"].as<double>();home.command_rate_hz=config.command_rate_hz;
  home.home_tolerance_rad=auto_cfg["home_tolerance_rad"].as<double>();
  home.feedback_timeout_s=deployment["feedback_timeout_s"].as<double>();home.target_timeout_s=deployment["target_timeout_s"].as<double>();
  home.enable_feedback_grace_s=config.initial_feedback_grace.count()/1000.;
  if(cfg["port_retry_interval_s"])home.connect_retry_s=cfg["port_retry_interval_s"].as<double>();
  const auto target=auto_cfg["home_target"].as<std::vector<float>>();if(target.size()!=7)throw std::runtime_error("Seven HOME targets required");std::copy(target.begin(),target.end(),home.home_target.begin());
  if(home.enabled&&!connect_)throw std::runtime_error("AUTO HOME requires explicit connect_serial=true");
  const auto lockdir=declare_parameter<std::string>("lock_directory","");
  if(lockdir.empty())throw std::runtime_error("Shared serial lease directory required");
  std::filesystem::create_directories(lockdir);
  // A deployment-wide owner lock remains stable while /dev/by-id appears or
  // changes its symlink target. Also retain the cooperating per-device lease.
  owner_lease_=std::make_unique<SerialOwnerLease>(lockdir,"/rars01-single-owner");
  lease_=std::make_unique<SerialOwnerLease>(lockdir,config.port_name);
  if(!owner_lease_->acquired()||!lease_->acquired())throw std::runtime_error("Serial owner lease busy");
  const auto journal=declare_parameter<std::string>("journal_path","");
  if(journal.empty())throw std::runtime_error("Persistent journal path required");
  std::filesystem::create_directories(std::filesystem::path(journal).parent_path());
  journal_=std::make_unique<FileAutoHomeJournal>(journal,BootId());
  sdk_=std::make_unique<rars_arm::RarsArm>(config);transport_=std::make_unique<SdkHomeTransport>(*sdk_);
  home_=std::make_unique<AutoHomeController>(home,*transport_,*journal_);
  state_=create_publisher<std_msgs::msg::String>("/rars01/commissioning/state",10);
  return_home_=create_service<std_srvs::srv::Trigger>("/rars01/control/return_home",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
   response->success=home_->ReturnHome(ArmClock::now());response->message=response->success?"HOME stream accepted; completion requires measured HOME":"healthy enabled owner/feedback/target required; no enable issued";
  });
  emergency_=create_service<std_srvs::srv::Trigger>("/rars01/control/emergency_disable",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
   response->success=home_->EmergencyDisable();response->message=response->success?"disable sent; physical disable requires measured confirmation":"emergency disable rejected: bench validation missing, read-only, or SDK failure";
  });
  timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1/config.command_rate_hz)),[this]{Poll();});
  RCLCPP_INFO(get_logger(),"RARS owner: device=%s read_only=%d delay=%.3fs rate=%.3fHz HOME=[0,0,0,0,0,0,0]",config.port_name.c_str(),read_only,home.startup_delay_s,home.command_rate_hz);
  for(size_t i=0;i<7;++i)RCLCPP_INFO(get_logger(),"saved calibration motor%zu direction=%g offset=%g",i+1,config.motors[i].direction,config.motors[i].zero_offset);
 }
 private:
 void Poll(){
  // ROS already schedules this callback at ArmConfiguration.command_rate_hz.
  // A second deadline gate would drop early callbacks under normal jitter.
  try{home_->Tick(ArmClock::now(),connect_,true);}catch(const std::exception& e){home_->Fault(std::string("owner_exception: ")+e.what());}
  const auto& s=home_->status();
  if(s.state!=last_state_){RCLCPP_INFO(get_logger(),"arm state=%s error=%s",AutoHomeStateName(s.state),s.last_error.c_str());last_state_=s.state;}
  YAML::Emitter e;e<<YAML::Flow<<YAML::BeginMap
   <<YAML::Key<<"observed_ns"<<YAML::Value<<Ns(ArmClock::now())
   <<YAML::Key<<"owner_state"<<YAML::Value<<AutoHomeStateName(s.state)
   <<YAML::Key<<"connected"<<YAML::Value<<s.communication.connected
   <<YAML::Key<<"enabled_local"<<YAML::Value<<s.communication.enabled
   <<YAML::Key<<"enable_attempted"<<YAML::Value<<s.enable_attempted
   <<YAML::Key<<"command_rate_hz"<<YAML::Value<<home_->config().command_rate_hz
   <<YAML::Key<<"successful_sends"<<YAML::Value<<s.successful_sends
   <<YAML::Key<<"first_accepted_ns"<<YAML::Value<<(s.first_accepted_stamp?Ns(*s.first_accepted_stamp):0)
   <<YAML::Key<<"last_send_gap_ms"<<YAML::Value<<s.last_send_gap_ms
   <<YAML::Key<<"max_send_gap_ms"<<YAML::Value<<s.max_send_gap_ms
   <<YAML::Key<<"frame_received_ns"<<YAML::Value<<(s.feedback_stamp?Ns(*s.feedback_stamp):0)
   <<YAML::Key<<"common_feedback_age_s"<<YAML::Value<<s.feedback_age_s
   <<YAML::Key<<"feedback_age_s"<<YAML::Value<<s.feedback_age_s
   <<YAML::Key<<"measured_q"<<YAML::Value<<std::vector<float>(s.joints.position.begin(),s.joints.position.begin()+6)
   <<YAML::Key<<"measured_dq"<<YAML::Value<<std::vector<float>(s.joints.velocity.begin(),s.joints.velocity.begin()+6)
   <<YAML::Key<<"motor_ids"<<YAML::Value<<std::vector<int>(s.joints.motor_id.begin(),s.joints.motor_id.begin()+6)
   <<YAML::Key<<"valid"<<YAML::Value<<std::vector<bool>(s.joints.valid.begin(),s.joints.valid.begin()+6)
   <<YAML::Key<<"motor_id"<<YAML::Value<<std::vector<int>(s.joints.motor_id.begin(),s.joints.motor_id.end())
   <<YAML::Key<<"motor_status"<<YAML::Value<<std::vector<int>(s.joints.error.begin(),s.joints.error.end())
   <<YAML::Key<<"valid7"<<YAML::Value<<Values(s.joints.valid)
   <<YAML::Key<<"q"<<YAML::Value<<Values(s.joints.position)<<YAML::Key<<"dq"<<YAML::Value<<Values(s.joints.velocity)
   <<YAML::Key<<"home_error"<<YAML::Value<<Values(s.home_error)
   <<YAML::Key<<"home_target"<<YAML::Value<<Values(home_->config().home_target)
   <<YAML::Key<<"feedback_ready"<<YAML::Value<<s.feedback_ready
   <<YAML::Key<<"motors_enabled"<<YAML::Value<<s.motors_enabled
   <<YAML::Key<<"static_hold"<<YAML::Value<<s.arm_home_ready
   <<YAML::Key<<"arm_emergency_validated"<<YAML::Value<<home_->config().emergency_disable_validated
   <<YAML::Key<<"arm_home_ready"<<YAML::Value<<s.arm_home_ready
   <<YAML::Key<<"watchdog_armed"<<YAML::Value<<s.communication.watchdog_armed
   <<YAML::Key<<"watchdog_tripped"<<YAML::Value<<s.communication.watchdog_tripped
   <<YAML::Key<<"stm32_watchdog_tripped"<<YAML::Value<<s.communication.stm32_watchdog_tripped
   <<YAML::Key<<"protocol_v2_detected"<<YAML::Value<<s.communication.protocol_v2_detected
   <<YAML::Key<<"per_joint_freshness_proven"<<YAML::Value<<false
   <<YAML::Key<<"target_age_s"<<YAML::Value<<s.target_age_s
   <<YAML::Key<<"target_valid"<<YAML::Value<<s.target_fresh
   <<YAML::Key<<"last_error"<<YAML::Value<<s.last_error;
  if(s.accepted_stamp)e<<YAML::Key<<"accepted_target"<<YAML::Value<<std::vector<float>(home_->config().home_target.begin(),home_->config().home_target.begin()+6)
   <<YAML::Key<<"accepted_ns"<<YAML::Value<<Ns(*s.accepted_stamp);
  e<<YAML::EndMap;std_msgs::msg::String msg;msg.data=e.c_str();state_->publish(msg);
 }
 bool connect_=false;AutoHomeState last_state_=AutoHomeState::READ_ONLY;
 std::unique_ptr<SerialOwnerLease> owner_lease_,lease_;
 std::unique_ptr<FileAutoHomeJournal> journal_;
 std::unique_ptr<rars_arm::RarsArm> sdk_;std::unique_ptr<SdkHomeTransport> transport_;
 std::unique_ptr<AutoHomeController> home_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_,emergency_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<ArmOwner>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("rars_r3"),"Failed closed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
