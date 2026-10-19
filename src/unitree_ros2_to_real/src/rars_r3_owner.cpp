#include "rars_auto_home.hpp"
#include "rars_mission_stream.hpp"
#include <regex>
#include <sstream>
#include <future>
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
 bool SendConfigured(const rars_arm::RarsArm::MotorValues& q,const rars_arm::RarsArm::MotorValues& kp,const rars_arm::RarsArm::MotorValues& kd,const rars_arm::RarsArm::MotorValues& tau) override{rars_arm::RarsArm::MotorValues zero{};return sdk_.sendConfigured(q,zero,kp,kd,tau);}
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
  home.emergency_disable_enabled=deployment["emergency_disable_enabled"]?deployment["emergency_disable_enabled"].as<bool>():home.emergency_disable_validated;
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
  owner_session_=std::to_string(Ns(ArmClock::now()));
  const auto mission_path=declare_parameter<std::string>("mission_config_path","");
  const auto workspace=declare_parameter<std::string>("workspace","");
  if(!mission_path.empty())ConfigureMission(mission_path,workspace,config,path,read_only);
  cancel_=create_service<std_srvs::srv::Trigger>("/rars01/mission/cancel_manipulation",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
   plan_cancelled_ns_=Ns(ArmClock::now());
   if(mission_)mission_->Cancel(home_->status(),"operator_cancel");
   // Revoke this coordinator session: queued plans and a late inference result
   // cannot restart the owner after X/B. A new operator session is required.
   if(!mission_session_.empty())retired_session_=mission_session_;
   response->success=true;response->message="owner advancement cancelled; gripper hold preserved; no HOME issued";
  });
  state_=create_publisher<std_msgs::msg::String>("/rars01/commissioning/state",10);
  return_home_=create_service<std_srvs::srv::Trigger>("/rars01/control/return_home",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
   home_requested_=bool(mission_);
   response->success=mission_?(home_->status().feedback_ready&&home_->status().motors_enabled&&home_->status().target_fresh):home_->ReturnHome(ArmClock::now());response->message=response->success?"HOME stream accepted; completion requires measured HOME":"healthy enabled owner/feedback/target required; no enable issued";
  });
  emergency_=create_service<std_srvs::srv::Trigger>("/rars01/control/emergency_disable",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
   response->success=home_->EmergencyDisable();response->message=response->success?"disable sent; physical disable requires measured confirmation":"emergency disable rejected: bench validation missing, read-only, or SDK failure";
  });
  timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1/config.command_rate_hz)),[this]{Poll();});
  RCLCPP_INFO(get_logger(),"RARS owner: device=%s read_only=%d delay=%.3fs rate=%.3fHz HOME=[0,0,0,0,0,0,0]",config.port_name.c_str(),read_only,home.startup_delay_s,home.command_rate_hz);
  for(size_t i=0;i<7;++i)RCLCPP_INFO(get_logger(),"saved calibration motor%zu direction=%g offset=%g",i+1,config.motors[i].direction,config.motors[i].zero_offset);
 }
 private:
 bool Authority(ArmTime now,bool home_request=false){
  const auto fresh=[&](ArmTime stamp){return stamp!=ArmTime{}&&std::chrono::duration<double>(now-stamp).count()>=0&&std::chrono::duration<double>(now-stamp).count()<=mission_ttl_;};
  return mission_&&home_->status().feedback_age_s>=0&&home_->status().feedback_age_s<=mission_feedback_ttl_&&fresh(r3_stamp_)&&fresh(nav_stamp_)&&fresh(health_stamp_)&&r3_authority_&&nav_ready_&&health_ready_&&
   !mission_session_.empty()&&(mission_session_!=retired_session_||(home_request&&r3_stopping_))&&nav_session_==health_nav_session_&&r3_nav_session_==nav_session_&&
   count_publishers("/go2/locomotion_status")==1&&count_publishers("/go2/mission/readiness")==1&&count_publishers("/go2/mission/health")==1&&
   (r3_active_&&health_goal_revision_>0&&health_manipulating_||(home_request&&r3_stopping_));
 }
 void ConfigureMission(const std::string& path,const std::string& workspace,const rars_arm::ArmConfiguration& sdk_config,const std::string&,bool read_only){
  const auto y=YAML::LoadFile(path)["go2_mission"];mission_ttl_=y["execution"]["heartbeat_ttl_s"].as<double>();
  if(!y["physical_enabled"].as<bool>()||read_only){mission_blocker_="mission_physical_disabled";return;}
  auto absolute=[&](const std::string& name){return (std::filesystem::path(workspace)/y["paths"][name].as<std::string>()).string();};
  // Mission evidence is not an owner admission protocol. The owner enforces
  // live authority, feedback, trajectory bounds and the existing stop FSM.
  if(y["frames"]["vehicle_to_mount"].IsNull())throw std::runtime_error("mission mount transform missing");
  const auto mode=y["test_mode"]?y["test_mode"].as<std::string>():"full";
  if(mode!="full"&&mode!="ready_observe")throw std::runtime_error("invalid test_mode");
  ready_observe_=mode=="ready_observe";
  const auto grasp=YAML::LoadFile(absolute("sdk_config"));const auto g=grasp["robot"]["gripper"]["rars01"];
  const auto feedback=grasp["robot"]["motion"]["motion_feedback"];mission_feedback_ttl_=feedback["max_feedback_age_s"].as<double>();
  if(y["grasp_verification"]["mode"].as<std::string>()!="stub")throw std::runtime_error("unsupported grasp verification mode");
  payload_checks_enabled_=y["collision"]["payload_enabled"]?y["collision"]["payload_enabled"].as<bool>():true;
  MissionStreamConfig c;c.close_duration_s=y["grasp_verification"]["close_duration_s"].as<double>();c.gripper_velocity=sdk_config.position_velocity_limits[6];c.kp=sdk_config.default_kp;c.kd=sdk_config.default_kd;c.rate=sdk_config.command_rate_hz;
  c.start_tolerance=y["execution"]["start_tolerance_rad"].as<double>();c.reached_tolerance=y["execution"]["joint_reached_tolerance_rad"].as<double>();
  c.max_duration_s=y["execution"]["max_plan_duration_s"].as<double>();
  max_plan_samples_=size_t(c.rate*c.max_duration_s)+1;
  c.settled_velocity=feedback["velocity_tolerance_deg_s"].as<double>()*std::acos(-1.)/180;
  c.settle_s=feedback["settle_time_s"].as<double>();c.completion_timeout_s=feedback["timeout_margin_s"].as<double>();
  c.closed=g["closed_position_rad"].as<double>();c.open=g["angle_open"].as<double>();
  const double close_sign=g["counterclockwise"].as<bool>()?1:-1;
  c.close_torque=close_sign*g["close_torque"].as<double>();c.hold_torque=close_sign*g["default_force"].as<double>();
  c.move_kp=g["move_kp"].as<double>();c.move_kd=g["move_kd"].as<double>();c.hold_kp=g["hold_kp"].as<double>();c.hold_kd=g["hold_kd"].as<double>();c.close_kd=g["close_kd"].as<double>();
  c.hard_stop=g["hard_stop_angle_rad"].as<double>();
  if(g["hold_ramp_s"].as<double>()!=0)throw std::runtime_error("unsupported gripper hold ramp; do not silently alter it");
  // Read authoritative URDF joint bounds without copying them to mission YAML.
  std::ifstream f(absolute("arm_urdf"));std::stringstream buffer;buffer<<f.rdbuf();const auto xml=buffer.str();
  for(int j=0;j<6;++j){
   std::smatch joint,limit,lo,hi;
   if(!std::regex_search(xml,joint,std::regex("<joint\\s+name=\"joint"+std::to_string(j+1)+"\"[\\s\\S]*?</joint>")))throw std::runtime_error("URDF joint limit missing");
   const auto joint_text=joint.str();if(!std::regex_search(joint_text,limit,std::regex("<limit[^>]*>")))throw std::runtime_error("URDF limit missing");
   const auto text=limit.str();
   if(!std::regex_search(text,lo,std::regex("lower=\"([^\"]+)\""))||!std::regex_search(text,hi,std::regex("upper=\"([^\"]+)\"")))throw std::runtime_error("URDF bounds missing");
   c.lower[j]=std::stod(lo[1]);c.upper[j]=std::stod(hi[1]);c.velocity[j]=std::min(double(sdk_config.position_velocity_limits[j]),y["execution"]["arm_speed_rad_s"].as<double>());
  }
  mission_=std::make_unique<MissionStream>(c);
  home_->SetMissionProvider([this](ArmTime now,const AutoHomeStatus& status)->std::optional<AutoHomeController::Command>{
   if(!mission_->has_command())return std::nullopt;
   return mission_->Tick(now,status,Authority(now,home_requested_||active_home_));
  });
  r3_sub_=create_subscription<std_msgs::msg::String>("/go2/locomotion_status",1,[this](std_msgs::msg::String::SharedPtr msg){
   r3_authority_=false;try{auto v=YAML::Load(msg->data);const auto ns=v["observed_ns"].as<int64_t>();
    if(ns<=Ns(r3_stamp_))return;r3_stamp_=ArmTime(std::chrono::nanoseconds(ns));
    r3_nav_session_=v["navigation_session_id"].as<std::string>();
    const auto session=v["controller_session_id"].as<std::string>();
    if(!r3_session_.empty()&&r3_session_!=session){mission_->Cancel(home_->status(),"controller_session_changed");retired_session_=mission_session_;}
    r3_session_=session;r3_active_=v["state"].as<std::string>()=="ACTIVE"&&v["phase"].as<std::string>()=="RL_ZERO";
    r3_stopping_=v["state"].as<std::string>()=="CONTROLLED_STOP";
    auto command=v["motion_command"].as<std::vector<double>>();bool zero=command.size()==3;for(double x:command)zero=zero&&std::isfinite(x)&&std::abs(x)<1e-6;
    r3_authority_=v["operation_profile"].as<std::string>()=="full_mission"&&v["capabilities"]["arm_motion"].as<bool>()&&!v["fault_latched"].as<bool>()&&zero;
   }catch(...){r3_authority_=false;}
  });
  nav_sub_=create_subscription<std_msgs::msg::String>("/go2/mission/readiness",1,[this](std_msgs::msg::String::SharedPtr msg){
   nav_ready_=false;try{auto v=YAML::Load(msg->data);const auto ns=v["observed_ns"].as<int64_t>();if(ns<=Ns(nav_stamp_))return;
    nav_stamp_=ArmTime(std::chrono::nanoseconds(ns));nav_session_=v["session_id"].as<std::string>();nav_ready_=v["navigation_ready"].as<bool>();
   }catch(...){nav_ready_=false;}
  });
  health_sub_=create_subscription<std_msgs::msg::String>("/go2/mission/health",1,[this](std_msgs::msg::String::SharedPtr msg){
   health_ready_=false;try{auto v=YAML::Load(msg->data);const auto ns=v["observed_ns"].as<int64_t>();if(ns<=Ns(health_stamp_))return;
    health_stamp_=ArmTime(std::chrono::nanoseconds(ns));const auto session=v["session_id"].as<std::string>();
    if(!mission_session_.empty()&&mission_session_!=session){mission_->Cancel(home_->status(),"mission_session_changed");retired_session_=mission_session_;}
    mission_session_=session;health_nav_session_=v["navigation_session_id"].as<std::string>();health_ready_=v["ready"].as<bool>();health_goal_revision_=v["goal_revision"].as<uint64_t>();
    const auto phase=v["phase"].as<std::string>();
    health_manipulating_=phase=="BASE_SETTLE"||phase=="ARM_SCAN_POSE"||phase=="ARM_SEARCH"||phase=="VIEW_REFINEMENT"||phase=="PLAN_GRASP"||phase=="PREGRASP"||phase=="GRASP"||phase=="RETREAT"||phase=="RETURN_HOME";
   }catch(...){health_ready_=false;}
  });
  plan_sub_=create_subscription<std_msgs::msg::String>("/rars01/mission/trajectory",1,[this](std_msgs::msg::String::SharedPtr msg){
   if(msg->data.size()>max_plan_samples_*256+65536||plan_future_.valid()){mission_blocker_="plan too large or parser busy";return;}
   // YAML/arrays can take longer than an arm command slot. Decode on a separate
   // OTHER thread. The ROS owner remains responsive to cancel/emergency/timer.
   plan_future_=std::async(std::launch::async,[msg,limit=max_plan_samples_]{
    ParsedPlan result;try{
     auto v=YAML::Load(msg->data);auto samples=v["points"].as<std::vector<std::vector<float>>>();
     if(samples.size()>limit)throw std::runtime_error("plan sample cap exceeded");
     result.points.reserve(samples.size());
     for(const auto& q:samples){if(q.size()!=7)throw std::runtime_error("seven target values required");std::array<float,7> p;std::copy(q.begin(),q.end(),p.begin());result.points.push_back(p);}
     v.remove("points");result.header=v;
    }catch(const std::exception& e){result.error=e.what();}return result;
   });
  });
 }
 void AdmitPlan(){
  if(!mission_||!plan_future_.valid()||plan_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
  auto result=plan_future_.get();try{
   if(!result.error.empty())throw std::runtime_error(result.error);
   const auto& v=result.header;const auto now=ArmClock::now();const bool is_home=v["kind"].as<std::string>()=="home";
   // Heartbeat TTL belongs to live Authority(), not parsing/planning duration.
   // Recheck cancellation, sessions, current feedback and start pose on admission.
   if(!PlanTimestampAllowed(v["observed_ns"].as<int64_t>(),Ns(now),plan_cancelled_ns_))
      throw std::runtime_error("plan timestamp invalid or predates cancellation");
   if(!Authority(now,is_home)||count_publishers("/rars01/mission/trajectory")!=1||
      v["session_id"].as<std::string>()!=mission_session_||v["owner_session_id"].as<std::string>()!=owner_session_||
      v["navigation_session_id"].as<std::string>()!=nav_session_||v["goal_revision"].as<uint64_t>()!=health_goal_revision_||!v["collision_checked"].as<bool>()||
      v["payload_checked"].as<bool>()!=(payload_checks_enabled_&&mission_->holding()))throw std::runtime_error("plan authority/session/collision rejected");
   const auto kind=v["kind"].as<std::string>();
   if(!ReadyObservePlanAllowed(ready_observe_,ready_move_seen_,kind))throw std::runtime_error("ready_observe permits one initial move and HOME only");
   if(!mission_->Start(v["id"].as<uint64_t>(),v["kind"].as<std::string>(),result.points,now,home_->status()))throw std::runtime_error("trajectory bounds/start/hold rejected");
   if(ready_observe_&&kind=="move")ready_move_seen_=true;
   mission_blocker_.clear();active_home_=is_home;if(is_home)home_requested_=false;
  }catch(const std::exception& e){mission_blocker_=e.what();}
 }
 void Poll(){
  AdmitPlan();
  // ROS already schedules this callback at ArmConfiguration.command_rate_hz.
  // A second deadline gate would drop early callbacks under normal jitter.
  try{home_->Tick(ArmClock::now(),connect_,true);}catch(const std::exception& e){home_->Fault(std::string("owner_exception: ")+e.what());}
  const auto& s=home_->status();
  if(s.state!=last_state_){RCLCPP_INFO(get_logger(),"arm state=%s error=%s",AutoHomeStateName(s.state),s.last_error.c_str());last_state_=s.state;}
  YAML::Emitter e;e<<YAML::Flow<<YAML::BeginMap
   <<YAML::Key<<"observed_ns"<<YAML::Value<<Ns(ArmClock::now())
   <<YAML::Key<<"owner_session_id"<<YAML::Value<<owner_session_
   <<YAML::Key<<"mission_capable"<<YAML::Value<<bool(mission_)
   <<YAML::Key<<"mission_id"<<YAML::Value<<(mission_?mission_->id():0)
   <<YAML::Key<<"mission_state"<<YAML::Value<<(mission_?mission_->state():"DISABLED")
   <<YAML::Key<<"mission_error"<<YAML::Value<<(mission_?mission_->error():mission_blocker_)
   <<YAML::Key<<"mission_blocker"<<YAML::Value<<mission_blocker_
   <<YAML::Key<<"home_requested"<<YAML::Value<<home_requested_
   <<YAML::Key<<"grasp_verification"<<YAML::Value<<(mission_?"stub":"inactive")
   <<YAML::Key<<"object_grasped"<<YAML::Value<<YAML::Null
   <<YAML::Key<<"home_joint_count"<<YAML::Value<<(mission_?6:7)
   <<YAML::Key<<"gripper_command_active"<<YAML::Value<<(mission_&&mission_->holding())
   <<YAML::Key<<"gripper_hold_confirmed"<<YAML::Value<<(mission_&&mission_->HoldConfirmed(s))
   <<YAML::Key<<"gripper_command_torque_nm"<<YAML::Value<<(mission_?mission_->command().torque[6]:0)
   <<YAML::Key<<"gripper_hold_target"<<YAML::Value<<(mission_?mission_->command().q[6]:0)
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
   <<YAML::Key<<"torque"<<YAML::Value<<Values(s.joints.torque)
   <<YAML::Key<<"home_error"<<YAML::Value<<Values(s.home_error)
   <<YAML::Key<<"home_target"<<YAML::Value<<Values(home_->config().home_target)
   <<YAML::Key<<"feedback_ready"<<YAML::Value<<s.feedback_ready
   <<YAML::Key<<"motors_enabled"<<YAML::Value<<s.motors_enabled
   <<YAML::Key<<"static_hold"<<YAML::Value<<(mission_?mission_->home6(s,home_->config().home_tolerance_rad):s.arm_home_ready)
   <<YAML::Key<<"arm_emergency_disable_enabled"<<YAML::Value<<home_->config().emergency_disable_enabled
   <<YAML::Key<<"arm_emergency_validated"<<YAML::Value<<home_->config().emergency_disable_validated
   <<YAML::Key<<"arm_home_ready"<<YAML::Value<<(mission_?mission_->home6(s,home_->config().home_tolerance_rad):s.arm_home_ready)
   <<YAML::Key<<"watchdog_armed"<<YAML::Value<<s.communication.watchdog_armed
   <<YAML::Key<<"watchdog_tripped"<<YAML::Value<<s.communication.watchdog_tripped
   <<YAML::Key<<"stm32_watchdog_tripped"<<YAML::Value<<s.communication.stm32_watchdog_tripped
   <<YAML::Key<<"protocol_v2_detected"<<YAML::Value<<s.communication.protocol_v2_detected
   <<YAML::Key<<"per_joint_freshness_proven"<<YAML::Value<<false
   <<YAML::Key<<"target_age_s"<<YAML::Value<<s.target_age_s
   <<YAML::Key<<"target_valid"<<YAML::Value<<s.target_fresh
   <<YAML::Key<<"last_error"<<YAML::Value<<s.last_error;
  if(s.accepted_stamp)e<<YAML::Key<<"accepted_target"<<YAML::Value<<std::vector<float>(home_->accepted_target().begin(),home_->accepted_target().begin()+6)
   <<YAML::Key<<"accepted_ns"<<YAML::Value<<Ns(*s.accepted_stamp);
  e<<YAML::EndMap;std_msgs::msg::String msg;msg.data=e.c_str();state_->publish(msg);
 }
 struct ParsedPlan{YAML::Node header;std::vector<std::array<float,7>> points;std::string error;};
 std::future<ParsedPlan> plan_future_;
 int64_t plan_cancelled_ns_=0;
 bool ready_observe_=false,ready_move_seen_=false;
 size_t max_plan_samples_=0;
 std::unique_ptr<MissionStream> mission_;std::string owner_session_,mission_session_,retired_session_,nav_session_,health_nav_session_,r3_session_,r3_nav_session_,mission_blocker_;
 uint64_t health_goal_revision_=0;bool health_manipulating_=false;
 double mission_feedback_ttl_=.1,mission_ttl_=.25;bool payload_checks_enabled_=true,r3_authority_=false,r3_active_=false,r3_stopping_=false,nav_ready_=false,health_ready_=false,home_requested_=false,active_home_=false;
 ArmTime r3_stamp_{},nav_stamp_{},health_stamp_{};
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr r3_sub_,nav_sub_,health_sub_,plan_sub_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_;
 bool connect_=false;AutoHomeState last_state_=AutoHomeState::READ_ONLY;
 std::unique_ptr<SerialOwnerLease> owner_lease_,lease_;
 std::unique_ptr<FileAutoHomeJournal> journal_;
 std::unique_ptr<rars_arm::RarsArm> sdk_;std::unique_ptr<SdkHomeTransport> transport_;
 std::unique_ptr<AutoHomeController> home_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_,emergency_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<ArmOwner>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("rars_r3"),"Failed closed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
