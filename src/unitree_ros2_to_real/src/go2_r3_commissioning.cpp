#include "r3_commissioning.hpp"
#include "real_controller_core.hpp"
#include "output_lease.hpp"
#include "sdk_mode_process.hpp"
#include <future>
#include <ATen/Parallel.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <functional>
#include <filesystem>
#include <cmath>
#include <algorithm>
using namespace sim2real;
namespace {
SafetyTime TimeNs(int64_t ns){return SafetyTime(std::chrono::nanoseconds(ns));}
double Age(SafetyTime now,SafetyTime stamp){return std::chrono::duration<double>(now-stamp).count();}
std::array<float,6> Six(const YAML::Node& n){auto v=n.as<std::vector<float>>();if(v.size()!=6)throw std::runtime_error("Exactly six arm values required");std::array<float,6> a;std::copy(v.begin(),v.end(),a.begin());for(float x:a)if(!std::isfinite(x))throw std::runtime_error("Invalid arm values");return a;}
std::string FloatList(const std::array<float,12>& values){YAML::Emitter e;e<<YAML::Flow<<std::vector<float>(values.begin(),values.end());return e.c_str();}
}
class R3Node final:public rclcpp::Node {
 public:
 R3Node():Node("go2_r3_commissioning") {
  at::set_num_threads(1);at::set_num_interop_threads(1);
  rcl_interfaces::msg::ParameterDescriptor immutable;immutable.read_only=true;
  const auto config=declare_parameter<std::string>("config_path","",immutable);
  const auto model=declare_parameter<std::string>("model_path","",immutable);
  const auto profile_name=declare_parameter<std::string>("operation_profile","",immutable);
  std::optional<OperationProfile> explicit_profile;
  if(!profile_name.empty())explicit_profile=ParseOperationProfile(profile_name);
  read_only_=declare_parameter<bool>("read_only",explicit_profile?!ResolveOperationCapabilities(*explicit_profile).physical_leg_output:true,immutable);
  automatic_sequence_=declare_parameter<bool>("remote_auto_sequence",true,immutable);
  const bool legacy_remote=declare_parameter<bool>("remote_test_mode",false,immutable);
  control_mode_=declare_parameter<std::string>("control_mode","",immutable);
  if(control_mode_.empty())control_mode_=explicit_profile&&ResolveOperationCapabilities(*explicit_profile).command_source==CommandSource::NAVIGATION?"autonomy":"remote_test";
  if(control_mode_!="remote_test"&&control_mode_!="autonomy")throw std::runtime_error("control_mode must be remote_test or autonomy");
  motion_commands_enabled_=declare_parameter<bool>("motion_commands_enabled",explicit_profile?ResolveOperationCapabilities(*explicit_profile).allow_nonzero_velocity:legacy_remote,immutable);
  const auto operation=explicit_profile.value_or(LegacyOperationProfile(read_only_,control_mode_,motion_commands_enabled_));
  const auto capabilities=ResolveOperationCapabilities(operation);
  if(explicit_profile) {
   if(read_only_!=!capabilities.physical_leg_output||motion_commands_enabled_!=capabilities.allow_nonzero_velocity||
      (capabilities.command_source==CommandSource::NAVIGATION&&control_mode_!="autonomy")||
      (capabilities.command_source==CommandSource::REMOTE&&control_mode_!="remote_test")||
      (legacy_remote&&capabilities.command_source!=CommandSource::REMOTE))throw std::runtime_error("Legacy flags conflict with immutable operation_profile");
  }
  remote_test_mode_=control_mode_=="remote_test"&&motion_commands_enabled_;
  network_interface_=declare_parameter<std::string>("network_interface","",immutable);
  sdk_helper_=(std::filesystem::read_symlink("/proc/self/exe").parent_path()/"go2_mode_switch").string();
  if(declare_parameter<bool>("enable_actuator_output",false,immutable))throw std::runtime_error("startup output must be false; use explicit service gate");
  core_.Load(config,model);const auto y=YAML::LoadFile(config),d=y["real_deployment"],r=d["r3_commissioning"];
  // Warm TorchScript before any subscriptions, output publisher or takeover.
  // Discard all warm-up targets; the real RL entry resets history from feedback.
  {
   auto& a=core_.agent();a.obs.dof_pos=a.params.default_dof_pos.clone();a.ResetPolicyState();
   for(int i=0;i<20;++i){const auto q=a.Act();
    if(q.numel()!=12||!torch::isfinite(q).all().item<bool>())throw std::runtime_error("Policy warm-up returned invalid targets");
   }
   a.ResetPolicyState();
  }
  home_tolerance_=d["rars01"]["auto_home"]["home_tolerance_rad"].as<double>();
  if(!std::isfinite(home_tolerance_)||home_tolerance_<=0)throw std::runtime_error("Invalid HOME tolerance");
  auto profile=LoadR3Profile(y);
  profile.controlled_stop_lie_down_trial=declare_parameter<bool>("controlled_stop_lie_down_trial",profile.controlled_stop_lie_down_trial,immutable);
  supervisor_=std::make_unique<R3Supervisor>(profile,operation);
  lowcmd_wait_=std::make_unique<LowCmdDiscoveryWait>(profile.lowcmd_discovery_timeout_s,profile.lowcmd_clear_duration_s);
  lowstate_=LowStateReader(profile.lowstate_timeout_s);
  remote_=std::make_unique<R3RemoteCommands>(r["remote"]["controlled_abort_chord"].as<std::vector<std::string>>(),r["remote"]["emergency_chord"].as<std::vector<std::string>>(),d["remote"]["takeover_hold_s"].as<double>(),profile.remote_timeout_s);
  output_lock_=r["output_lock"].as<std::string>();
  inputs_.ready.model_loaded=core_.loaded();inputs_.ready.config_valid=true;inputs_.ready.transport_ready=!read_only_;
  status_=create_publisher<std_msgs::msg::String>("/go2/locomotion_status",10);
  low_sub_=create_subscription<unitree_go::msg::LowState>("/lowstate",rclcpp::SensorDataQoS(),[this](unitree_go::msg::LowState::ConstSharedPtr m){
   std::lock_guard lock(mutex_);const auto now=SafetyClock::now();
   const bool valid=lowstate_.Receive(*m,now);remote_->Receive(m->wireless_remote,now);
   inputs_.lowstate_stamp=now;inputs_.remote_stamp=now;inputs_.ready.motor_state_valid=valid;
   if(valid)inputs_.measured_q=lowstate_.snapshot().motor_q;
   else if(supervisor_->output_enabled())supervisor_->Fault("invalid_lowstate",now);
  });
  sport_sub_=create_subscription<std_msgs::msg::String>("/go2/commissioning/sport_observation",10,[this](std_msgs::msg::String::ConstSharedPtr m){
   std::lock_guard lock(mutex_);try{auto y=YAML::Load(m->data);inputs_.sport=ParseSportMode(y["state"].as<std::string>());inputs_.sport_stamp=TimeNs(y["observed_ns"].as<int64_t>());}
   catch(...){inputs_.sport=SportMode::ERROR;inputs_.sport_stamp=SafetyTime{};}
  });
  mission_readiness_sub_=create_subscription<std_msgs::msg::String>("/go2/mission/readiness",10,[this](std_msgs::msg::String::ConstSharedPtr message){
   std::lock_guard lock(mutex_);mission_navigation_ready_=mission_perception_ready_=false;mission_session_.clear();
   try{auto y=YAML::Load(message->data);mission_readiness_stamp_=TimeNs(y["observed_ns"].as<int64_t>());mission_session_=y["session_id"].as<std::string>();
    if(mission_session_.empty())throw std::runtime_error("mission adapter session required");
    mission_navigation_ready_=y["navigation_ready"].as<bool>();mission_perception_ready_=y["perception_ready"].as<bool>();
   }catch(...){mission_readiness_stamp_={};}
  });
  arm_home_client_=create_client<std_srvs::srv::Trigger>("/rars01/control/return_home");
  arm_emergency_client_=create_client<std_srvs::srv::Trigger>("/rars01/control/emergency_disable");
  nav_cancel_client_=create_client<std_srvs::srv::Trigger>("/go2/mission/cancel_navigation");
  manipulation_cancel_client_=create_client<std_srvs::srv::Trigger>("/rars01/mission/cancel_manipulation");
  port_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  port_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{PortTick();},port_group_);
  arm_sub_=create_subscription<std_msgs::msg::String>("/rars01/commissioning/state",10,[this](std_msgs::msg::String::ConstSharedPtr m){
   std::lock_guard lock(mutex_);inputs_.ready.arm_feedback_ready=false;inputs_.ready.arm_target_ready=false;inputs_.arm_static_hold=false;inputs_.arm_home_ready=false;inputs_.arm_control_ready=false;inputs_.arm_emergency_validated=false;
   try {
    const auto y=YAML::Load(m->data);const auto now=SafetyClock::now();arm_error_.clear();
    auto q=Six(y["measured_q"]),dq=Six(y["measured_dq"]);
    auto ids=y["motor_ids"].as<std::vector<int>>();auto valid=y["valid"].as<std::vector<bool>>();
    if(ids.size()!=6||valid.size()!=6)throw std::runtime_error("arm identity length");
    for(int i=0;i<6;++i)if(ids[i]!=i+1||!valid[i])throw std::runtime_error("arm identity invalid");
    const auto received=TimeNs(y["frame_received_ns"].as<int64_t>());
    const double age=y["common_feedback_age_s"].as<double>();
    if(!std::isfinite(age)||age<0||Age(now,received)<0)throw std::runtime_error("invalid arm frame age");
    const auto observed=TimeNs(y["observed_ns"].as<int64_t>());
    if(Age(now,observed)<0)throw std::runtime_error("future arm observation");
    inputs_.arm_stamp=observed-std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(age));
    inputs_.ready.arm_feedback_ready=y["feedback_ready"].as<bool>();
    arm_q_=q;arm_dq_=dq;
    inputs_.arm_emergency_validated=y["arm_emergency_validated"]&&y["arm_emergency_validated"].as<bool>();
    inputs_.arm_static_hold=y["static_hold"].as<bool>();
    // New owner publishes all seven motors. Legacy six-only status cannot pass
    // the deployment HOME gate; gripper remains excluded from actor tensors.
    if(y["arm_home_ready"]&&y["arm_home_ready"].as<bool>()) {
     const auto ids7=y["motor_id"].as<std::vector<int>>();
     const auto states=y["motor_status"].as<std::vector<int>>();
     const auto valid7=y["valid7"].as<std::vector<bool>>();
     const auto q7=y["q"].as<std::vector<float>>(),dq7=y["dq"].as<std::vector<float>>();
     const auto target7=y["home_target"].as<std::vector<float>>();
     if(ids7.size()!=7||states.size()!=7||valid7.size()!=7||q7.size()!=7||dq7.size()!=7||target7.size()!=7)
      throw std::runtime_error("HOME requires seven motor diagnostics");
     for(size_t i=0;i<7;++i)if(ids7[i]!=int(i+1)||!valid7[i]||states[i]!=1||!std::isfinite(q7[i])||std::abs(q7[i])>home_tolerance_||!std::isfinite(dq7[i])||target7[i]!=0)
      throw std::runtime_error("HOME motor diagnostics invalid");
     inputs_.arm_home_ready=y["connected"].as<bool>()&&y["enabled_local"].as<bool>()&&
      !y["watchdog_tripped"].as<bool>()&&!y["stm32_watchdog_tripped"].as<bool>();
    }
    const bool owner_healthy=y["connected"].as<bool>()&&y["enabled_local"].as<bool>()&&
     !y["watchdog_tripped"].as<bool>()&&!y["stm32_watchdog_tripped"].as<bool>()&&
     y["owner_state"].as<std::string>()!="FAULT_LATCHED";
    const auto motor_states=y["motor_status"].as<std::vector<int>>();
    if(motor_states.size()!=7)throw std::runtime_error("seven motor control health values required");
    const bool motors_healthy=std::all_of(motor_states.begin(),motor_states.end(),[](int v){return v==1;});
    if(y["target_valid"].as<bool>()) {
     arm_target_=Six(y["accepted_target"]);
     if(inputs_.arm_home_ready)for(float v:arm_target_)if(v!=0)throw std::runtime_error("HOME accepted target must be zero");
     inputs_.target_stamp=TimeNs(y["accepted_ns"].as<int64_t>());
     inputs_.ready.arm_target_ready=Age(now,inputs_.target_stamp)>=0;
     inputs_.arm_control_ready=owner_healthy&&motors_healthy&&inputs_.ready.arm_feedback_ready&&inputs_.ready.arm_target_ready;
    }
   }catch(const std::exception& e){inputs_.ready.arm_feedback_ready=false;inputs_.ready.arm_target_ready=false;inputs_.arm_static_hold=false;inputs_.arm_home_ready=false;inputs_.arm_control_ready=false;inputs_.arm_emergency_validated=false;arm_error_=e.what();}
  });
  enable_=create_service<std_srvs::srv::SetBool>("/go2/commissioning/enable_output",[this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res){
   std::lock_guard lock(mutex_);const auto now=SafetyClock::now();Refresh(now);
   supervisor_->CancelRemoteSequence();
   const auto result=SetOutput(req->data,now);res->success=result.success;res->message=result.message;
  });
  Trigger("request_hold",[this](auto now){return supervisor_->Dispatch(SystemEvent::REQUEST_HOLD,now);});
  Trigger("request_stand",[this](auto now){return supervisor_->Dispatch(SystemEvent::REQUEST_STAND,now);});
  Trigger("request_rl",[this](auto now){return supervisor_->Dispatch(SystemEvent::REQUEST_RL,now);});
  Trigger("request_lie_down",[this](auto now){return supervisor_->Dispatch(SystemEvent::REQUEST_LIE_DOWN,now);});
  Trigger("controlled_abort",[this](auto now){return supervisor_->Dispatch(SystemEvent::REQUEST_X,now);});
  Trigger("emergency_damp",[this](auto now){return supervisor_->Dispatch(SystemEvent::REQUEST_B,now);});
  Trigger("request_return_to_stock",[this](auto now){return supervisor_->Dispatch(SystemEvent::RETURN_TO_STOCK,now);});
  manual_=create_service<std_srvs::srv::Trigger>("/go2/commissioning/manual_step",[](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
   res->success=false;res->message="use selected remote_test/autonomy command source";
  });
  nav_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel",rclcpp::QoS(1),[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){
   std::lock_guard lock(mutex_);
   if(control_mode_!="autonomy"||!motion_commands_enabled_||supervisor_->system_state()==SystemState::CONTROLLED_STOP||supervisor_->fault_latched())return;
   const std::array<double,3> requested{m->twist.linear.x,m->twist.linear.y,m->twist.angular.z};
   navigation_valid_=false;navigation_command_={};
   if(!std::all_of(requested.begin(),requested.end(),[](double v){return std::isfinite(v);})||
      m->header.stamp.sec<0||m->header.stamp.nanosec>=1000000000u||
      (m->header.stamp.sec==0&&m->header.stamp.nanosec==0))return;
   try {
    const double message_age=(this->now()-rclcpp::Time(m->header.stamp)).seconds();
    if(message_age<0||message_age>supervisor_->profile().command_timeout_s)return;
   }catch(const std::exception&){return;}
   navigation_valid_=true;
   const auto& p=supervisor_->profile();
   navigation_command_={std::clamp(requested[0],-p.vx_bound,p.vx_bound),std::clamp(requested[1],-p.vy_bound,p.vy_bound),std::clamp(requested[2],-p.wz_bound,p.wz_bound)};
   navigation_stamp_=SafetyClock::now();
  });
  io_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const int io_ms=r["lowcmd_period_ms"].as<int>();if(io_ms!=2)throw std::runtime_error("Current transport profile expects SDK example 2ms period");
  io_timer_=create_wall_timer(std::chrono::milliseconds(io_ms),[this]{
   std::lock_guard lock(mutex_);const auto now=SafetyClock::now();HandleRemote(now);
   if(supervisor_->state()==R3State::RL_ACTIVE&&Age(now,last_deadman_)>supervisor_->profile().command_timeout_s)
    supervisor_->ManualCommand({},.1,now);
   auto packet=supervisor_->Tick(now);
   if(output_&&packet&&!supervisor_->AllowsPacket(*packet,now))supervisor_->Fault("crc_or_packet_validation",now);
   else if(output_&&packet) {
    try{output_->publish(*packet);++sent_;supervisor_->NotifyPacketPublished(*packet,SafetyClock::now());}catch(...){supervisor_->Fault("transport_exception",now);StopPublisher();}
   }
   if(output_&&!supervisor_->output_enabled())StopPublisher();
   TraceState(now);
  },io_group_);
  policy_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  policy_timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{PolicyTick();},policy_group_);
  RCLCPP_INFO(get_logger(),"R3 read_only=%d; startup disarmed; remote sequence requires fresh chord and all prerequisites",read_only_);
  RCLCPP_INFO(get_logger(),"config=%s fixed_kp=%s fixed_kd=%s rl_kp=%s rl_kd=%s",config.c_str(),FloatList(profile.kp).c_str(),FloatList(profile.kd).c_str(),FloatList(profile.rl_kp).c_str(),FloatList(profile.rl_kd).c_str());
 }
 ~R3Node(){if(sdk_future_.valid())sdk_future_.wait();}
 private:
 // The IO callback handles the remote even while Torch is computing.
 // mutex_ serializes cancellation with PolicyResult and packet publication.
 void HandleRemote(SafetyTime now) {
  const auto event=remote_->Poll(now);Refresh(now);
  switch(event) {
   case R3RemoteEvent::TAKEOVER: {
    if(supervisor_->system_state()==SystemState::SYSTEM_HOLD) {
     // Verify ownership at the restart edge, never poll the graph at 500Hz.
     inputs_.own_output_healthy=output_&&lease_&&lease_->acquired()&&count_publishers("/lowcmd")==1;
     supervisor_->Observe(inputs_,now);
    }
    const auto reply=(supervisor_->operation_profile()==OperationProfile::ARM_TEST||(!read_only_&&automatic_sequence_))?supervisor_->Dispatch(SystemEvent::REQUEST_A,now):supervisor_->Takeover(now);
    sequence_message_=reply.message;break;
   }
   case R3RemoteEvent::CONTROLLED_ABORT:supervisor_->Dispatch(SystemEvent::REQUEST_X,now);break;
   case R3RemoteEvent::EMERGENCY:supervisor_->Dispatch(SystemEvent::REQUEST_B,now);break;
   default:break;
  }
  if(event!=R3RemoteEvent::NONE)last_event_=event;
 }
 bool NavigationFresh(SafetyTime now) const {return navigation_valid_&&Age(now,navigation_stamp_)>=0&&Age(now,navigation_stamp_)<=supervisor_->profile().command_timeout_s;}
 std::array<float,12> StandErrors() const {
  std::array<float,12> errors{};
  for(int i=0;i<12;++i)errors[i]=std::abs(inputs_.measured_q[i]-supervisor_->profile().stand[i]);
  return errors;
 }
 // Called under mutex_; log changes, never every 2ms output tick.
 void TraceState(SafetyTime now) {
  const auto state=supervisor_->phase();const auto& fault=supervisor_->last_fault();
  if(traced_state_&&*traced_state_==state&&traced_fault_==fault&&traced_latched_==supervisor_->fault_latched())return;
  traced_state_=state;traced_fault_=fault;traced_latched_=supervisor_->fault_latched();
  const auto errors=StandErrors();const auto worst=std::max_element(errors.begin(),errors.end());
  YAML::Emitter e;e<<YAML::Flow<<YAML::BeginMap
   <<YAML::Key<<"state"<<YAML::Value<<SystemStateName(supervisor_->system_state())
   <<YAML::Key<<"phase"<<YAML::Value<<SystemPhaseName(state)
   <<YAML::Key<<"legacy_state"<<YAML::Value<<R3StateName(supervisor_->state())
   <<YAML::Key<<"fault"<<YAML::Value<<fault
   <<YAML::Key<<"fault_latched"<<YAML::Value<<traced_latched_
   <<YAML::Key<<"sent_packets"<<YAML::Value<<sent_
   <<YAML::Key<<"stand_error_max_rad"<<YAML::Value<<*worst
   <<YAML::Key<<"stand_error_motor_index"<<YAML::Value<<std::distance(errors.begin(),worst)
   <<YAML::Key<<"policy_ms"<<YAML::Value<<last_policy_ms_
   <<YAML::Key<<"policy_result_age_s"<<YAML::Value<<supervisor_->PolicyAgeSeconds(now)
   <<YAML::Key<<"policy_inference_age_s"<<YAML::Value<<supervisor_->PolicyInferenceAgeSeconds(now)
   <<YAML::Key<<"policy_observation_age_s"<<YAML::Value<<supervisor_->PolicyObservationAgeSeconds(now)
   <<YAML::Key<<"lowstate_age_s"<<YAML::Value<<Age(now,inputs_.lowstate_stamp)
   <<YAML::Key<<"arm_feedback_age_s"<<YAML::Value<<Age(now,inputs_.arm_stamp)
   <<YAML::Key<<"arm_target_age_s"<<YAML::Value<<Age(now,inputs_.target_stamp)
   <<YAML::Key<<"sport_age_s"<<YAML::Value<<Age(now,inputs_.sport_stamp)<<YAML::EndMap;
  if(traced_latched_)RCLCPP_ERROR(get_logger(),"R3 transition %s",e.c_str());
  else RCLCPP_INFO(get_logger(),"R3 transition %s",e.c_str());
 }
 R3Reply SetOutput(bool enable,SafetyTime now) {
  if(enable&&sdk_release_pending_)return {false,"SDK release still pending"};
  if(enable&&read_only_)return {false,"read_only=true: physical output not available"};
  if(enable&&count_publishers("/lowcmd")!=0)return {false,"existing LowCmd publisher"};
  if(enable&&!lease_){lease_=std::make_unique<OutputLease>(output_lock_);if(!lease_->acquired()){lease_.reset();return {false,"output lease unavailable"};}}
  const auto result=supervisor_->Dispatch(enable?SystemEvent::ENABLE_OUTPUT:SystemEvent::DISABLE_OUTPUT,now);
  if(enable&&result.success){output_=create_publisher<unitree_go::msg::LowCmd>("/lowcmd",rclcpp::QoS(1));}
  else if(!enable)StopPublisher();
  else if(!sdk_release_pending_)lease_.reset();
  return result;
 }
 void SdkTick(SafetyTime now) {
  if(sdk_future_.valid()) {
   if(sdk_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
   const auto result=sdk_future_.get();const bool released_request=sdk_release_pending_;sdk_release_pending_=false;
   inputs_.sport=ParseSportMode(result.state);inputs_.sport_stamp=sdk_started_;sdk_detail_=result.detail;
   if(released_request&&!result.ok)supervisor_->Fault("sdk_release_failed: "+result.detail,now);
   if(!supervisor_->remote_sequence_active()&&!output_)lease_.reset();
   next_sdk_query_=now+std::chrono::milliseconds(100);
  }
  if(!sdk_future_.valid()&&!network_interface_.empty()&&now>=next_sdk_query_)StartSdk(false,now);
 }
 void StartSdk(bool release,SafetyTime now) {
  if(sdk_future_.valid()||network_interface_.empty()||(release&&read_only_))return;
  sdk_release_pending_=release;sdk_started_=now;
  sdk_future_=std::async(std::launch::async,[helper=sdk_helper_,nic=network_interface_,release]{return RunModeProcess(helper,nic,release);});
 }
 void SequenceTick(SafetyTime now) {
  if(!supervisor_->remote_sequence_active()||output_)lowcmd_wait_->Reset();
  if(read_only_||!automatic_sequence_||sdk_release_pending_)return;
  // A pending read-only query must finish before the single release request.
  if(sdk_future_.valid()&&!output_)return;
  const auto action=supervisor_->RemoteSequenceNext(now);R3Reply result{true,""};
  switch(action) {
   case R3SequenceAction::RELEASE_SPORT:
    if(network_interface_.empty()){result={false,"SDK interface missing"};break;}
    if(!lease_)lease_=std::make_unique<OutputLease>(output_lock_);
    if(!lease_->acquired()){lease_.reset();result={false,"output lease busy before release"};break;}
    StartSdk(true,now);break;
   case R3SequenceAction::ENABLE_OUTPUT: {
    const auto& profile=supervisor_->profile();
    const double sport_age=Age(now,inputs_.sport_stamp);
    const bool released=inputs_.sport==SportMode::RELEASED&&sport_age>=0&&sport_age<=profile.sport_timeout_s;
    const auto waiting=lowcmd_wait_->Update(now,released,count_publishers("/lowcmd"));
    if(waiting==LowCmdWaitResult::TIMEOUT){result={false,"LowCmd discovery timeout after Sport release"};break;}
    if(waiting==LowCmdWaitResult::WAITING){sequence_message_="waiting for fresh Sport release and no LowCmd publishers";break;}
    result=SetOutput(true,now);
    if(result.success){lowcmd_wait_->Reset();sequence_message_="LowCmd discovery clear; output enabled";}
    break;
   }
   case R3SequenceAction::STAND:result=supervisor_->Dispatch(SystemEvent::REQUEST_STAND,now);break;
   case R3SequenceAction::RL:result=supervisor_->Dispatch(SystemEvent::REQUEST_RL,now);break;
   default:break;
  }
  if(!result.success){lowcmd_wait_->Reset();sequence_message_=result.message;supervisor_->Fault("remote_sequence_failed: "+result.message,now);}
  if(!supervisor_->remote_sequence_active())lowcmd_wait_->Reset();
 }
 void Trigger(const std::string& name,std::function<R3Reply(SafetyTime)> request) {
  services_.push_back(create_service<std_srvs::srv::Trigger>("/go2/commissioning/"+name,[this,request](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
   std::lock_guard lock(mutex_);auto now=SafetyClock::now();Refresh(now);auto r=request(now);res->success=r.success;res->message=r.message;
  }));
 }
 void PortTick() {
  std::lock_guard lock(mutex_);const auto requests=supervisor_->ConsumePortRequests();
  const auto generation=supervisor_->orchestration_generation();
  if(requests.cancel_navigation){navigation_valid_=false;navigation_command_={};nav_cancel_pending_=true;nav_cancel_status_="pending";}
  if(requests.cancel_manipulation){manipulation_cancel_pending_=true;manipulation_cancel_status_="pending";}
  if(requests.arm_return_home){home_pending_=true;home_port_status_="pending";}
  if(requests.arm_emergency){home_pending_=false;emergency_pending_=true;emergency_port_status_="pending";}
  if(supervisor_->system_state()!=SystemState::CONTROLLED_STOP)home_pending_=false;
  if(home_pending_){
   if(!arm_home_client_->service_is_ready())home_port_status_="unavailable";
   else {home_pending_=false;home_port_status_="request_sent";
    arm_home_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
     std::lock_guard lock(mutex_);if(generation!=supervisor_->orchestration_generation())return;
     try{const auto result=future.get();home_port_status_=result->success?"accepted_waiting_measured_home":"rejected: "+result->message;supervisor_->ArmHomeRequestAccepted(result->success,SafetyClock::now());}
     catch(const std::exception& e){home_port_status_=e.what();supervisor_->ArmHomeRequestAccepted(false,SafetyClock::now());}
    });
   }
  }
  if(emergency_pending_){
   if(!arm_emergency_client_->service_is_ready())emergency_port_status_="unavailable";
   else {emergency_pending_=false;emergency_port_status_="request_sent";
    arm_emergency_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
     std::lock_guard lock(mutex_);if(generation!=supervisor_->orchestration_generation())return;
     try{const auto result=future.get();emergency_port_status_=(result->success?"accepted_not_physically_confirmed: ":"rejected: ")+result->message;supervisor_->ArmEmergencyResult(result->success,emergency_port_status_);}
     catch(const std::exception& e){emergency_port_status_=e.what();supervisor_->ArmEmergencyResult(false,emergency_port_status_);}
    });
   }
  }
  CancelPort(nav_cancel_pending_,nav_cancel_status_,nav_cancel_client_,generation,true);
  CancelPort(manipulation_cancel_pending_,manipulation_cancel_status_,manipulation_cancel_client_,generation,false);
 }
 void CancelPort(bool& pending,std::string& status,rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,uint64_t generation,bool navigation) {
  if(!pending)return;
  if(!client->service_is_ready()){status="unavailable; velocity/mission gate closed";return;}
  pending=false;status="request_sent";
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation,navigation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
   std::lock_guard lock(mutex_);if(generation!=supervisor_->orchestration_generation())return;
   auto& status=navigation?nav_cancel_status_:manipulation_cancel_status_;
   try{const auto result=future.get();status=(result->success?"accepted: ":"rejected: ")+result->message;}catch(const std::exception& e){status=e.what();}
  });
 }
 void Refresh(SafetyTime now) {
  inputs_.ready.lowstate_fresh=lowstate_.Fresh(now);inputs_.ready.remote_fresh=remote_->status().remote_valid;
  const bool mission_fresh=!mission_session_.empty()&&Age(now,mission_readiness_stamp_)>=0&&Age(now,mission_readiness_stamp_)<=.5;
  inputs_.navigation_ready=mission_fresh&&mission_navigation_ready_&&nav_cancel_available_;
  inputs_.perception_ready=mission_fresh&&mission_perception_ready_&&manipulation_cancel_available_;
  inputs_.own_output_healthy=output_&&lease_&&lease_->acquired()&&cached_publisher_count_==1&&Age(now,graph_stamp_)>=0&&Age(now,graph_stamp_)<=.5;
  supervisor_->Observe(inputs_,now);supervisor_->ConfirmStockObserved(now);
 }
 void StopPublisher(){supervisor_->Dispatch(SystemEvent::DISABLE_OUTPUT,SafetyClock::now());output_.reset();if(!sdk_release_pending_)lease_.reset();if(!output_&&!lease_)supervisor_->Dispatch(SystemEvent::OUTPUT_STOPPED,SafetyClock::now());}
 void PolicyTick() {
  std::array<float,6> armq,armdq,target;LowStateSnapshot low;std::array<double,3> cmd{};bool infer=false,reset=false;
  std::optional<PolicyTicket> ticket;
  {
   std::lock_guard lock(mutex_);auto now=SafetyClock::now();
   SdkTick(now);
   // Baseline graph diagnostics already ran at policy/status rate (50Hz).
   cached_publisher_count_=count_publishers("/lowcmd");graph_stamp_=now;
   const bool mission_fresh=!mission_session_.empty()&&Age(now,mission_readiness_stamp_)>=0&&Age(now,mission_readiness_stamp_)<=.5;
   nav_cancel_available_=mission_fresh&&nav_cancel_client_->service_is_ready();
   manipulation_cancel_available_=mission_fresh&&manipulation_cancel_client_->service_is_ready();
   Refresh(now);SequenceTick(now);
   remote_test_requested_=RemoteStickCommand(remote_->status(),supervisor_->profile());
   if(!read_only_&&supervisor_->system_state()==SystemState::ACTIVE&&supervisor_->NeedsPolicy()) {
    std::array<double,3> selected{};
    if(motion_commands_enabled_) {
     if(control_mode_=="remote_test")selected=remote_test_requested_;
     else if(NavigationFresh(now))selected=navigation_command_;
    }
    const auto result=control_mode_=="autonomy"?supervisor_->NavigationCommand(selected,now):supervisor_->RemoteTestCommand(selected,now);
    if(result.success)last_deadman_=now;
   }
   reset=supervisor_->ConsumePolicyReset();ticket=supervisor_->BeginPolicy(now);infer=ticket.has_value();low=lowstate_.snapshot();
   armq=arm_q_;armdq=arm_dq_;target=arm_target_;cmd=supervisor_->command();
   // Serial/SDK remain exclusively in the RARS owner; leg node uses async ports.
   supervisor_->ConsumeArmHoldRequest();
  }
  if(infer) {
   try{
    auto& a=core_.agent();core_.SetMeasuredLegs(low.motor_q,low.motor_dq);
    a.obs.base_quat=torch::tensor(std::vector<float>(low.quaternion_xyzw.begin(),low.quaternion_xyzw.end()));
    a.obs.ang_vel=torch::tensor(std::vector<float>(low.gyro.begin(),low.gyro.end()));
    a.obs.arm_pos=torch::tensor(std::vector<float>(armq.begin(),armq.end()));a.obs.arm_vel=torch::tensor(std::vector<float>(armdq.begin(),armdq.end()));a.obs.arm_target=torch::tensor(std::vector<float>(target.begin(),target.end()));
    a.obs.command=torch::tensor({float(cmd[0]),float(cmd[1]),float(cmd[2])});
    if(reset){a.obs.command=torch::zeros({3});a.ResetPolicyState();}
    const auto began=SafetyClock::now();auto action=a.Act();std::array<float,12> q;for(int i=0;i<12;++i)q[i]=action[io_motor_to_policy[i]].item<float>();
    const auto ended=SafetyClock::now();const double elapsed=std::chrono::duration<double,std::milli>(ended-began).count();
    std::lock_guard lock(mutex_);const auto completed=SafetyClock::now();
    const bool accepted=supervisor_->PolicyResult(*ticket,q,elapsed,completed);last_policy_ms_=elapsed;
    if(!accepted)RCLCPP_WARN(get_logger(),"Policy result rejected: compute_ms=%g job_age_s=%g observation_age_s=%g captured_arm_age_s=%g captured_target_age_s=%g state=%s fault=%s",elapsed,Age(completed,ticket->started),Age(completed,ticket->observation_stamp),Age(completed,ticket->arm_stamp),Age(completed,ticket->target_stamp),R3StateName(supervisor_->state()),supervisor_->last_fault().c_str());
   }catch(const std::exception& e){std::lock_guard lock(mutex_);supervisor_->PolicyFailed(*ticket,std::string("policy_exception: ")+e.what(),SafetyClock::now());}
  }
  std::lock_guard lock(mutex_);const auto now=SafetyClock::now();
  TraceState(now);
  const auto stand_errors=StandErrors();const auto worst_stand=std::max_element(stand_errors.begin(),stand_errors.end());
  const auto blockers=supervisor_->Blockers(now);YAML::Emitter e;
  e<<YAML::Flow<<YAML::BeginMap<<YAML::Key<<"state"<<YAML::Value<<SystemStateName(supervisor_->system_state())
   <<YAML::Key<<"phase"<<YAML::Value<<SystemPhaseName(supervisor_->phase())
   <<YAML::Key<<"legacy_state"<<YAML::Value<<R3StateName(supervisor_->state())
   <<YAML::Key<<"read_only"<<YAML::Value<<read_only_<<YAML::Key<<"output_enabled"<<YAML::Value<<supervisor_->output_enabled()
   <<YAML::Key<<"custom_leg_output"<<YAML::Value<<YAML::DoubleQuoted<<(output_&&supervisor_->output_enabled()?"ON":"OFF")<<YAML::Auto
   <<YAML::Key<<"lowcmd_lease_present"<<YAML::Value<<bool(lease_)
   <<YAML::Key<<"output_stop_confirmed"<<YAML::Value<<supervisor_->output_stopped()
   <<YAML::Key<<"passive_command_sent"<<YAML::Value<<(supervisor_->passive_packets_sent()>0)
   <<YAML::Key<<"passive_packets_sent"<<YAML::Value<<supervisor_->passive_packets_sent()
   <<YAML::Key<<"passive_packet_limit"<<YAML::Value<<R3Supervisor::passive_packet_limit
   <<YAML::Key<<"passive_timeout_s"<<YAML::Value<<R3Supervisor::passive_timeout_s
   <<YAML::Key<<"passive_sequence_complete"<<YAML::Value<<supervisor_->passive_sequence_complete()
   <<YAML::Key<<"last_commanded_leg_mode"<<YAML::Value<<supervisor_->last_commanded_leg_mode()
   <<YAML::Key<<"motor_power_off_confirmed"<<YAML::Value<<false
   <<YAML::Key<<"lowcmd_publisher_present"<<YAML::Value<<bool(output_)<<YAML::Key<<"sent_packets"<<YAML::Value<<sent_
   <<YAML::Key<<"sport_state"<<YAML::Value<<SportModeName(inputs_.sport)<<YAML::Key<<"sport_age_s"<<YAML::Value<<Age(now,inputs_.sport_stamp)
   <<YAML::Key<<"lowstate_age_s"<<YAML::Value<<lowstate_.AgeMs(now)/1000<<YAML::Key<<"remote_age_s"<<YAML::Value<<remote_->status().remote_age_ms/1000
   <<YAML::Key<<"remote_mask"<<YAML::Value<<remote_->status().button_mask<<YAML::Key<<"remote_buttons"<<YAML::Value<<remote_->status().decoded_buttons
   <<YAML::Key<<"remote_event"<<YAML::Value<<int(last_event_)<<YAML::Key<<"arm_feedback_age_s"<<YAML::Value<<Age(now,inputs_.arm_stamp)
   <<YAML::Key<<"control_mode"<<YAML::Value<<control_mode_
   <<YAML::Key<<"motion_commands_enabled"<<YAML::Value<<motion_commands_enabled_
   <<YAML::Key<<"navigation_command_fresh"<<YAML::Value<<NavigationFresh(now)
   <<YAML::Key<<"navigation_requested_command"<<YAML::Value<<std::vector<double>(navigation_command_.begin(),navigation_command_.end())
   <<YAML::Key<<"stand_completion"<<YAML::Value<<"time"
   <<YAML::Key<<"remote_test_mode"<<YAML::Value<<remote_test_mode_
   <<YAML::Key<<"remote_test_requested_command"<<YAML::Value<<std::vector<double>(remote_test_requested_.begin(),remote_test_requested_.end())
   <<YAML::Key<<"motion_command"<<YAML::Value<<std::vector<double>(supervisor_->command().begin(),supervisor_->command().end())
   <<YAML::Key<<"arm_target_age_s"<<YAML::Value<<Age(now,inputs_.target_stamp)<<YAML::Key<<"arm_ready"<<YAML::Value<<(inputs_.arm_home_ready&&inputs_.ready.arm_feedback_ready&&inputs_.ready.arm_target_ready&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s)
   <<YAML::Key<<"arm_home_ready"<<YAML::Value<<(inputs_.arm_home_ready&&Age(now,inputs_.arm_stamp)>=0&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)>=0&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s)
   <<YAML::Key<<"arm_control_ready"<<YAML::Value<<(inputs_.arm_control_ready&&Age(now,inputs_.arm_stamp)>=0&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)>=0&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s)
   <<YAML::Key<<"operation_profile"<<YAML::Value<<OperationProfileName(supervisor_->operation_profile())
   <<YAML::Key<<"command_source"<<YAML::Value<<CommandSourceName(supervisor_->capabilities().command_source)
   <<YAML::Key<<"capabilities"<<YAML::Value<<YAML::BeginMap
     <<YAML::Key<<"leg_output"<<YAML::Value<<supervisor_->capabilities().physical_leg_output
     <<YAML::Key<<"sport_release"<<YAML::Value<<supervisor_->capabilities().allow_sport_release
     <<YAML::Key<<"rl"<<YAML::Value<<supervisor_->capabilities().allow_rl
     <<YAML::Key<<"nonzero_velocity"<<YAML::Value<<supervisor_->capabilities().allow_nonzero_velocity
     <<YAML::Key<<"arm_motion"<<YAML::Value<<supervisor_->capabilities().allow_arm_motion<<YAML::EndMap
   <<YAML::Key<<"own_output_healthy"<<YAML::Value<<inputs_.own_output_healthy
   <<YAML::Key<<"navigation_ready"<<YAML::Value<<inputs_.navigation_ready
   <<YAML::Key<<"perception_ready"<<YAML::Value<<inputs_.perception_ready
   <<YAML::Key<<"arm_emergency_validated"<<YAML::Value<<inputs_.arm_emergency_validated
   <<YAML::Key<<"stop_blocker"<<YAML::Value<<supervisor_->stop_blocker()
   <<YAML::Key<<"arm_home_port"<<YAML::Value<<home_port_status_
   <<YAML::Key<<"arm_emergency_port"<<YAML::Value<<emergency_port_status_
   <<YAML::Key<<"navigation_cancel_port"<<YAML::Value<<nav_cancel_status_
   <<YAML::Key<<"manipulation_cancel_port"<<YAML::Value<<manipulation_cancel_status_
   <<YAML::Key<<"orchestration_generation"<<YAML::Value<<supervisor_->orchestration_generation()
   <<YAML::Key<<"fixed_target_motor_q"<<YAML::Value<<std::vector<float>(supervisor_->fixed_target().begin(),supervisor_->fixed_target().end())
   <<YAML::Key<<"lie_down_target_approved"<<YAML::Value<<supervisor_->LieDownTargetApproved()
   <<YAML::Key<<"controlled_stop_lie_down_trial"<<YAML::Value<<supervisor_->profile().controlled_stop_lie_down_trial
   <<YAML::Key<<"lie_down_commissioning_active"<<YAML::Value<<(supervisor_->profile().controlled_stop_lie_down_trial&&!supervisor_->profile().lie_down_validated&&supervisor_->system_state()==SystemState::CONTROLLED_STOP&&!supervisor_->NeedsPolicy())
   <<YAML::Key<<"lie_down_dynamics_validated"<<YAML::Value<<supervisor_->profile().lie_down_validated
   <<YAML::Key<<"critical_control_fault"<<YAML::Value<<(supervisor_->fault_latched()&&supervisor_->last_fault()!="operator_emergency")
   <<YAML::Key<<"arm_error"<<YAML::Value<<arm_error_
   <<YAML::Key<<"measured_motor_q"<<YAML::Value<<std::vector<float>(inputs_.measured_q.begin(),inputs_.measured_q.end())
   <<YAML::Key<<"stand_target_motor_q"<<YAML::Value<<std::vector<float>(supervisor_->profile().stand.begin(),supervisor_->profile().stand.end())
   <<YAML::Key<<"stand_error_rad"<<YAML::Value<<std::vector<float>(stand_errors.begin(),stand_errors.end())
   <<YAML::Key<<"stand_error_max_rad"<<YAML::Value<<*worst_stand
   <<YAML::Key<<"stand_error_motor_index"<<YAML::Value<<std::distance(stand_errors.begin(),worst_stand)
   <<YAML::Key<<"remote_auto_sequence"<<YAML::Value<<automatic_sequence_
   <<YAML::Key<<"remote_sequence_active"<<YAML::Value<<supervisor_->remote_sequence_active()
   <<YAML::Key<<"remote_sequence_message"<<YAML::Value<<sequence_message_
   <<YAML::Key<<"remote_sequence_blockers"<<YAML::Value<<supervisor_->RemoteSequenceBlockers(now)
   <<YAML::Key<<"sdk_release_pending"<<YAML::Value<<sdk_release_pending_
   <<YAML::Key<<"lowcmd_discovery_waiting"<<YAML::Value<<lowcmd_wait_->active()
   <<YAML::Key<<"lowcmd_discovery_wait_age_s"<<YAML::Value<<lowcmd_wait_->AgeSeconds(now)
   <<YAML::Key<<"lowcmd_publisher_count"<<YAML::Value<<cached_publisher_count_
   <<YAML::Key<<"model_loaded"<<YAML::Value<<core_.loaded()<<YAML::Key<<"policy_ms"<<YAML::Value<<last_policy_ms_
   <<YAML::Key<<"policy_result_age_s"<<YAML::Value<<supervisor_->PolicyAgeSeconds(now)
   <<YAML::Key<<"policy_inference_age_s"<<YAML::Value<<supervisor_->PolicyInferenceAgeSeconds(now)
   <<YAML::Key<<"policy_observation_age_s"<<YAML::Value<<supervisor_->PolicyObservationAgeSeconds(now)
   <<YAML::Key<<"policy_inflight"<<YAML::Value<<supervisor_->policy_inflight()
   <<YAML::Key<<"rejected_policy_results"<<YAML::Value<<supervisor_->rejected_policy_results()
   <<YAML::Key<<"deadline_misses"<<YAML::Value<<supervisor_->deadline_misses()<<YAML::Key<<"fault_latched"<<YAML::Value<<supervisor_->fault_latched()
   <<YAML::Key<<"fault"<<YAML::Value<<supervisor_->last_fault()<<YAML::Key<<"blockers"<<YAML::Value<<blockers<<YAML::EndMap;
  std_msgs::msg::String status;status.data=e.c_str();status_->publish(status);
 }
 std::mutex mutex_;RealControllerCore core_;LowStateReader lowstate_;
 std::optional<SystemPhase> traced_state_;std::string traced_fault_;bool traced_latched_=false;
 std::unique_ptr<R3Supervisor> supervisor_;std::unique_ptr<R3RemoteCommands> remote_;R3Inputs inputs_;
 std::future<ModeProcessResult> sdk_future_;SafetyTime sdk_started_{},next_sdk_query_{};
 bool sdk_release_pending_=false,automatic_sequence_=true;std::string network_interface_,sdk_helper_,sdk_detail_,sequence_message_;
 std::unique_ptr<LowCmdDiscoveryWait> lowcmd_wait_;
 bool read_only_=true,remote_test_mode_=false,motion_commands_enabled_=false,navigation_valid_=false;std::string control_mode_;std::string output_lock_,arm_error_;
 std::array<double,3> remote_test_requested_{};
 std::unique_ptr<OutputLease> lease_;std::array<float,6> arm_q_{},arm_dq_{},arm_target_{};
 std::array<double,3> navigation_command_{};double last_policy_ms_=0,home_tolerance_=.15;size_t sent_=0;
 SafetyTime navigation_stamp_{},last_deadman_{};R3RemoteEvent last_event_=R3RemoteEvent::NONE;
 rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr output_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sport_sub_,arm_sub_,mission_readiness_sub_;
 size_t cached_publisher_count_=0;SafetyTime graph_stamp_{};
 bool nav_cancel_available_=false,manipulation_cancel_available_=false;
 bool mission_navigation_ready_=false,mission_perception_ready_=false;SafetyTime mission_readiness_stamp_{};std::string mission_session_;
 rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nav_sub_;
 rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_;
 std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
 rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_home_client_,arm_emergency_client_,nav_cancel_client_,manipulation_cancel_client_;
 bool home_pending_=false,emergency_pending_=false,nav_cancel_pending_=false,manipulation_cancel_pending_=false;
 std::string home_port_status_="idle",emergency_port_status_="idle",nav_cancel_status_="idle",manipulation_cancel_status_="idle";
 rclcpp::TimerBase::SharedPtr port_timer_;rclcpp::CallbackGroup::SharedPtr port_group_;
 rclcpp::TimerBase::SharedPtr io_timer_,policy_timer_;rclcpp::CallbackGroup::SharedPtr io_group_,policy_group_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<R3Node>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);executor.add_node(node);executor.spin();}
 catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("r3"),"Startup failed closed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
