#include "r3_commissioning.hpp"
#include "real_controller_core.hpp"
#include "policy_tensor_io.hpp"
#include "output_lease.hpp"
#include "sdk_mode_session.hpp"
#include "sport_query_schedule.hpp"
#include "async_mode_worker.hpp"
#include "motion_trace.hpp"
#include "control_latency.hpp"
#include "arm_status_snapshot.hpp"
#include "navigation_request_gate.hpp"
#include "control_diagnostics.hpp"
#include "control_executors.hpp"
#include "policy_rt_registration.hpp"
#include "policy_thread.hpp"
#include "policy_process.hpp"
#include "isolated_replay.hpp"
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
#include <limits>
#include <cstdlib>
#include <ctime>
#include <fstream>
using namespace sim2real;
namespace {
#ifdef R3_COMPUTE_PROBE_ONLY
constexpr bool compute_probe=true;
#else
constexpr bool compute_probe=false;
#endif
#ifdef R3_LIVE_SHADOW_ONLY
constexpr bool live_shadow=true;
#else
constexpr bool live_shadow=false;
#endif
void RequireProbeNetwork() {
 if constexpr(!live_shadow) { RequireIsolatedReplay(); return; }
 const auto domain=std::getenv("ROS_DOMAIN_ID"),dds=std::getenv("CYCLONEDDS_URI");
 if(!domain||std::string(domain)!="0"||!dds||std::string(dds).find("name=\"enP8p1s0\"")==std::string::npos)
  throw std::runtime_error("Live shadow requires real domain0/enP8p1s0");
}
int64_t ThreadCpuNs(){timespec t{};return clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t)==0?int64_t(t.tv_sec)*1000000000+t.tv_nsec:-1;}
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
  if(declare_parameter<bool>("cooperative_policy_registration",false,immutable))policy_rt_=std::make_unique<PolicyRtRegistration>();
  const auto config=declare_parameter<std::string>("config_path","",immutable);
  const auto model=declare_parameter<std::string>("model_path","",immutable);
  const auto y=YAML::LoadFile(config),d=y["real_deployment"],r=d["r3_commissioning"],trace_config=r["motion_diagnostics"];
  const bool trace_enabled=declare_parameter<bool>("motion_diagnostics_enabled",compute_probe||trace_config["enabled"].as<bool>(),immutable);
  const double trace_seconds=declare_parameter<double>("motion_diagnostics_duration_s",trace_config["duration_s"].as<double>(),immutable);
  const auto trace_prefix=declare_parameter<std::string>("motion_diagnostics_path",trace_config["path"].as<std::string>(),immutable);
  if(trace_enabled) {
   const auto path=trace_prefix+"-"+std::to_string(SafetyClock::now().time_since_epoch().count())+".csv";
   motion_trace_=std::make_unique<MotionTrace>(path,trace_seconds,trace_config["queue_capacity"].as<size_t>(),trace_config["writer_idle_ms"].as<int>(),trace_config["max_duration_s"].as<double>());
   RCLCPP_INFO(get_logger(),"Temporary motion diagnostics: %s; auto-stop after %.1fs from first sample",path.c_str(),trace_seconds);
  }
  services_.push_back(create_service<std_srvs::srv::Trigger>("/go2/diagnostics/stop_motion_log",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
   if(motion_trace_)motion_trace_->Stop();response->success=true;response->message="motion logging stopped; queued rows drain asynchronously";
  }));
  const auto profile_name=declare_parameter<std::string>("operation_profile",compute_probe?"read_only":"",immutable);
  std::optional<OperationProfile> explicit_profile;
  if(!profile_name.empty())explicit_profile=ParseOperationProfile(profile_name);
  read_only_=declare_parameter<bool>("read_only",explicit_profile?!ResolveOperationCapabilities(*explicit_profile).physical_leg_output:true,immutable);
  automatic_sequence_=declare_parameter<bool>("remote_auto_sequence",r["runtime"]["remote_auto_sequence"].as<bool>(),immutable);
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
  receiver_trace_path_=declare_parameter<std::string>("receiver_diagnostics_path","",immutable);
  io_trace_path_=declare_parameter<std::string>("io_diagnostics_path","",immutable);
  if(!receiver_trace_path_.empty()||!io_trace_path_.empty()) {
   RequireProbeNetwork();
   if(!read_only_||!network_interface_.empty())throw std::runtime_error("Input/IO diagnostics require read_only and no SDK interface");
   if(!receiver_trace_path_.empty())receiver_trace_.reserve(300001);
   if(!io_trace_path_.empty())io_trace_.reserve(300001);
  }
  worker_self_test_frames_=declare_parameter<int>("policy_worker_self_test_frames",0,immutable);
  if(worker_self_test_frames_<0||worker_self_test_frames_>1000)throw std::runtime_error("worker self test frames must be 0..1000");
  if(worker_self_test_frames_) {
   RequireIsolatedReplay();
   const auto domain=std::getenv("ROS_DOMAIN_ID"),dds=std::getenv("CYCLONEDDS_URI");
   if(operation!=OperationProfile::READ_ONLY||!read_only_||!network_interface_.empty()||
      !domain||std::string(domain)!="228"||!dds||std::string(dds).find("name=\"lo\"")==std::string::npos)
    throw std::runtime_error("Worker self test requires read_only, domain228, loopback and empty SDK interface");
  }
  if constexpr(compute_probe) {
   RequireProbeNetwork();
   const auto domain=std::getenv("ROS_DOMAIN_ID"),dds=std::getenv("CYCLONEDDS_URI");
   if((!live_shadow&&(!domain||std::string(domain)!="228"||!dds||std::string(dds).find("name=\"lo\"")==std::string::npos))||
      operation!=OperationProfile::READ_ONLY||!read_only_||!network_interface_.empty())
    throw std::runtime_error("Compute probe requires loopback domain228, read_only profile and empty SDK interface");
   probe_shared_executor_=declare_parameter<bool>("probe_shared_executor",false,immutable);
   if(probe_shared_executor_&&policy_rt_)throw std::runtime_error("RT requires dedicated policy executor");
   const double seconds=declare_parameter<double>("probe_duration_s",60.,immutable);
   if(!std::isfinite(seconds)||seconds<=0||seconds>600)throw std::runtime_error("Probe duration must be 0..600 seconds");
   probe_stop_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds)),[]{rclcpp::shutdown();});
  }
  sdk_helper_=(std::filesystem::read_symlink("/proc/self/exe").parent_path()/"go2_mode_switch").string();
  if(declare_parameter<bool>("enable_actuator_output",false,immutable))throw std::runtime_error("startup output must be false; use explicit service gate");
  sdk_session_=std::make_shared<ModeSessionProcess>(sdk_helper_,network_interface_,config);
  sdk_worker_=std::make_unique<AsyncModeWorker>([session=sdk_session_](bool release,int timeout){return session->Request(release,timeout);});
  core_.Load(config,model);
  // Warm TorchScript before any subscriptions, output publisher or takeover.
  // Discard all warm-up targets; the real RL entry resets history from feedback.
  {
   InitializePolicyThread();
   torch::InferenceMode inference;
   auto& a=core_.agent();a.obs.dof_pos=a.params.default_dof_pos.clone();a.ResetPolicyState();
   for(int i=0;i<20;++i){const auto q=a.Act();
    if(q.numel()!=12||!torch::isfinite(q).all().item<bool>())throw std::runtime_error("Policy warm-up returned invalid targets");
   }
   a.ResetPolicyState();
  }
  const auto latency=r["control_latency"];
  if(latency && latency["enabled"].as<bool>()) {
   const auto path=latency["path"].as<std::string>()+"-"+std::to_string(ControlMonotonicNs())+".csv";
   control_latency_=std::make_unique<ControlLatency>(path,latency["queue_capacity"].as<size_t>(),latency["duration_s"].as<double>(),latency["flush_ms"].as<int>(),latency["writer_idle_ms"].as<int>());
   RCLCPP_INFO(get_logger(),"Control latency trace: %s (CLOCK_MONOTONIC); control mutex=PTHREAD_PRIO_INHERIT",path.c_str());
  }
  home_tolerance_=d["rars01"]["auto_home"]["home_tolerance_rad"].as<double>();
  if(!std::isfinite(home_tolerance_)||home_tolerance_<=0)throw std::runtime_error("Invalid HOME tolerance");
  auto profile=LoadR3Profile(y);
  policy_period_ns_=int64_t(profile.policy_period_ms)*1000000;
  const auto execution=r["runtime"]["policy_execution"].as<std::string>("monotonic_thread");
  if(execution!="monotonic_thread"&&execution!="ros_executor")throw std::runtime_error("Invalid policy_execution");
  if(!read_only_&&execution!="monotonic_thread")throw std::runtime_error("Physical policy requires monotonic_thread");
  direct_policy_=execution=="monotonic_thread"&&!probe_shared_executor_;
  profile.controlled_stop_lie_down_trial=declare_parameter<bool>("controlled_stop_lie_down_trial",profile.controlled_stop_lie_down_trial,immutable);
  supervisor_=std::make_unique<R3Supervisor>(profile,operation);
  sdk_query_schedule_=std::make_unique<SportQuerySchedule>(profile.sport_startup_only,profile.sdk_query_period_ms);
  if(policy_rt_) {
   const auto guardian=r["guardian"];
   const auto seconds=guardian["native_registration_timeout_s"].as<double>();
   const auto poll=guardian["native_registration_poll_ms"].as<int>();
   if(!std::isfinite(seconds)||seconds<=0||seconds>8||poll<1||poll>100)throw std::runtime_error("Invalid native RT admission timing");
   policy_rt_->registration_timeout_ns=uint64_t(seconds*1.e9);policy_rt_->registration_poll_ms=poll;
  }
  navigation_gate_=std::make_unique<NavigationRequestGate>(std::array<double,3>{profile.vx_bound,profile.vy_bound,profile.wz_bound},profile.command_timeout_s);
  lowcmd_wait_=std::make_unique<LowCmdDiscoveryWait>(profile.lowcmd_discovery_timeout_s,profile.lowcmd_clear_duration_s);
  lowstate_=LowStateReader(profile.lowstate_timeout_s);
  remote_=std::make_unique<R3RemoteCommands>(r["remote"]["controlled_abort_chord"].as<std::vector<std::string>>(),r["remote"]["emergency_chord"].as<std::vector<std::string>>(),d["remote"]["takeover_hold_s"].as<double>(),profile.remote_timeout_s);
  output_lock_=r["output_lock"].as<std::string>();
  inputs_.ready.model_loaded=core_.loaded();inputs_.ready.config_valid=true;inputs_.ready.transport_ready=!read_only_;
  status_=create_publisher<std_msgs::msg::String>("/go2/locomotion_status",10);
  // Parsing arm/NAV status must not exclude delivery of the next leg sample
  // through the default mutually-exclusive callback group.
  state_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions state_options;state_options.callback_group=state_group_;
  low_sub_=create_subscription<unitree_go::msg::LowState>("/lowstate",rclcpp::SensorDataQoS().keep_last(1),[this](unitree_go::msg::LowState::ConstSharedPtr m){
   ControlScope callback(control_latency_.get(),5,__LINE__);
   const auto callback_started=SafetyClock::now();
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);const auto now=SafetyClock::now();
   const bool valid=lowstate_.Receive(*m,now);remote_->Receive(m->wireless_remote,now);
   inputs_.lowstate_stamp=now;inputs_.remote_stamp=now;inputs_.ready.motor_state_valid=valid;
   if(policy_rt_)policy_rt_->Input();
   if(!receiver_trace_path_.empty()) {
    if(receiver_trace_.size()<receiver_trace_.capacity())receiver_trace_.push_back({m->tick,callback_started.time_since_epoch().count(),now.time_since_epoch().count()});
    else ++receiver_trace_dropped_;
   }
   if(valid)inputs_.measured_q=lowstate_.snapshot().motor_q;
   else if(supervisor_->output_enabled())supervisor_->Fault("invalid_lowstate",now);
  });
  sport_sub_=create_subscription<std_msgs::msg::String>("/go2/commissioning/sport_observation",10,[this](std_msgs::msg::String::ConstSharedPtr m){
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);if(!network_interface_.empty())return;try{auto y=YAML::Load(m->data);inputs_.sport=ParseSportMode(y["state"].as<std::string>());inputs_.sport_stamp=TimeNs(y["observed_ns"].as<int64_t>());}
   catch(...){inputs_.sport=SportMode::ERROR;inputs_.sport_stamp=SafetyTime{};}
  });
  mission_readiness_sub_=create_subscription<std_msgs::msg::String>("/go2/mission/readiness",rclcpp::QoS(1),[this](std_msgs::msg::String::ConstSharedPtr message){
   YAML::Node decoded;std::string decode_error;
   try{if(message->data.size()>4096)throw std::runtime_error("oversize navigation envelope");decoded=YAML::Load(message->data);}
   catch(const std::exception& e){decode_error=e.what();}
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);mission_perception_ready_=false;
   try{if(!decode_error.empty())throw std::runtime_error(decode_error);
    const auto& y=decoded;const auto stamp=TimeNs(y["observed_ns"].as<int64_t>());
    const auto session=y["session_id"].as<std::string>();const bool ready=y["navigation_ready"].as<bool>();
    if(!navigation_gate_->Observe(session,ready,y["command_enabled"].as<bool>(),stamp,SafetyClock::now()))return;
    mission_readiness_stamp_=stamp;mission_session_=session;
    mission_perception_ready_=y["perception_ready"].as<bool>();
    // Atomic envelope binds readiness, session and velocity. /cmd_vel remains
    // a standard output for diagnostics; unbound publishers cannot drive R3.
    const auto c=y["command"];
    if(!c.IsSequence()||c.size()!=3)throw std::runtime_error("invalid navigation command shape");
    if(control_mode_=="autonomy"&&motion_commands_enabled_&&supervisor_->system_state()!=SystemState::CONTROLLED_STOP&&!supervisor_->fault_latched())
     navigation_gate_->Accept(session,{c[0].as<double>(),c[1].as<double>(),c[2].as<double>()},y["command_age_s"].as<double>(),stamp,SafetyClock::now());
   }catch(...){navigation_gate_->Invalidate();mission_readiness_stamp_={};}
  },state_options);
  arm_home_client_=create_client<std_srvs::srv::Trigger>("/rars01/control/return_home");
  arm_emergency_client_=create_client<std_srvs::srv::Trigger>("/rars01/control/emergency_disable");
  nav_cancel_client_=create_client<std_srvs::srv::Trigger>("/go2/mission/cancel_navigation");
  manipulation_cancel_client_=create_client<std_srvs::srv::Trigger>("/rars01/mission/cancel_manipulation");
  port_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  port_timer_=create_wall_timer(std::chrono::milliseconds(profile.port_period_ms),[this]{PortTick();},port_group_);
  arm_sub_=create_subscription<std_msgs::msg::String>("/rars01/commissioning/state",rclcpp::QoS(1),[this](std_msgs::msg::String::ConstSharedPtr m){
   ControlScope callback(control_latency_.get(),4,__LINE__);
   auto parsed=ParseArmStatus(m->data,home_tolerance_,SafetyClock::now());
   {
    ControlLock lock(mutex_,control_latency_.get(),__LINE__);
    ApplyArmStatus(parsed,inputs_,arm_q_,arm_dq_,arm_target_);
    arm_error_.swap(parsed.error);
   }
  },state_options);
  enable_=create_service<std_srvs::srv::SetBool>("/go2/commissioning/enable_output",[this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res){
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);const auto now=SafetyClock::now();Refresh(now);
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
  io_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  const int io_ms=r["lowcmd_period_ms"].as<int>();if(io_ms!=2)throw std::runtime_error("Current transport profile expects SDK example 2ms period");
  io_timer_=create_wall_timer(std::chrono::milliseconds(io_ms),[this](const rclcpp::TimerInfo& info){
   ControlScope callback(control_latency_.get(),3,__LINE__,info.expected_call_time.nanoseconds(),info.actual_call_time.nanoseconds());
   if(!io_trace_path_.empty()) {
    if(io_trace_.size()<io_trace_.capacity())io_trace_.push_back(SafetyClock::now().time_since_epoch().count());
    else ++io_trace_dropped_;
   }
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);const auto now=SafetyClock::now();HandleRemote(now);
   if(compute_probe||(motion_trace_&&motion_trace_->enabled())) {
    if(last_io_tick_!=SafetyTime{})io_interval_max_ms_=std::max(io_interval_max_ms_,Age(now,last_io_tick_)*1000.);
    last_io_tick_=now;
   }
   // NAV dropout/cancel changes velocity only, never switches leg mode.
   if(!read_only_&&control_mode_=="autonomy"&&supervisor_->system_state()==SystemState::ACTIVE&&supervisor_->NeedsPolicy()) {
    supervisor_->NavigationCommand(inputs_.navigation_ready?navigation_gate_->Get(now):std::array<double,3>{},now);
    last_deadman_=now;
   }
   if(supervisor_->state()==R3State::RL_ACTIVE&&Age(now,last_deadman_)>supervisor_->profile().command_timeout_s)
    supervisor_->ManualCommand({},.1,now);
   auto packet=supervisor_->Tick(now);
   if(output_&&packet&&!supervisor_->AllowsPacket(*packet,now))supervisor_->Fault("crc_or_packet_validation",now);
   else if(output_&&packet) {
    try{{ControlScope publish(control_latency_.get(),6,__LINE__);output_->publish(*packet);}
     ++sent_;const auto published=SafetyClock::now();supervisor_->NotifyPacketPublished(*packet,published);
     if(motion_trace_&&motion_trace_->enabled()) {
      try{TraceMotionIo(*packet,published);}catch(...){motion_trace_->Stop();}
     }
    }catch(...){supervisor_->Fault("transport_exception",now);StopPublisher();}
   }
   if(output_&&!supervisor_->output_enabled())StopPublisher();
   TraceState(now);
   if(policy_rt_)policy_rt_->Io();
  },io_group_);
  policy_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  if(!direct_policy_)policy_timer_=create_wall_timer(std::chrono::milliseconds(profile.policy_period_ms),[this](const rclcpp::TimerInfo& info){PolicyCycle(info.expected_call_time.nanoseconds(),info.actual_call_time.nanoseconds());},policy_group_);
  status_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  status_timer_=create_wall_timer(std::chrono::milliseconds(profile.status_period_ms),[this]{StatusTick();},status_group_);
  RCLCPP_INFO(get_logger(),"R3 read_only=%d; startup disarmed; remote sequence requires fresh chord and all prerequisites",read_only_);
  RCLCPP_INFO(get_logger(),"config=%s fixed_kp=%s fixed_kd=%s rl_kp=%s rl_kd=%s",config.c_str(),FloatList(profile.kp).c_str(),FloatList(profile.kd).c_str(),FloatList(profile.rl_kp).c_str(),FloatList(profile.rl_kd).c_str());
 }
 ~R3Node() override {
  if(!receiver_trace_path_.empty()) {
   std::ofstream f(receiver_trace_path_);f<<"sequence,callback_begin_ns,receive_ns\n";
   for(const auto& r:receiver_trace_)f<<r.sequence<<','<<r.callback<<','<<r.receive<<'\n';
   std::ofstream(receiver_trace_path_+".meta.json")<<"{\"dropped\":"<<receiver_trace_dropped_<<"}\n";
   if(!f)RCLCPP_ERROR(get_logger(),"Failed to export receiver diagnostics");
  }
  if(!io_trace_path_.empty()) {
   std::ofstream f(io_trace_path_);f<<"callback_begin_ns\n";for(auto t:io_trace_)f<<t<<'\n';
   std::ofstream(io_trace_path_+".meta.json")<<"{\"dropped\":"<<io_trace_dropped_<<"}\n";
   if(!f)RCLCPP_ERROR(get_logger(),"Failed to export IO diagnostics");
  }
 }
 auto IoGroup()const{return io_group_;}
 auto PolicyGroup()const{return policy_group_;}
 bool SharedProbeExecutor()const{return probe_shared_executor_;}
 bool DirectPolicy()const{return direct_policy_;}
 int64_t PolicyPeriodNs()const{return policy_period_ns_;}
 void PolicyCycle(int64_t expected,int64_t actual) {
  ControlScope callback(control_latency_.get(),2,__LINE__,expected,actual);
  if(policy_rt_)policy_rt_->Begin();
  PolicyTick();
  if(policy_rt_)policy_rt_->Complete();
 }
 private:
 // The IO callback handles the remote even while Torch is computing.
 // mutex_ serializes cancellation with PolicyResult and packet publication.
 void HandleRemote(SafetyTime now) {
  if constexpr(live_shadow) { Refresh(now); return; }
  const auto event=remote_->Poll(now);Refresh(now);
  if(event==R3RemoteEvent::TAKEOVER&&policy_rt_&&!policy_rt_->Ready())return;
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
 bool NavigationFresh(SafetyTime now) const {return inputs_.navigation_ready&&navigation_gate_->FreshCommand(now);}
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
   <<YAML::Key<<"policy_zero_handoff_pending"<<YAML::Value<<supervisor_->zero_policy_handoff_pending()
   <<YAML::Key<<"policy_inference_age_s"<<YAML::Value<<supervisor_->PolicyInferenceAgeSeconds(now)
   <<YAML::Key<<"policy_observation_age_s"<<YAML::Value<<supervisor_->PolicyObservationAgeSeconds(now)
   <<YAML::Key<<"policy_observation_timeout_s"<<YAML::Value<<supervisor_->profile().policy_observation_timeout_s
   <<YAML::Key<<"lowstate_age_s"<<YAML::Value<<Age(now,inputs_.lowstate_stamp)
   <<YAML::Key<<"arm_feedback_age_s"<<YAML::Value<<Age(now,inputs_.arm_stamp)
   <<YAML::Key<<"arm_target_age_s"<<YAML::Value<<Age(now,inputs_.target_stamp)
   <<YAML::Key<<"sport_age_s"<<YAML::Value<<Age(now,inputs_.sport_stamp)
   <<YAML::Key<<"sport_state"<<YAML::Value<<SportModeName(inputs_.sport)
   <<YAML::Key<<"sport_query_ok"<<YAML::Value<<sdk_last_ok_
   <<YAML::Key<<"sport_rpc_ms"<<YAML::Value<<sdk_rpc_ms_
   <<YAML::Key<<"sport_query_failures"<<YAML::Value<<sdk_failures_
   <<YAML::Key<<"sport_query_detail"<<YAML::Value<<sdk_detail_
   <<YAML::Key<<"blockers"<<YAML::Value<<supervisor_->Blockers(now)<<YAML::EndMap;
  if(traced_latched_)RCLCPP_ERROR(get_logger(),"R3 transition %s",e.c_str());
  else RCLCPP_INFO(get_logger(),"R3 transition %s",e.c_str());
 }
 R3Reply SetOutput(bool enable,SafetyTime now) {
  if(enable&&policy_rt_&&!policy_rt_->Ready())return {false,"policy RT registration pending"};
  if(compute_probe&&enable)return {false,"compute probe has no actuator output"};
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
  if(auto completed=sdk_worker_->Take()) {
   const auto result=std::move(*completed);const bool released_request=sdk_release_pending_;sdk_release_pending_=false;
   sdk_detail_=result.detail;
   sdk_last_ok_=result.ok;
   sdk_rpc_ms_=result.completed_ns>0?(result.completed_ns-result.started_ns)/1.e6:-1;
   // A timeout is not evidence that Sport became active. Keep the last genuine
   // observation, without renewing its timestamp. Startup-only admission is latched.
   if(result.ok) {
    inputs_.sport=ParseSportMode(result.state);inputs_.sport_stamp=TimeNs(result.started_ns);
    sdk_failures_=0;
   } else ++sdk_failures_;
   if(released_request&&!result.ok)supervisor_->Fault("sdk_release_failed: "+result.detail,now);
   if(!supervisor_->remote_sequence_active()&&!output_)lease_.reset();
   sdk_query_schedule_->Completed(now);
  }
  if(!network_interface_.empty()&&sdk_query_schedule_->Due(now,sdk_worker_->busy()))StartSdk(false,now);
 }
 void StartSdk(bool release,SafetyTime now) {
  if(sdk_worker_->busy()||network_interface_.empty()||(release&&read_only_))return;
  if(release&&supervisor_->output_enabled())return; // release belongs only to initial handover
  if(sdk_worker_->Submit(release,release?supervisor_->profile().sdk_release_timeout_ms:supervisor_->profile().sdk_query_timeout_ms)){
   if(!release){sdk_query_schedule_->Submitted();++sdk_status_queries_;}else ++sdk_release_requests_;
   sdk_release_pending_=release;sdk_started_=now;
  }
 }
 void SequenceTick(SafetyTime now) {
  if(!supervisor_->remote_sequence_active()||output_)lowcmd_wait_->Reset();
  if(read_only_||!automatic_sequence_||sdk_release_pending_||(policy_rt_&&!policy_rt_->Ready()))return;
  // A pending read-only query must finish before the single release request.
  if(sdk_worker_->busy()&&!output_)return;
  const auto action=supervisor_->RemoteSequenceNext(now);R3Reply result{true,""};
  switch(action) {
   case R3SequenceAction::RELEASE_SPORT:
    if(network_interface_.empty()){result={false,"SDK interface missing"};break;}
    if(!lease_)lease_=std::make_unique<OutputLease>(output_lock_);
    if(!lease_->acquired()){lease_.reset();result={false,"output lease busy before release"};break;}
    StartSdk(true,now);break;
   case R3SequenceAction::ENABLE_OUTPUT: {
    const bool released=supervisor_->SportReleased(now);
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
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);auto now=SafetyClock::now();Refresh(now);auto r=request(now);res->success=r.success;res->message=r.message;
  }));
 }
 void PortTick() {
  if constexpr(compute_probe)return; // No arm/NAV service requests from a shadow executor.
  ControlLock lock(mutex_,control_latency_.get(),__LINE__);const auto requests=supervisor_->ConsumePortRequests();
  const auto generation=supervisor_->orchestration_generation();
  if(requests.cancel_navigation){navigation_gate_->Invalidate();nav_cancel_pending_=true;nav_cancel_status_="pending";}
  if(requests.cancel_manipulation){manipulation_cancel_pending_=true;manipulation_cancel_status_="pending";}
  if(requests.arm_return_home){home_pending_=true;home_port_status_="pending";}
  if(requests.arm_emergency){home_pending_=false;emergency_pending_=true;emergency_port_status_="pending";}
  if(supervisor_->system_state()!=SystemState::CONTROLLED_STOP)home_pending_=false;
  if(home_pending_){
   if(!arm_home_client_->service_is_ready())home_port_status_="unavailable";
   else {home_pending_=false;home_port_status_="request_sent";
    arm_home_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
     ControlLock lock(mutex_,control_latency_.get(),__LINE__);if(generation!=supervisor_->orchestration_generation())return;
     try{const auto result=future.get();home_port_status_=result->success?"accepted_waiting_measured_home":"rejected: "+result->message;supervisor_->ArmHomeRequestAccepted(result->success,SafetyClock::now());}
     catch(const std::exception& e){home_port_status_=e.what();supervisor_->ArmHomeRequestAccepted(false,SafetyClock::now());}
    });
   }
  }
  if(emergency_pending_){
   if(!arm_emergency_client_->service_is_ready())emergency_port_status_="unavailable";
   else {emergency_pending_=false;emergency_port_status_="request_sent";
    arm_emergency_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
     ControlLock lock(mutex_,control_latency_.get(),__LINE__);if(generation!=supervisor_->orchestration_generation())return;
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
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);if(generation!=supervisor_->orchestration_generation())return;
   auto& status=navigation?nav_cancel_status_:manipulation_cancel_status_;
   try{const auto result=future.get();status=(result->success?"accepted: ":"rejected: ")+result->message;}catch(const std::exception& e){status=e.what();}
  });
 }
 void Refresh(SafetyTime now) {
  inputs_.ready.lowstate_fresh=lowstate_.Fresh(now);inputs_.ready.remote_fresh=remote_->status().remote_valid;
  const bool mission_fresh=!mission_session_.empty()&&Age(now,mission_readiness_stamp_)>=0&&Age(now,mission_readiness_stamp_)<=supervisor_->profile().mission_timeout_s;
  inputs_.navigation_ready=navigation_gate_->Ready(now)&&nav_cancel_available_&&nav_input_unique_;
  inputs_.perception_ready=mission_fresh&&mission_perception_ready_&&manipulation_cancel_available_;
  inputs_.own_output_healthy=output_&&lease_&&lease_->acquired()&&cached_publisher_count_==1&&Age(now,graph_stamp_)>=0&&Age(now,graph_stamp_)<=supervisor_->profile().graph_timeout_s;
  supervisor_->Observe(inputs_,now);supervisor_->ConfirmStockObserved(now);
 }
 void StopPublisher(){supervisor_->Dispatch(SystemEvent::DISABLE_OUTPUT,SafetyClock::now());output_.reset();if(!sdk_release_pending_)lease_.reset();if(!output_&&!lease_)supervisor_->Dispatch(SystemEvent::OUTPUT_STOPPED,SafetyClock::now());}
 void TraceMotionIo(const unitree_go::msg::LowCmd& packet,SafetyTime now) {
  const double interval=last_trace_io_==SafetyTime{}?0:Age(now,last_trace_io_);last_trace_io_=now;
  if(interval>0){trace_io_min_=std::min(trace_io_min_,interval);trace_io_max_=std::max(trace_io_max_,interval);}
  if(last_trace_row_!=SafetyTime{}&&Age(now,last_trace_row_)<supervisor_->profile().trace_io_period_s)return;
  MotionTraceSample sample;sample.stamp_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  sample.phase=int(supervisor_->phase());sample.packets=sent_;sample.interval_s=interval;
  sample.io_min_s=std::isfinite(trace_io_min_)?trace_io_min_:0;sample.io_max_s=trace_io_max_;
  sample.low_age_s=Age(now,inputs_.lowstate_stamp);sample.remote_age_s=Age(now,inputs_.remote_stamp);
  sample.arm_age_s=Age(now,inputs_.arm_stamp);sample.target_age_s=Age(now,inputs_.target_stamp);
  sample.result_age_s=supervisor_->PolicyAgeSeconds(now);
  const auto& remote=remote_->status();sample.sticks={remote.ly,-remote.rx,-remote.lx};
  sample.requested=control_mode_=="autonomy"?navigation_gate_->Get(now):remote_test_requested_;sample.command=supervisor_->command();
  const auto& low=lowstate_.snapshot();sample.q=low.motor_q;sample.dq=low.motor_dq;sample.quat=low.quaternion_xyzw;
  for(int i=0;i<3;++i)sample.gyro[i]=low.gyro[i];
  for(int i=0;i<12;++i){sample.target[i]=packet.motor_cmd[i].q;sample.kp[i]=packet.motor_cmd[i].kp;sample.kd[i]=packet.motor_cmd[i].kd;}
  motion_trace_->Submit(sample);last_trace_row_=now;trace_io_min_=std::numeric_limits<double>::infinity();trace_io_max_=0;
 }
 public:
 void InitializePolicyWorker() {
  const auto& settings=InitializePolicyThread();
  if(policy_rt_)policy_rt_->Enter();
  thread_local bool logged=false;
  if(!logged){RCLCPP_INFO(get_logger(),"Policy worker effective OpenMP max=%d interop=%d (thread initialized)",settings.openmp_max,settings.interop);logged=true;}
 }
 private:
 void PolicyTick() {
  // Fallback covers the diagnostic shared executor, whose workers are owned
  // by rclcpp. Normal execution initializes once before policy_executor.spin.
  InitializePolicyWorker();
  if(worker_self_test_frames_) {
   torch::InferenceMode inference;
   if(worker_self_test_done_==0)core_.agent().ResetPolicyState();
   if(omp_get_max_threads()!=1)throw std::runtime_error("OpenMP changed in policy worker");
   const auto action=core_.agent().Act();
   if(action.numel()!=12||!torch::isfinite(action).all().item<bool>()||omp_get_max_threads()!=1)
    throw std::runtime_error("Read-only worker self test failed");
   ++worker_self_test_done_;
   if(worker_self_test_done_==1||worker_self_test_done_==worker_self_test_frames_) {
    int actual_team=0;
    #pragma omp parallel reduction(max:actual_team)
    {actual_team=omp_get_num_threads();}
    if(actual_team!=1)throw std::runtime_error("Policy worker OpenMP team oversubscribed");
    RCLCPP_INFO(get_logger(),"Read-only production worker self test frame=%d actual diagnostic team=%d; targets discarded",worker_self_test_done_,actual_team);
   }
   if(worker_self_test_done_>=worker_self_test_frames_)rclcpp::shutdown();
   return;
  }
  std::array<float,6> armq,armdq,target;LowStateSnapshot low;std::array<double,3> cmd{};bool infer=false,reset=false;
  std::optional<PolicyTicket> ticket;
  std::optional<MotionTraceSample> trace;
  SafetyTime trace_remote_stamp{};
  const auto callback_started=SafetyClock::now();
  size_t publishers;bool nav_unique;
  try {
   publishers=count_publishers("/lowcmd");
   nav_unique=count_publishers("/go2/mission/readiness")==1;
  }catch(const rclcpp::exceptions::RCLError&){
   // SIGINT may invalidate the context between executor dispatch and this
   // graph query. Only an already-shutdown context is a normal exit.
   if(!rclcpp::ok())return;
   throw;
  }
  const auto graph_observed=SafetyClock::now();
  {
   ControlLock lock(mutex_,control_latency_.get(),__LINE__);auto now=SafetyClock::now();
   SdkTick(now);
   cached_publisher_count_=publishers;graph_stamp_=callback_started;
   nav_input_unique_=nav_unique;
   const bool mission_fresh=!mission_session_.empty()&&Age(now,mission_readiness_stamp_)>=0&&Age(now,mission_readiness_stamp_)<=supervisor_->profile().mission_timeout_s;
   nav_cancel_available_=mission_fresh&&nav_cancel_client_->service_is_ready();
   manipulation_cancel_available_=mission_fresh&&manipulation_cancel_client_->service_is_ready();
   Refresh(now);SequenceTick(now);
   remote_test_requested_=RemoteStickCommand(remote_->status(),supervisor_->profile());
   if(!read_only_&&supervisor_->system_state()==SystemState::ACTIVE&&supervisor_->NeedsPolicy()) {
    std::array<double,3> selected{};
    if(motion_commands_enabled_) {
     if(control_mode_=="remote_test")selected=remote_test_requested_;
     else if(NavigationFresh(now))selected=navigation_gate_->Get(now);
    }
    const auto result=control_mode_=="autonomy"?supervisor_->NavigationCommand(selected,now):supervisor_->RemoteTestCommand(selected,now);
    if(result.success)last_deadman_=now;
   }
   reset=supervisor_->ConsumePolicyReset();ticket=supervisor_->BeginPolicy(now);infer=ticket.has_value();low=lowstate_.snapshot();
   armq=arm_q_;armdq=arm_dq_;target=arm_target_;cmd=supervisor_->command();
   if constexpr(compute_probe) {
    const bool fresh=lowstate_.Fresh(now)&&inputs_.ready.arm_feedback_ready&&inputs_.ready.arm_target_ready&&
     Age(now,inputs_.arm_stamp)>=0&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&
     Age(now,inputs_.target_stamp)>=0&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s;
    if(fresh) {
     ticket=PolicyTicket{++probe_jobs_,0,now,inputs_.lowstate_stamp,inputs_.arm_stamp,inputs_.target_stamp};
     infer=true;reset=probe_reset_;probe_reset_=false;cmd={};
    }else probe_reset_=true;
   }
   if(infer&&motion_trace_&&motion_trace_->enabled()) {
    trace.emplace();trace->kind=compute_probe?3:1;trace->phase=int(supervisor_->phase());
    trace->graph_ms=Age(graph_observed,callback_started)*1000.;
    trace->callback_lock_wait_ms=Age(now,graph_observed)*1000.;
    trace_remote_stamp=inputs_.remote_stamp;
    trace->interval_s=last_trace_policy_==SafetyTime{}?0:Age(now,last_trace_policy_);last_trace_policy_=now;
    const auto& remote=remote_->status();trace->sticks={remote.ly,-remote.rx,-remote.lx};
    trace->requested=control_mode_=="autonomy"?navigation_gate_->Get(now):remote_test_requested_;trace->command=cmd;trace->q=low.motor_q;trace->dq=low.motor_dq;trace->quat=low.quaternion_xyzw;
    for(int i=0;i<3;++i)trace->gyro[i]=low.gyro[i];
    trace->callback_lock_hold_ms=Age(SafetyClock::now(),now)*1000.;
   }
   // Serial/SDK remain exclusively in the RARS owner; leg node uses async ports.
   supervisor_->ConsumeArmHoldRequest();
  }
  if(infer) {
   try{
    // Thread-local: constructor guard does not carry into executor worker callbacks.
    // Cover reset/history/previous-action as well as module.forward to prevent graph retention.
    torch::InferenceMode inference;
    auto& a=core_.agent();core_.SetMeasuredLegs(low.motor_q,low.motor_dq);
    CopyPolicyInput(a.obs.base_quat,low.quaternion_xyzw);
    CopyPolicyInput(a.obs.ang_vel,low.gyro);
    CopyPolicyInput(a.obs.arm_pos,armq);CopyPolicyInput(a.obs.arm_vel,armdq);CopyPolicyInput(a.obs.arm_target,target);
    CopyPolicyInput(a.obs.command,std::array<float,3>{float(cmd[0]),float(cmd[1]),float(cmd[2])});
    if(reset){CopyPolicyInput(a.obs.command,std::array<float,3>{});a.ResetPolicyState();}
    PolicyStateTransaction policy_state(a);
    const auto began=SafetyClock::now();const auto cpu_began=trace?ThreadCpuNs():-1;
    auto action=a.Act();const auto* action_data=PolicyOutputData(action);
    std::array<float,12> q;for(int i=0;i<12;++i)q[i]=action_data[io_motor_to_policy[i]];
    const auto cpu_ended=trace?ThreadCpuNs():-1;
    const auto ended=SafetyClock::now();const double elapsed=std::chrono::duration<double,std::milli>(ended-began).count();
    if(trace) {
     trace->prepare_ms=Age(began,ticket->started)*1000.;
     trace->compute_cpu_ms=cpu_began>=0&&cpu_ended>=cpu_began?(cpu_ended-cpu_began)/1.e6:-1;
     try{const auto* raw_action=PolicyOutputData(a.obs.action);for(int i=0;i<12;++i)trace->action[i]=raw_action[io_motor_to_policy[i]];}
     catch(...){trace.reset();motion_trace_->Stop();}
    }
    const auto result_wait_started=SafetyClock::now();
    ControlLock lock(mutex_,control_latency_.get(),__LINE__);const auto completed=SafetyClock::now();
    bool accepted;
    if constexpr(compute_probe) {
     // Diagnostic only: no supervisor state changes or target delivery. The
     // same worker/observations/locks run; only timing validity is recorded.
     accepted=Age(completed,ticket->started)<=supervisor_->profile().policy_inference_timeout_s&&Age(completed,ticket->observation_stamp)<=supervisor_->profile().policy_observation_timeout_s&&
      Age(completed,ticket->arm_stamp)<=supervisor_->profile().arm_timeout_s&&
      Age(completed,ticket->target_stamp)<=supervisor_->profile().arm_timeout_s;
     if(!accepted)++probe_misses_;
     if(accepted)probe_result_stamp_=completed;
    }else accepted=supervisor_->PolicyResult(*ticket,q,elapsed,completed);
    last_policy_ms_=elapsed;
    if(accepted)policy_state.Commit();
    if(trace&&motion_trace_->enabled()) {
     trace->postprocess_ms=Age(result_wait_started,ended)*1000.;
     trace->result_lock_wait_ms=Age(completed,result_wait_started)*1000.;
     trace->stamp_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(completed.time_since_epoch()).count();
     trace->accepted=accepted;trace->target=q;trace->compute_ms=elapsed;trace->packets=sent_;
     trace->job_age_s=Age(completed,ticket->started);trace->low_age_s=Age(completed,ticket->observation_stamp);
     trace->remote_age_s=Age(completed,trace_remote_stamp);trace->arm_age_s=Age(completed,ticket->arm_stamp);trace->target_age_s=Age(completed,ticket->target_stamp);
     trace->result_age_s=supervisor_->PolicyAgeSeconds(completed);
     if(compute_probe)trace->result_age_s=probe_result_stamp_==SafetyTime{}?-1:Age(completed,probe_result_stamp_);
     try{motion_trace_->Submit(*trace);}catch(...){motion_trace_->Stop();}
    }
    if(!accepted)RCLCPP_WARN(get_logger(),"Policy result rejected: compute_ms=%g job_age_s=%g observation_age_s=%g captured_arm_age_s=%g captured_target_age_s=%g state=%s fault=%s",elapsed,Age(completed,ticket->started),Age(completed,ticket->observation_stamp),Age(completed,ticket->arm_stamp),Age(completed,ticket->target_stamp),R3StateName(supervisor_->state()),supervisor_->last_fault().c_str());
   }catch(const std::exception& e){ControlLock lock(mutex_,control_latency_.get(),__LINE__);if(compute_probe){++probe_exceptions_;probe_reset_=true;probe_error_=e.what();}else supervisor_->PolicyFailed(*ticket,std::string("policy_exception: ")+e.what(),SafetyClock::now());}
  }
  ControlLock lock(mutex_,control_latency_.get(),__LINE__);TraceState(SafetyClock::now());
 }
 ControlDiagnostics CaptureStatus(SafetyTime now) {
  const auto stand_errors=StandErrors();const auto worst_stand=std::max_element(stand_errors.begin(),stand_errors.end());
  const auto remote_bounds=supervisor_->profile().RemoteBounds();
  auto blockers=supervisor_->Blockers(now);ControlDiagnostics snapshot;
  snapshot.Add("policy_rt_registered",policy_rt_&&policy_rt_->ready.load());
  snapshot.Add("policy_rt_ready",policy_rt_&&policy_rt_->Ready());
  snapshot.Add("observed_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
  snapshot.Add("controller_session_id",controller_session_id_);
  snapshot.Add("state", SystemStateName(supervisor_->system_state()));
  snapshot.Add("phase", SystemPhaseName(supervisor_->phase()));
  snapshot.Add("legacy_state", R3StateName(supervisor_->state()));
  snapshot.Add("read_only", read_only_);
  snapshot.Add("output_enabled", supervisor_->output_enabled());
  snapshot.Add("custom_leg_output", (output_&&supervisor_->output_enabled()?"ON":"OFF"));
  snapshot.Add("lowcmd_lease_present", bool(lease_));
  snapshot.Add("output_stop_confirmed", supervisor_->output_stopped());
  snapshot.Add("passive_command_sent", (supervisor_->passive_packets_sent()>0));
  snapshot.Add("passive_packets_sent", supervisor_->passive_packets_sent());
  snapshot.Add("passive_packet_limit", supervisor_->profile().passive_packet_limit);
  snapshot.Add("control_mutex", "PTHREAD_PRIO_INHERIT");
  snapshot.Add("policy_execution",direct_policy_?"monotonic_thread":"ros_executor");
  snapshot.Add("control_latency_enabled",control_latency_&&control_latency_->enabled());
  snapshot.Add("control_latency_dropped",control_latency_?control_latency_->dropped():0);
  snapshot.Add("passive_timeout_s", supervisor_->profile().passive_timeout_s);
  snapshot.Add("passive_sequence_complete", supervisor_->passive_sequence_complete());
  snapshot.Add("last_commanded_leg_mode", supervisor_->last_commanded_leg_mode());
  snapshot.Add("motor_power_off_confirmed", false);
  snapshot.Add("lowcmd_publisher_present", bool(output_));
  snapshot.Add("sent_packets", sent_);
  snapshot.Add("sport_state", SportModeName(inputs_.sport));
  snapshot.Add("sport_age_s", Age(now,inputs_.sport_stamp));
  snapshot.Add("sport_monitor", (network_interface_.empty()?"external":"persistent_sdk_session"));
  snapshot.Add("sport_query_ok", sdk_last_ok_);
  snapshot.Add("sport_startup_only", supervisor_->profile().sport_startup_only);
  snapshot.Add("sport_startup_checked", supervisor_->sport_startup_checked());
  snapshot.Add("sport_release_verified", supervisor_->sport_release_verified());
  snapshot.Add("sport_runtime_monitoring", !supervisor_->profile().sport_startup_only);
  snapshot.Add("sport_status_requests", sdk_status_queries_);
  snapshot.Add("sport_release_requests", sdk_release_requests_);
  snapshot.Add("sport_rpc_ms", sdk_rpc_ms_);
  snapshot.Add("sport_query_failures", sdk_failures_);
  snapshot.Add("sport_query_inflight", sdk_worker_->busy());
  snapshot.Add("sport_query_age_s", (sdk_worker_->busy()?Age(now,sdk_started_):-1));
  snapshot.Add("sport_query_detail", sdk_detail_);
  snapshot.Add("lowstate_age_s", lowstate_.AgeMs(now)/1000);
  snapshot.Add("remote_age_s", remote_->status().remote_age_ms/1000);
  snapshot.Add("remote_mask", remote_->status().button_mask);
  snapshot.Add("remote_buttons", remote_->status().decoded_buttons);
  snapshot.Add("remote_event", int(last_event_));
  snapshot.Add("arm_feedback_age_s", Age(now,inputs_.arm_stamp));
  snapshot.Add("control_mode", control_mode_);
  snapshot.Add("motion_commands_enabled", motion_commands_enabled_);
  snapshot.Add("navigation_command_enabled", (inputs_.navigation_ready&&navigation_gate_->Enabled(now)));
  snapshot.Add("navigation_session_id", navigation_gate_->session());
  snapshot.Add("navigation_command_fresh", NavigationFresh(now));
  snapshot.Add("navigation_requested_command", std::array<double,3>{navigation_gate_->Get(now)[0],navigation_gate_->Get(now)[1],navigation_gate_->Get(now)[2]});
  snapshot.Add("stand_completion", "time");
  snapshot.Add("remote_test_mode", remote_test_mode_);
  snapshot.Add("remote_command_limits", remote_bounds);
  snapshot.Add("navigation_command_limits", std::array<double,3>{supervisor_->profile().vx_bound,supervisor_->profile().vy_bound,supervisor_->profile().wz_bound});
  snapshot.Add("remote_test_requested_command", remote_test_requested_);
  snapshot.Add("motion_command", supervisor_->command());
  snapshot.Add("arm_target_age_s", Age(now,inputs_.target_stamp));
  snapshot.Add("arm_ready", (inputs_.arm_home_ready&&inputs_.ready.arm_feedback_ready&&inputs_.ready.arm_target_ready&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s));
  snapshot.Add("arm_home_ready", (inputs_.arm_home_ready&&Age(now,inputs_.arm_stamp)>=0&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)>=0&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s));
  snapshot.Add("arm_control_ready", (inputs_.arm_control_ready&&Age(now,inputs_.arm_stamp)>=0&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)>=0&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s));
  snapshot.Add("operation_profile", OperationProfileName(supervisor_->operation_profile()));
  snapshot.Add("command_source", CommandSourceName(supervisor_->capabilities().command_source));
  snapshot.Add("capabilities", supervisor_->capabilities());
  snapshot.Add("own_output_healthy", inputs_.own_output_healthy);
  snapshot.Add("navigation_ready", inputs_.navigation_ready);
  snapshot.Add("perception_ready", inputs_.perception_ready);
  snapshot.Add("arm_emergency_validated", inputs_.arm_emergency_validated);
  snapshot.Add("stop_blocker", supervisor_->stop_blocker());
  snapshot.Add("arm_home_port", home_port_status_);
  snapshot.Add("arm_emergency_port", emergency_port_status_);
  snapshot.Add("navigation_cancel_port", nav_cancel_status_);
  snapshot.Add("manipulation_cancel_port", manipulation_cancel_status_);
  snapshot.Add("orchestration_generation", supervisor_->orchestration_generation());
  snapshot.Add("fixed_target_motor_q", supervisor_->fixed_target());
  snapshot.Add("lie_down_target_approved", supervisor_->LieDownTargetApproved());
  snapshot.Add("controlled_stop_lie_down_trial", supervisor_->profile().controlled_stop_lie_down_trial);
  snapshot.Add("lie_down_commissioning_active", (supervisor_->profile().controlled_stop_lie_down_trial&&!supervisor_->profile().lie_down_validated&&supervisor_->system_state()==SystemState::CONTROLLED_STOP&&!supervisor_->NeedsPolicy()));
  snapshot.Add("lie_down_dynamics_validated", supervisor_->profile().lie_down_validated);
  snapshot.Add("critical_control_fault", (supervisor_->fault_latched()&&supervisor_->last_fault()!="operator_emergency"));
  snapshot.Add("arm_error", arm_error_);
  snapshot.Add("measured_motor_q", inputs_.measured_q);
  snapshot.Add("stand_target_motor_q", supervisor_->profile().stand);
  snapshot.Add("stand_error_rad", stand_errors);
  snapshot.Add("stand_error_max_rad", *worst_stand);
  snapshot.Add("stand_error_motor_index", std::distance(stand_errors.begin(),worst_stand));
  snapshot.Add("remote_auto_sequence", automatic_sequence_);
  snapshot.Add("remote_sequence_active", supervisor_->remote_sequence_active());
  snapshot.Add("remote_sequence_message", sequence_message_);
  snapshot.Add("remote_sequence_blockers", supervisor_->RemoteSequenceBlockers(now));
  snapshot.Add("sdk_release_pending", sdk_release_pending_);
  snapshot.Add("lowcmd_discovery_waiting", lowcmd_wait_->active());
  snapshot.Add("lowcmd_discovery_wait_age_s", lowcmd_wait_->AgeSeconds(now));
  snapshot.Add("lowcmd_publisher_count", cached_publisher_count_);
  snapshot.Add("motion_diagnostics_active", (motion_trace_&&motion_trace_->enabled()));
  snapshot.Add("motion_diagnostics_dropped", (motion_trace_?motion_trace_->dropped():0));
  snapshot.Add("model_loaded", core_.loaded());
  snapshot.Add("policy_ms", last_policy_ms_);
  snapshot.Add("policy_result_age_s", supervisor_->PolicyAgeSeconds(now));
  snapshot.Add("policy_zero_handoff_pending", supervisor_->zero_policy_handoff_pending());
  snapshot.Add("policy_inference_age_s", supervisor_->PolicyInferenceAgeSeconds(now));
  snapshot.Add("policy_observation_age_s", supervisor_->PolicyObservationAgeSeconds(now));
  snapshot.Add("policy_observation_timeout_s", supervisor_->profile().policy_observation_timeout_s);
  snapshot.Add("policy_result_timeout_s", supervisor_->profile().policy_result_timeout_s);
  snapshot.Add("policy_inference_timeout_s", supervisor_->profile().policy_inference_timeout_s);
  snapshot.Add("zero_policy_handoff_timeout_s", supervisor_->profile().zero_policy_handoff_timeout_s);
  snapshot.Add("pd_handoff_lowstate_timeout_s", supervisor_->profile().pd_handoff_lowstate_timeout_s);
  snapshot.Add("mission_timeout_s", supervisor_->profile().mission_timeout_s);
  snapshot.Add("graph_timeout_s", supervisor_->profile().graph_timeout_s);
  snapshot.Add("policy_period_ms", supervisor_->profile().policy_period_ms);
  snapshot.Add("port_period_ms", supervisor_->profile().port_period_ms);
  snapshot.Add("status_period_ms", supervisor_->profile().status_period_ms);
  snapshot.Add("sdk_query_period_ms", supervisor_->profile().sdk_query_period_ms);
  snapshot.Add("sdk_query_timeout_ms", supervisor_->profile().sdk_query_timeout_ms);
  snapshot.Add("sdk_release_timeout_ms", supervisor_->profile().sdk_release_timeout_ms);
  snapshot.Add("trace_io_period_s", supervisor_->profile().trace_io_period_s);

  snapshot.Add("policy_inflight", supervisor_->policy_inflight());
  snapshot.Add("rejected_policy_results", supervisor_->rejected_policy_results());
  snapshot.Add("deadline_misses", supervisor_->deadline_misses());
  snapshot.Add("fault_latched", supervisor_->fault_latched());
  snapshot.Add("fault", supervisor_->last_fault());
  snapshot.Add("fault_phase", SystemPhaseName(supervisor_->fault_phase()));
  snapshot.Add("fault_blockers", supervisor_->fault_blockers());
  const auto ns=[](SafetyTime value){return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();};
  snapshot.Add("fault_observed_ns", ns(supervisor_->fault_time()));
  snapshot.Add("fault_lowstate_source_ns", ns(supervisor_->fault_inputs().lowstate_stamp));
  snapshot.Add("fault_arm_source_ns", ns(supervisor_->fault_inputs().arm_stamp));
  snapshot.Add("fault_target_source_ns", ns(supervisor_->fault_inputs().target_stamp));
  snapshot.Add("phase_warnings", supervisor_->PhaseWarnings(now));
  snapshot.Add("blockers", std::move(blockers));
  if(compute_probe||(motion_trace_&&motion_trace_->enabled()))snapshot.Add("runtime_io_interval_max_ms",io_interval_max_ms_);
  if constexpr(compute_probe) {
   snapshot.Add("compute_probe",true);snapshot.Add("live_robot_inputs",live_shadow);snapshot.Add("probe_jobs",probe_jobs_);
   snapshot.Add("probe_misses",probe_misses_);snapshot.Add("probe_exceptions",probe_exceptions_);
   snapshot.Add("probe_error",probe_error_);
  }
  return snapshot;
 }
 void StatusTick() {
  try {
   ControlDiagnostics snapshot;
   {ControlLock lock(mutex_,control_latency_.get(),__LINE__);snapshot=CaptureStatus(SafetyClock::now());}
   std_msgs::msg::String status;status.data=snapshot.Serialize();status_->publish(status);
  }catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Status formatting failed: %s",e.what());}
 }

 std::unique_ptr<PolicyRtRegistration> policy_rt_;
 std::unique_ptr<ControlLatency> control_latency_;
 PriorityMutex mutex_;RealControllerCore core_;LowStateReader lowstate_;
 std::optional<SystemPhase> traced_state_;std::string traced_fault_;bool traced_latched_=false;
 std::unique_ptr<R3Supervisor> supervisor_;std::unique_ptr<R3RemoteCommands> remote_;R3Inputs inputs_;
 std::shared_ptr<ModeSessionProcess> sdk_session_;
 std::unique_ptr<AsyncModeWorker> sdk_worker_;
 bool sdk_last_ok_=false;double sdk_rpc_ms_=-1;size_t sdk_failures_=0;SafetyTime sdk_started_{};
 std::unique_ptr<SportQuerySchedule> sdk_query_schedule_;size_t sdk_status_queries_=0,sdk_release_requests_=0;
 bool sdk_release_pending_=false,automatic_sequence_=true;std::string network_interface_,sdk_helper_,sdk_detail_,sequence_message_;
 std::unique_ptr<LowCmdDiscoveryWait> lowcmd_wait_;
 std::string controller_session_id_=std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(SafetyClock::now().time_since_epoch()).count());
 bool read_only_=true,remote_test_mode_=false,motion_commands_enabled_=false;std::string control_mode_;std::string output_lock_,arm_error_;
 std::array<double,3> remote_test_requested_{};
 std::unique_ptr<MotionTrace> motion_trace_;
 SafetyTime last_trace_io_{},last_trace_row_{},last_trace_policy_{};
 double trace_io_min_=std::numeric_limits<double>::infinity(),trace_io_max_=0;
 SafetyTime last_io_tick_{},probe_result_stamp_{};double io_interval_max_ms_=0;
 size_t probe_jobs_=0,probe_misses_=0,probe_exceptions_=0;
 bool probe_reset_=true,probe_shared_executor_=false;std::string probe_error_;
 bool direct_policy_=true;int64_t policy_period_ns_=0;
 int worker_self_test_frames_=0,worker_self_test_done_=0;
 struct ReceiveTrace {uint32_t sequence;int64_t callback,receive;};
 std::string receiver_trace_path_,io_trace_path_;
 std::vector<ReceiveTrace> receiver_trace_;std::vector<int64_t> io_trace_;
 size_t receiver_trace_dropped_=0,io_trace_dropped_=0;
 rclcpp::TimerBase::SharedPtr probe_stop_;
 std::unique_ptr<OutputLease> lease_;std::array<float,6> arm_q_{},arm_dq_{},arm_target_{};
 bool nav_input_unique_=false;
 std::unique_ptr<NavigationRequestGate> navigation_gate_;double last_policy_ms_=0,home_tolerance_=.15;size_t sent_=0;
 SafetyTime last_deadman_{};R3RemoteEvent last_event_=R3RemoteEvent::NONE;
 rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr output_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sport_sub_,arm_sub_,mission_readiness_sub_;
 size_t cached_publisher_count_=0;SafetyTime graph_stamp_{};
 bool nav_cancel_available_=false,manipulation_cancel_available_=false;
 bool mission_perception_ready_=false;SafetyTime mission_readiness_stamp_{};std::string mission_session_;

 rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_;
 std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
 rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_home_client_,arm_emergency_client_,nav_cancel_client_,manipulation_cancel_client_;
 bool home_pending_=false,emergency_pending_=false,nav_cancel_pending_=false,manipulation_cancel_pending_=false;
 std::string home_port_status_="idle",emergency_port_status_="idle",nav_cancel_status_="idle",manipulation_cancel_status_="idle";
 rclcpp::TimerBase::SharedPtr port_timer_;rclcpp::CallbackGroup::SharedPtr port_group_;
 rclcpp::TimerBase::SharedPtr io_timer_,policy_timer_,status_timer_;rclcpp::CallbackGroup::SharedPtr io_group_,policy_group_,status_group_,state_group_;
};
int main(int argc,char** argv){try{InitializePolicyProcess(argv);}catch(const std::exception& e){std::cerr<<"Startup failed closed: "<<e.what()<<'\n';return 1;}rclcpp::init(argc,argv);try{auto node=std::make_shared<R3Node>();
 std::function<void(int64_t,int64_t)> tick;
 if(node->DirectPolicy())tick=[&](int64_t expected,int64_t actual){node->PolicyCycle(expected,actual);};
 SpinControlExecutors(node,node->IoGroup(),node->PolicyGroup(),node->SharedProbeExecutor(),[&]{node->InitializePolicyWorker();},tick,node->PolicyPeriodNs());}
 catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("r3"),"Startup failed closed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
