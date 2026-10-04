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
}
class R3Node final:public rclcpp::Node {
 public:
 R3Node():Node("go2_r3_commissioning") {
  at::set_num_threads(1);at::set_num_interop_threads(1);
  const auto config=declare_parameter<std::string>("config_path","");
  const auto model=declare_parameter<std::string>("model_path","");
  read_only_=declare_parameter<bool>("read_only",true);
  automatic_sequence_=declare_parameter<bool>("remote_auto_sequence",true);
  network_interface_=declare_parameter<std::string>("network_interface","");
  sdk_helper_=(std::filesystem::read_symlink("/proc/self/exe").parent_path()/"go2_mode_switch").string();
  if(declare_parameter<bool>("enable_actuator_output",false))throw std::runtime_error("startup output must be false; use explicit service gate");
  core_.Load(config,model);const auto y=YAML::LoadFile(config),d=y["real_deployment"],r=d["r3_commissioning"];
  home_tolerance_=d["rars01"]["auto_home"]["home_tolerance_rad"].as<double>();
  if(!std::isfinite(home_tolerance_)||home_tolerance_<=0)throw std::runtime_error("Invalid HOME tolerance");
  const auto profile=LoadR3Profile(y);supervisor_=std::make_unique<R3Supervisor>(profile);
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
  arm_sub_=create_subscription<std_msgs::msg::String>("/rars01/commissioning/state",10,[this](std_msgs::msg::String::ConstSharedPtr m){
   std::lock_guard lock(mutex_);inputs_.ready.arm_feedback_ready=false;inputs_.ready.arm_target_ready=false;inputs_.arm_static_hold=false;inputs_.arm_home_ready=false;
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
    if(y["target_valid"].as<bool>()) {
     arm_target_=Six(y["accepted_target"]);
     if(inputs_.arm_home_ready)for(float v:arm_target_)if(v!=0)throw std::runtime_error("HOME accepted target must be zero");
     inputs_.target_stamp=TimeNs(y["accepted_ns"].as<int64_t>());
     inputs_.ready.arm_target_ready=Age(now,inputs_.target_stamp)>=0;
    }
   }catch(const std::exception& e){inputs_.ready.arm_feedback_ready=false;inputs_.ready.arm_target_ready=false;inputs_.arm_static_hold=false;inputs_.arm_home_ready=false;arm_error_=e.what();}
  });
  enable_=create_service<std_srvs::srv::SetBool>("/go2/commissioning/enable_output",[this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res){
   std::lock_guard lock(mutex_);const auto now=SafetyClock::now();Refresh(now);
   supervisor_->CancelRemoteSequence();
   const auto result=SetOutput(req->data,now);res->success=result.success;res->message=result.message;
  });
  Trigger("request_hold",[this](auto now){return supervisor_->RequestHold(now);});
  Trigger("request_stand",[this](auto now){return supervisor_->RequestStand(now);});
  Trigger("request_rl",[this](auto now){return supervisor_->RequestRl(now);});
  Trigger("request_lie_down",[this](auto now){return supervisor_->RequestLieDown(now);});
  Trigger("controlled_abort",[this](auto now){return supervisor_->ControlledAbort(now);});
  Trigger("emergency_damp",[this](auto now){return supervisor_->Emergency(now);});
  Trigger("request_return_to_stock",[this](auto now){return supervisor_->RequestReturnToStock(now);});
  manual_=create_service<std_srvs::srv::Trigger>("/go2/commissioning/manual_step",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
   std::lock_guard lock(mutex_);const auto now=SafetyClock::now();Refresh(now);
   if(!pending_manual_){res->success=false;res->message="fresh staged manual command required";return;}
   const auto result=supervisor_->ManualCommand(pending_command_,pending_duration_,now);pending_manual_=false;
   if(result.success)last_deadman_=now;
   res->success=result.success;res->message=result.message;
  });
  stage_sub_=create_subscription<std_msgs::msg::String>("/go2/commissioning/manual_request",1,[this](std_msgs::msg::String::ConstSharedPtr m){
   std::lock_guard lock(mutex_);pending_manual_=false;
   try{auto y=YAML::Load(m->data);auto v=y["command"].as<std::vector<double>>();if(v.size()!=3)throw std::runtime_error("manual size");
    const auto now=SafetyClock::now();const auto stamp=TimeNs(y["sent_ns"].as<int64_t>());if(Age(now,stamp)<0||Age(now,stamp)>.1)throw std::runtime_error("manual message age");
    std::copy(v.begin(),v.end(),pending_command_.begin());pending_duration_=y["duration_s"].as<double>();pending_stamp_=now;pending_manual_=true;
   }catch(...){}
  });
  deadman_sub_=create_subscription<std_msgs::msg::String>("/go2/commissioning/manual_deadman",1,[this](std_msgs::msg::String::ConstSharedPtr m){
   std::lock_guard lock(mutex_);try{auto y=YAML::Load(m->data);auto stamp=TimeNs(y["sent_ns"].as<int64_t>());const auto now=SafetyClock::now();
    if(Age(now,stamp)>=0&&Age(now,stamp)<.1&&supervisor_->state()==R3State::RL_ACTIVE){last_deadman_=now;supervisor_->RenewDeadman(now);}
   }catch(...){}
  });
  io_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const int io_ms=r["lowcmd_period_ms"].as<int>();if(io_ms!=2)throw std::runtime_error("Current transport profile expects SDK example 2ms period");
  io_timer_=create_wall_timer(std::chrono::milliseconds(io_ms),[this]{
   std::lock_guard lock(mutex_);const auto now=SafetyClock::now();Refresh(now);
   if(supervisor_->state()==R3State::RL_ACTIVE&&Age(now,last_deadman_)>supervisor_->profile().command_timeout_s)
    supervisor_->ManualCommand({},.1,now);
   auto packet=supervisor_->Tick(now);
   if(output_&&packet&&!supervisor_->AllowsPacket(*packet,now))supervisor_->Fault("crc_or_packet_validation",now);
   else if(output_&&packet) {
    try{output_->publish(*packet);++sent_;}catch(...){supervisor_->Fault("transport_exception",now);StopPublisher();}
   }
   if(output_&&!supervisor_->output_enabled())StopPublisher();
  },io_group_);
  policy_timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{PolicyTick();});
  RCLCPP_INFO(get_logger(),"R3 read_only=%d; startup disarmed; remote sequence requires fresh chord and all prerequisites",read_only_);
 }
 ~R3Node(){if(sdk_future_.valid())sdk_future_.wait();}
 private:
 R3Reply SetOutput(bool enable,SafetyTime now) {
  if(enable&&sdk_release_pending_)return {false,"SDK release still pending"};
  if(enable&&read_only_)return {false,"read_only=true: physical output not available"};
  if(enable&&count_publishers("/lowcmd")!=0)return {false,"existing LowCmd publisher"};
  if(enable&&!lease_){lease_=std::make_unique<OutputLease>(output_lock_);if(!lease_->acquired()){lease_.reset();return {false,"output lease unavailable"};}}
  const auto result=supervisor_->EnableOutput(enable,now);
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
  if(read_only_||!automatic_sequence_||sdk_release_pending_)return;
  // A pending read-only query must finish before the single release request.
  if(sdk_future_.valid()&&!output_)return;
  const auto action=supervisor_->RemoteSequenceNext(now);R3Reply result{true,""};
  switch(action) {
   case R3SequenceAction::RELEASE_SPORT:
    if(network_interface_.empty()||count_publishers("/lowcmd")!=0){result={false,"SDK interface missing / foreign LowCmd publisher"};break;}
    if(!lease_)lease_=std::make_unique<OutputLease>(output_lock_);
    if(!lease_->acquired()){lease_.reset();result={false,"output lease busy before release"};break;}
    StartSdk(true,now);break;
   case R3SequenceAction::ENABLE_OUTPUT:result=SetOutput(true,now);break;
   case R3SequenceAction::STAND:result=supervisor_->RequestStand(now);break;
   case R3SequenceAction::RL:result=supervisor_->RequestRl(now);break;
   default:break;
  }
  if(!result.success){sequence_message_=result.message;supervisor_->Fault("remote_sequence_failed: "+result.message,now);}
 }
 void Trigger(const std::string& name,std::function<R3Reply(SafetyTime)> request) {
  services_.push_back(create_service<std_srvs::srv::Trigger>("/go2/commissioning/"+name,[this,request](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
   std::lock_guard lock(mutex_);auto now=SafetyClock::now();Refresh(now);auto r=request(now);res->success=r.success;res->message=r.message;
  }));
 }
 void Refresh(SafetyTime now) {
  inputs_.ready.lowstate_fresh=lowstate_.Fresh(now);inputs_.ready.remote_fresh=remote_->status().remote_valid;
  if(pending_manual_&&Age(now,pending_stamp_)>.1)pending_manual_=false;
  supervisor_->Observe(inputs_,now);supervisor_->ConfirmStockObserved(now);
 }
 void StopPublisher(){supervisor_->EnableOutput(false,SafetyClock::now());output_.reset();if(!sdk_release_pending_)lease_.reset();supervisor_->ConfirmOutputStopped();}
 void PolicyTick() {
  std::array<float,6> armq,armdq,target;LowStateSnapshot low;std::array<double,3> cmd{};bool infer=false,reset=false;
  {
   std::lock_guard lock(mutex_);auto now=SafetyClock::now();
   SdkTick(now);const auto event=remote_->Poll(now);Refresh(now);
   switch(event){case R3RemoteEvent::TAKEOVER:{
     const auto reply=(!read_only_&&automatic_sequence_)?supervisor_->StartRemoteSequence(now):supervisor_->Takeover(now);
     sequence_message_=reply.message;break;
    }
    case R3RemoteEvent::CONTROLLED_ABORT:supervisor_->ControlledAbort(now);break;
    case R3RemoteEvent::EMERGENCY:supervisor_->Emergency(now);break;default:break;}
   if(event!=R3RemoteEvent::NONE)last_event_=event;
   Refresh(now);SequenceTick(now);
   infer=supervisor_->NeedsPolicy();reset=supervisor_->ConsumePolicyReset();low=lowstate_.snapshot();
   armq=arm_q_;armdq=arm_dq_;target=arm_target_;cmd=supervisor_->command();
   // Arm lifecycle is independent. Abort/emergency never replaces its HOME
   // target or issues SDK commands; only consume the historical intent flag.
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
    std::lock_guard lock(mutex_);supervisor_->PolicyResult(q,elapsed,ended);last_policy_ms_=elapsed;
   }catch(const std::exception& e){std::lock_guard lock(mutex_);supervisor_->Fault(std::string("policy_exception: ")+e.what(),SafetyClock::now());}
  }
  std::lock_guard lock(mutex_);const auto now=SafetyClock::now();
  const auto blockers=supervisor_->Blockers(now);YAML::Emitter e;
  e<<YAML::Flow<<YAML::BeginMap<<YAML::Key<<"state"<<YAML::Value<<R3StateName(supervisor_->state())
   <<YAML::Key<<"read_only"<<YAML::Value<<read_only_<<YAML::Key<<"output_enabled"<<YAML::Value<<supervisor_->output_enabled()
   <<YAML::Key<<"lowcmd_publisher_present"<<YAML::Value<<bool(output_)<<YAML::Key<<"sent_packets"<<YAML::Value<<sent_
   <<YAML::Key<<"sport_state"<<YAML::Value<<SportModeName(inputs_.sport)<<YAML::Key<<"sport_age_s"<<YAML::Value<<Age(now,inputs_.sport_stamp)
   <<YAML::Key<<"lowstate_age_s"<<YAML::Value<<lowstate_.AgeMs(now)/1000<<YAML::Key<<"remote_age_s"<<YAML::Value<<remote_->status().remote_age_ms/1000
   <<YAML::Key<<"remote_mask"<<YAML::Value<<remote_->status().button_mask<<YAML::Key<<"remote_buttons"<<YAML::Value<<remote_->status().decoded_buttons
   <<YAML::Key<<"remote_event"<<YAML::Value<<int(last_event_)<<YAML::Key<<"arm_feedback_age_s"<<YAML::Value<<Age(now,inputs_.arm_stamp)
   <<YAML::Key<<"arm_target_age_s"<<YAML::Value<<Age(now,inputs_.target_stamp)<<YAML::Key<<"arm_ready"<<YAML::Value<<(inputs_.arm_home_ready&&inputs_.ready.arm_feedback_ready&&inputs_.ready.arm_target_ready&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s)
   <<YAML::Key<<"arm_home_ready"<<YAML::Value<<(inputs_.arm_home_ready&&Age(now,inputs_.arm_stamp)>=0&&Age(now,inputs_.arm_stamp)<=supervisor_->profile().arm_timeout_s&&Age(now,inputs_.target_stamp)>=0&&Age(now,inputs_.target_stamp)<=supervisor_->profile().arm_timeout_s)
   <<YAML::Key<<"arm_error"<<YAML::Value<<arm_error_
   <<YAML::Key<<"remote_auto_sequence"<<YAML::Value<<automatic_sequence_
   <<YAML::Key<<"remote_sequence_active"<<YAML::Value<<supervisor_->remote_sequence_active()
   <<YAML::Key<<"remote_sequence_message"<<YAML::Value<<sequence_message_
   <<YAML::Key<<"remote_sequence_blockers"<<YAML::Value<<supervisor_->RemoteSequenceBlockers(now)
   <<YAML::Key<<"sdk_release_pending"<<YAML::Value<<sdk_release_pending_
   <<YAML::Key<<"model_loaded"<<YAML::Value<<core_.loaded()<<YAML::Key<<"policy_ms"<<YAML::Value<<last_policy_ms_
   <<YAML::Key<<"deadline_misses"<<YAML::Value<<supervisor_->deadline_misses()<<YAML::Key<<"fault_latched"<<YAML::Value<<supervisor_->fault_latched()
   <<YAML::Key<<"fault"<<YAML::Value<<supervisor_->last_fault()<<YAML::Key<<"blockers"<<YAML::Value<<blockers<<YAML::EndMap;
  std_msgs::msg::String status;status.data=e.c_str();status_->publish(status);
 }
 std::mutex mutex_;RealControllerCore core_;LowStateReader lowstate_;
 std::unique_ptr<R3Supervisor> supervisor_;std::unique_ptr<R3RemoteCommands> remote_;R3Inputs inputs_;
 std::future<ModeProcessResult> sdk_future_;SafetyTime sdk_started_{},next_sdk_query_{};
 bool sdk_release_pending_=false,automatic_sequence_=true;std::string network_interface_,sdk_helper_,sdk_detail_,sequence_message_;
 bool read_only_=true,pending_manual_=false;std::string output_lock_,arm_error_;
 std::unique_ptr<OutputLease> lease_;std::array<float,6> arm_q_{},arm_dq_{},arm_target_{};
 std::array<double,3> pending_command_{};double pending_duration_=0,last_policy_ms_=0,home_tolerance_=.15;size_t sent_=0;
 SafetyTime pending_stamp_{},last_deadman_{};R3RemoteEvent last_event_=R3RemoteEvent::NONE;
 rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr output_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sport_sub_,arm_sub_,stage_sub_,deadman_sub_;
 rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_;
 std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
 rclcpp::TimerBase::SharedPtr io_timer_,policy_timer_;rclcpp::CallbackGroup::SharedPtr io_group_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<R3Node>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);executor.add_node(node);executor.spin();}
 catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("r3"),"Startup failed closed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
