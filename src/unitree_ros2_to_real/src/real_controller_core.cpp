#include "real_controller_core.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace sim2real {
namespace {
Legs Array(const torch::Tensor& t) {
  if (t.dim()!=1 || t.numel()!=12 || !torch::isfinite(t).all().item<bool>())
    throw std::runtime_error("Expected twelve finite leg values");
  Legs a{}; for(int i=0;i<12;++i) a[i]=t[i].item<float>(); return a;
}
float Positive(const YAML::Node& root, const char* name) {
  float v=root[name].as<float>();
  if(!std::isfinite(v) || v<=0) throw std::runtime_error(std::string("Invalid ")+name);
  return v;
}
}
Legs PolicyToMotor(const Legs& p) { Legs m{}; for(int i=0;i<12;++i) m[i]=p[motor_to_policy[i]]; return m; }
Legs MotorToPolicy(const Legs& m) { Legs p{}; for(int i=0;i<12;++i) p[motor_to_policy[i]]=m[i]; return p; }
void RealControllerCore::Load(const std::string& config, const std::string& policy) {
  loaded_=false; mode_=Mode::DISARMED;
  if(config.empty() || policy.empty() || !std::filesystem::is_regular_file(config) ||
     !std::filesystem::is_regular_file(policy)) throw std::runtime_error("Explicit existing config_path and model_path required");
  const auto yaml=YAML::LoadFile(config);
  if(yaml["go2_rars01"]["observation_layout"].as<std::string>()!="go2_rars01_unified_v1")
    throw std::runtime_error("Real controller requires unified configuration");
  const auto d=yaml["real_deployment"];
  if(d["enable_actuator_output"].as<bool>()) throw std::runtime_error("R1 actuator output is unavailable");
  if(d["control_period_ms"].as<int>()!=20) throw std::runtime_error("Policy requires 20 ms");
  duration_=Positive(d,"stand_duration_sec"); hold_duration_=Positive(d,"hold_transition_sec");
  for(const auto* name:{"cmd_vel_timeout_sec","low_state_timeout_sec","arm_state_timeout_sec","max_linear_x","max_linear_y","max_yaw_rate"}) Positive(d,name);
  agent_.ReadYaml("go2_rars01",config);
  if(!agent_.Load_Model(policy)) throw std::runtime_error("TorchScript model loading failed");
  loaded_=true;
}
void RealControllerCore::SetMeasuredLegs(const Legs& q, const Legs& dq) {
  auto p=MotorToPolicy(q); auto v=MotorToPolicy(dq);
  for(int i=0;i<12;++i) if(!std::isfinite(p[i]) || !std::isfinite(v[i])) throw std::runtime_error("Nonfinite leg feedback");
  agent_.obs.dof_pos=torch::tensor(std::vector<float>(p.begin(),p.end()));
  agent_.obs.dof_vel=torch::tensor(std::vector<float>(v.begin(),v.end()));
}
bool RealControllerCore::RequestMode(Mode next, const Readiness& r) {
  if(next==Mode::DISARMED) { if(mode_!=Mode::FAULT) mode_=next; return mode_==next; }
  if(next==Mode::FAULT) { mode_=next; return true; }
  if(mode_==Mode::FAULT || !loaded_ || !r.All()) return false;
  if(next==Mode::RL && mode_!=Mode::HOLD) return false;
  if(next==Mode::RL) agent_.ResetPolicyState(); // measurements applied before transition
  if(next==Mode::STAND || next==Mode::HOLD_TRANSITION) transition_start_=Array(agent_.obs.dof_pos);
  mode_=next; return true;
}
Targets RealControllerCore::MapTargets(const Legs& q, bool rl) const {
  return {PolicyToMotor(q), PolicyToMotor(Array(rl?agent_.params.rl_kp:agent_.params.fixed_kp)),
          PolicyToMotor(Array(rl?agent_.params.rl_kd:agent_.params.fixed_kd)),
          PolicyToMotor(Array(agent_.params.torque_limits))};
}
std::optional<Targets> RealControllerCore::Tick(float elapsed) {
  if(mode_==Mode::DISARMED || mode_==Mode::FAULT || !loaded_) return std::nullopt;
  if(!std::isfinite(elapsed) || elapsed<0) throw std::runtime_error("Invalid transition time");
  torch::InferenceMode inference;
  if(mode_==Mode::RL) return MapTargets(Array(agent_.Act()),true);
  auto q=Array(agent_.params.default_dof_pos);
  if(mode_==Mode::STAND || mode_==Mode::HOLD_TRANSITION) {
    const float rate=std::clamp(elapsed/(mode_==Mode::STAND?duration_:hold_duration_),0.0F,1.0F);
    for(int i=0;i<12;++i) q[i]=transition_start_[i]*(1-rate)+q[i]*rate;
    if(rate==1) mode_=Mode::HOLD;
  }
  return MapTargets(q,false);
}
}  // namespace sim2real
