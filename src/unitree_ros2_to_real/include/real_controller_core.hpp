#pragma once
#include "rl_agent.h"
#include <array>
#include <optional>

namespace sim2real {
using Legs = std::array<float, 12>;
inline constexpr std::array<int, 12> motor_to_policy{3,4,5,0,1,2,9,10,11,6,7,8};
Legs PolicyToMotor(const Legs& policy);
Legs MotorToPolicy(const Legs& motor);
enum class Mode { DISARMED, STAND, HOLD, RL, HOLD_TRANSITION, FAULT };
struct Readiness {
  bool model_loaded = false;
  bool low_state_fresh = false;
  bool arm_state_fresh = false;
  bool ownership_verified = false;
  bool All() const { return model_loaded && low_state_fresh && arm_state_fresh && ownership_verified; }
};
// Gate is independent of computation. R1 has no actuator transport.
class OutputGate {
 public:
  bool Allows(const Readiness& r) const { return armed_ && !fault_ && r.All(); }
  bool Arm(const Readiness& r) { armed_ = !fault_ && r.All(); return armed_; }
  void Disarm() { armed_ = false; }
  void Fault() { fault_ = true; Disarm(); }
 private:
  bool armed_ = false;
  bool fault_ = false;
};
struct Targets { Legs q{}, kp{}, kd{}, torque_limits{}; };
// Transport-free controller: no ROS/DDS, SDK, serial, LowCmd or CRC.
class RealControllerCore {
 public:
  void Load(const std::string& config, const std::string& policy);
  bool RequestMode(Mode mode, const Readiness& readiness);
  std::optional<Targets> Tick(float elapsed_sec);
  Mode mode() const { return mode_; }
  bool loaded() const { return loaded_; }
  Agent& agent() { return agent_; }
  void SetMeasuredLegs(const Legs& q, const Legs& dq);
 private:
  Targets MapTargets(const Legs& q, bool rl) const;
  Agent agent_;
  Mode mode_ = Mode::DISARMED;
  bool loaded_ = false;
  Legs transition_start_{};
  float duration_ = 8.0F;
  float hold_duration_ = 1.0F;
};
}  // namespace sim2real
