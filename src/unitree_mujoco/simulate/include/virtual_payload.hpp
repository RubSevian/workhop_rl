#pragma once
#include <mujoco/mujoco.h>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace stage4d {
struct PayloadSettings {
  bool enabled = false;
  double mass = 0.5;
  std::array<double, 3> offset{-0.04, 0, 0};
};

// Called exclusively by the physics owner under the simulator mutex.
class VirtualPayload {
 public:
  explicit VirtualPayload(PayloadSettings settings) : settings_(settings) {
    if (!std::isfinite(settings.mass) || settings.mass < 0)
      throw std::invalid_argument("payload mass must be finite and nonnegative");
    for (double value : settings.offset)
      if (!std::isfinite(value)) throw std::invalid_argument("invalid payload offset");
  }
  void Initialize(mjModel* model) {
    body_ = mj_name2id(model, mjOBJ_BODY, "virtual_payload");
    end_ = mj_name2id(model, mjOBJ_BODY, "End_link");
    weld_ = mj_name2id(model, mjOBJ_EQUALITY, "virtual_payload_weld");
    joint_ = mj_name2id(model, mjOBJ_JOINT, "virtual_payload_freejoint");
    geom_ = mj_name2id(model, mjOBJ_GEOM, "virtual_payload_geom");
    if (body_ < 0 || end_ < 0 || weld_ < 0 || joint_ < 0 || geom_ < 0) {
      available_ = false;
      if (settings_.enabled) throw std::runtime_error("payload model entities missing");
      return;
    }
    available_ = true;
    // MuJoCo requires positive inertia for a free body. A zero-load test
    // keeps the parked positive-inertia body disconnected (never welded).
    const double mass = settings_.mass > 0 ? settings_.mass : 1e-6;
    model->body_mass[body_] = mass;
    for (int i = 0; i < 3; ++i) model->body_inertia[3*body_+i] = mass * 0.06*0.06/6;
    model->body_gravcomp[body_] = 1;
    model->geom_rgba[4*geom_+3] = 0;
    model->eq_active0[weld_] = 0;
    // Body weld data: anchor(3), relative position(3), quaternion(4), torque scale.
    mjtNum* rel = model->eq_data + mjNEQDATA*weld_;
    for (int i = 0; i < 3; ++i) { rel[i] = 0; rel[3+i] = settings_.offset[i]; }
    rel[6] = 1; rel[7] = rel[8] = rel[9] = 0;
    mjData* temporary = mj_makeData(model);
    mj_setConst(model, temporary);
    mj_deleteData(temporary);
    attached_ = false;
    last_time_ = -1;
    jacp_.resize(3*model->nv); jacr_.resize(3*model->nv);
  }
  bool Update(mjModel* model, mjData* data) {
    if (!available_) return false;
    const bool reset = data->time < last_time_ ||
        (attached_ && settings_.mass > 0 && !data->eq_active[weld_]);
    if (reset) { Detach(model, data); ++reset_count_; }
    last_time_ = data->time;
    if (!attached_ || settings_.mass == 0) Park(model, data);
    return reset;
  }
  void Attach(mjModel* model, mjData* data) {
    if (!settings_.enabled || !available_) throw std::runtime_error("payload feature disabled/unavailable");
    if (attached_) return;
    mj_forward(model, data);
    const int q = model->jnt_qposadr[joint_], v = model->jnt_dofadr[joint_];
    mjtNum rotated[3];
    mju_mulMatVec(rotated, data->xmat+9*end_, settings_.offset.data(), 3, 3);
    for (int i = 0; i < 3; ++i) data->qpos[q+i] = data->xpos[3*end_+i] + rotated[i];
    mju_copy(data->qpos+q+3, data->xquat+4*end_, 4);
    mj_jac(model, data, jacp_.data(), jacr_.data(), data->qpos+q, end_);
    mjtNum omega[3], velocity[3];
    mju_mulMatVec(velocity, jacp_.data(), data->qvel, 3, model->nv);
    mju_mulMatVec(omega, jacr_.data(), data->qvel, 3, model->nv);
    mju_copy(data->qvel+v, velocity, 3);
    mju_mulMatTVec(data->qvel+v+3, data->xmat+9*end_, omega, 3, 3);
    model->body_gravcomp[body_] = settings_.mass > 0 ? 0 : 1;
    // Placement and compatible velocity precede constraint activation.
    data->eq_active[weld_] = settings_.mass > 0;
    model->geom_rgba[4*geom_+3] = settings_.mass > 0 ? 1 : 0;
    attached_ = true;
    mj_forward(model, data);
  }
  void Detach(mjModel* model, mjData* data) {
    if (!available_) return;
    data->eq_active[weld_] = 0;
    attached_ = false;
    model->body_gravcomp[body_] = 1;
    model->geom_rgba[4*geom_+3] = 0;
    Park(model, data);
  }
  bool attached() const { return attached_; }
  unsigned reset_count() const { return reset_count_; }
  const PayloadSettings& settings() const { return settings_; }
 private:
  void Park(const mjModel* model, mjData* data) {
    const int q = model->jnt_qposadr[joint_], v = model->jnt_dofadr[joint_];
    mju_zero(data->qpos+q, 7); data->qpos[q+2] = -10; data->qpos[q+3] = 1;
    mju_zero(data->qvel+v, 6);
  }
  PayloadSettings settings_;
  int body_=-1, end_=-1, weld_=-1, joint_=-1, geom_=-1;
  bool available_=false, attached_=false;
  double last_time_=-1;
  unsigned reset_count_=0;
  std::vector<mjtNum> jacp_, jacr_;
};
}  // namespace stage4d
