#pragma once

#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace stage4d {
// Read-only physics snapshot. tau_cmd is the complete PD+feedforward request,
// not only LowCmd.tau. tau_act is generalized joint actuator torque, not a
// contact/weld reaction or an estimate of motor electrical torque.
inline std::string Go2TorqueHud(const mjModel* model, const mjData* data,
                                const std::array<double, 12>& requested,
                                const char* mode, bool navigation_active) {
  constexpr const char* names[] = {
      "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
      "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};
  std::ostringstream text;
  text << "GO2 LEGS [Nm]  " << mode << "\nNAV: "
       << (navigation_active ? "ACTIVE" : "IDLE")
       << "\nJoint       cmd      act   %lim\n";
  for (int i = 0; i < 12; ++i) {
    const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, names[i]);
    if (actuator < 0 || model->actuator_trntype[actuator] != mjTRN_JOINT) {
      text << names[i] << " unavailable\n";
      continue;
    }
    const int joint = model->actuator_trnid[2 * actuator];
    const double actual = data->qfrc_actuator[model->jnt_dofadr[joint]];
    double limit = (i % 3 == 2) ? 35.55 : 23.7; // unchanged bridge clamp
    if (model->jnt_actfrclimited[joint]) {
      const double mechanical = actual < 0 ? -model->jnt_actfrcrange[2 * joint]
                                           : model->jnt_actfrcrange[2 * joint + 1];
      if (mechanical > 0) limit = std::min(limit, mechanical);
    }
    text << std::left << std::setw(9) << names[i] << std::right << std::fixed
         << std::setprecision(2) << std::setw(8) << requested[i]
         << std::setw(9) << actual << std::setprecision(0)
         << std::setw(6) << 100 * std::abs(actual) / limit << '\n';
  }
  const int base = mj_name2id(model, mjOBJ_BODY, "base");
  if (base >= 0) {
    const mjtNum* q = data->xquat + 4 * base;
    constexpr double degrees = 180 / 3.14159265358979323846;
    const double roll = std::atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
    const double pitch = std::asin(std::clamp(2*(q[0]*q[2]-q[3]*q[1]), -1.0, 1.0));
    text << std::setprecision(1) << "BASE roll/pitch: " << roll * degrees
         << " / " << pitch * degrees << " deg\n";
  }
  const int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
  text << "FLOOR:";
  for (const char* leg : {"FR", "FL", "RR", "RL"}) {
    const std::string name = std::string(leg) + "_foot_collision";
    const int foot = mj_name2id(model, mjOBJ_GEOM, name.c_str());
    bool touching = false;
    for (int i = 0; i < data->ncon; ++i) {
      const auto& c = data->contact[i];
      if (c.efc_address >= 0 && ((c.geom1 == foot && c.geom2 == floor) ||
                               (c.geom2 == foot && c.geom1 == floor))) touching = true;
    }
    text << ' ' << leg << '=' << (floor < 0 || foot < 0 ? "?" : touching ? "1" : "0");
  }
  return text.str();
}
}  // namespace stage4d
