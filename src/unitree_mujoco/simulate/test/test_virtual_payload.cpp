#include "virtual_payload.hpp"
#include <algorithm>
#include <iostream>
#include <memory>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  try {
    require(argc == 2, "provide MJCF path");
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
        mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
    require(bool(m), error);
    const int joint = mj_name2id(m.get(), mjOBJ_JOINT, "virtual_payload_freejoint");
    const int weld = mj_name2id(m.get(), mjOBJ_EQUALITY, "virtual_payload_weld");
    const int end = mj_name2id(m.get(), mjOBJ_BODY, "End_link");
    const int body = mj_name2id(m.get(), mjOBJ_BODY, "virtual_payload");
    require(joint >= 0 && weld >= 0 && end >= 0 && body >= 0, "missing entities");
    for (double mass : {0.5, 0.0}) {
      stage4d::VirtualPayload payload({true, mass, {-0.04, 0, 0}});
      payload.Initialize(m.get());
      std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
      mj_resetDataKeyframe(m.get(), d.get(), 0);
      payload.Update(m.get(), d.get());
      require(!d->eq_active[weld], "weld active before request");
      for (int i=0; i<m->jnt_dofadr[joint]; ++i) d->qvel[i]=0.01*std::sin(i+1);
      mj_forward(m.get(), d.get());
      std::vector<mjtNum> before(d->qacc, d->qacc + m->nv);
      std::vector<mjtNum> velocity_before(d->qvel, d->qvel + m->jnt_dofadr[joint]);
      payload.Attach(m.get(), d.get());
      for (int i=0; i<m->jnt_dofadr[joint]; ++i)
        require(d->qvel[i] == velocity_before[i], "attach changes robot velocity");
      const int q = m->jnt_qposadr[joint];
      for (int i=0; i<3; ++i)
        require(std::abs(d->qpos[q+i] - (d->xpos[3*end+i] - 0.04*d->xmat[9*end+3*i])) < 1e-10,
                "incorrect attachment transform");
      require(bool(d->eq_active[weld]) == (mass > 0), "wrong weld state");
      if (mass > 0) {
        double residual = 0, speed_residual = 0, difference = 0;
        for (int i=0; i<d->nefc; ++i)
          if (d->efc_type[i] == mjCNSTR_EQUALITY && d->efc_id[i] == weld) {
            residual = std::max(residual, std::abs(d->efc_pos[i]));
            speed_residual = std::max(speed_residual, std::abs(d->efc_vel[i]));
          }
        for (int i=0; i<m->jnt_dofadr[joint]; ++i)
          difference = std::max(difference, std::abs(d->qacc[i]-before[i]));
        require(residual < 1e-9, "activation pulls payload from parking");
        require(speed_residual < 1e-9, "incompatible attachment velocity");
        require(difference > 1e-5, "payload has no physical effect");
        std::cout << "mass=" << mass << " weld_residual=" << residual
                  << " weld_speed_residual=" << speed_residual
                  << " robot_qacc_change=" << difference << '\n';
      } else {
        for (int i=0; i<m->jnt_dofadr[joint]; ++i)
          require(std::abs(d->qacc[i]-before[i]) < 1e-10, "zero mass affects robot");
      }
      for (int repeat=0; repeat<3; ++repeat) {
        payload.Attach(m.get(), d.get()); // idempotent
        d->time = 1;
        payload.Update(m.get(), d.get());
        mj_resetDataKeyframe(m.get(), d.get(), 0);
        require(payload.Update(m.get(), d.get()), "reset not detected");
        require(!payload.attached() && !d->eq_active[weld] && d->qpos[q+2] == -10,
                "reset failed to detach/park");
        payload.Attach(m.get(), d.get());
      }
      payload.Detach(m.get(), d.get());
    }
    stage4d::VirtualPayload disabled({false, 0.5, {-0.04, 0, 0}});
    disabled.Initialize(m.get());
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    bool rejected=false;
    try { disabled.Attach(m.get(), d.get()); } catch (const std::runtime_error&) { rejected=true; }
    require(rejected && !d->eq_active[weld], "disabled feature allows attach");
    std::cout << "PASS placement, physical load, zero-load, reset x3, disabled\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n'; return 1;
  }
}
