#include "go2_torque_hud.hpp"
#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("provide MJCF");
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    mj_resetDataKeyframe(model.get(), data.get(), 0);
    mj_forward(model.get(), data.get());
    std::array<double, 12> request{};
    request[0] = 100;
    int joint = mj_name2id(model.get(), mjOBJ_JOINT, "FR_hip_joint");
    data->qfrc_actuator[model->jnt_dofadr[joint]] = -23.7;
    const double time_before = data->time;
    const auto text = stage4d::Go2TorqueHud(model.get(), data.get(), request, "SAFE_TIMEOUT", false);
    if (text.find("FR_hip") == std::string::npos || text.find("100.00") == std::string::npos ||
        text.find("-23.70") == std::string::npos || text.find("BASE roll/pitch") == std::string::npos ||
        text.find("FLOOR: FR=") == std::string::npos || text.find("RL_calf") == std::string::npos ||
        data->time != time_before || data->ctrl[0] != 0)
      throw std::runtime_error("incorrect or mutating HUD snapshot");
    std::cout << text << "\nPASS requested/actual distinction, all 12 joints, read-only snapshot\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
