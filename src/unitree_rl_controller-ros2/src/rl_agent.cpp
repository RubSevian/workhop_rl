#include <torch/torch.h>
#include "unitree_rl_controller/rl_agent.h"
#include <map>
#include "std_msgs/msg/string.hpp"

torch::Tensor Agent::quat_rotate_inverse(torch::Tensor q, torch::Tensor v) {
    torch::Tensor q_w = q.index({3});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(torch::indexing::None, 3)});
    torch::Tensor a = v * (2.0 * q_w * q_w - 1.0);
    torch::Tensor b = torch::cross(q_vec, v) * q_w * 2.0;
    torch::Tensor c = q_vec * torch::matmul(q_vec.view({1, 3}), v.view({3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

// Agent::Agent()
// {
//     dof_pos = torch::zeros({12});
//     dof_vel = torch::zeros({12});
//     base_linear_velocity = torch::zeros({3});
//     base_angular_velocity = torch::zeros({3});
//     base_linear_acceleration = torch::zeros({3});
//     orientation = torch::tensor({0.0, 0.0, 0.0, 1.0});
//     commands = torch::zeros({3});
//     mode = "trotting";
// }


bool Agent::load_model(std::string model_path)
{
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(model_path);
    }
    catch (const c10::Error& e) {
        return false;
    }
    return true;
}

// bool Agent::load_model_dict(std::map<std::string, std::string> model_paths)
// {
//     try {
//         for (const auto& [k, v] : model_paths) {
//             module_dict[k] = torch::jit::load(v);
//         }
//     }
//     catch (const c10::Error& e) {
//         return false;
//     }
//     return true;
// }

// torch::Tensor Agent::get_observations()
// {
//     projected_gravity = quat_rotate_inverse(orientation, _gravity_vector);
//     torch::Tensor observations = torch::cat({
//         (base_linear_acceleration + projected_gravity * 9.81) * scales.lin_vel,
//         base_angular_velocity * scales.ang_vel,
//         projected_gravity,
//         commands * scales.commands,
//         dof_pos * scales.dof_pos,
//         dof_vel * scales.dof_vel,
//         _previous_actions
//     });

//     std::cout << "observations " << observations << std::endl;
//     return observations;
// }

torch::Tensor Agent::act()
{
    // torch::Tensor observations = ComputeObservation();
    // // std::cout << "GRAVITY: " << projected_gravity << std::endl;
    // // Create a vector of inputs.
    // // 2. Создаём входной вектор для модели
    // std::vector<torch::jit::IValue> inputs;
    // inputs.push_back(observations);

    torch::Tensor actions = this->Forward();

    // // 4. Сохраняем действия для последующего использования
    // _previous_actions = actions;

    // 5. Масштабируем выходные действия
    actions *= this->params.action_scale;

    // 6. Корректируем действия суставов
    for (int i : this->params.hip_scale_reduction_indices)
    {
        actions[0][i] *= this->params.hip_scale_reduction;
    }

    // 7. Вычисляем крутящие моменты и  позиции суставов
    //output_torques = this->ComputeTorques(actions);
    output_dof_pos = this->ComputePosition(actions);

    return output_dof_pos;

    // Execute the model and turn its output into a tensor.
    // std::cout << "mode " << mode << std::endl;
    // torch::Tensor output = module_dict[mode].forward(inputs).toTensor();
    // Save the output to use as previous_actions.
    //_previous_actions = output;
    //return output *  this->params.action_scale;

    

}

// void Agent::set_mode(const std_msgs::msg::String::SharedPtr& msg)
// {
//     mode = msg->data;
    
// }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MY_CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void Agent::InitObservations()
{
    this->obs.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    this->obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
    this->obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    this->obs.dof_pos = this->params.default_dof_pos;
    this->obs.dof_vel = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    this->obs.actions = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
}

void Agent::InitOutputs()
{
    output_torques = torch::tensor({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    output_dof_pos = params.default_dof_pos;
}

torch::Tensor Agent::ComputeTorques(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    torch::Tensor output_torques = this->params.rl_kp * (actions_scaled + this->params.default_dof_pos - this->obs.dof_pos) - this->params.rl_kd * this->obs.dof_vel;
    torch::Tensor clamped = torch::clamp(output_torques, -(this->params.torque_limits), this->params.torque_limits);
    return clamped;
}


torch::Tensor Agent::ComputePosition(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    return actions_scaled + this->params.default_dof_pos;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for(const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void Agent::ReadYaml(std::string robot_name)
{
    // The config file is located at "rl_sar/src/rl_sar/models/<robot_name>/config.yaml"
   // std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/config.yaml";
   std::string config_path = std::string("/home/ruben/Desktop/ros2_ws/src/unitree_rl_controller-ros2/weights/config.yaml");
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[robot_name];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    this->params.model_name = config["model_name"].as<std::string>();
    //this->params.framework = config["framework"].as<std::string>();
    this->params.num_observations = config["num_observations"].as<int>();
    this->params.clip_obs = config["clip_obs"].as<float>();
    this->params.clip_actions = config["clip_actions"].as<float>();
    this->params.action_scale = config["action_scale"].as<float>();
    this->params.hip_scale_reduction = config["hip_scale_reduction"].as<float>();
    this->params.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    this->params.num_of_dofs = config["num_of_dofs"].as<int>();
    this->params.lin_vel_scale = config["lin_vel_scale"].as<double>();
    this->params.ang_vel_scale = config["ang_vel_scale"].as<float>();
    this->params.dof_pos_scale = config["dof_pos_scale"].as<float>();
    this->params.dof_vel_scale = config["dof_vel_scale"].as<float>();
    // this->params.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    this->params.commands_scale = torch::tensor({this->params.lin_vel_scale, this->params.lin_vel_scale, this->params.ang_vel_scale});
    this->params.rl_kp = torch::tensor(ReadVectorFromYaml<float>(config["rl_kp"])).view({1, -1});
    this->params.rl_kd = torch::tensor(ReadVectorFromYaml<float>(config["rl_kd"])).view({1, -1});
    //this->params.fixed_kp = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kp"], this->params.framework, rows, cols)).view({1, -1});
    //this->params.fixed_kd = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kd"], this->params.framework, rows, cols)).view({1, -1});
    this->params.torque_limits = torch::tensor(ReadVectorFromYaml<float>(config["torque_limits"])).view({1, -1});
    this->params.default_dof_pos = torch::tensor(ReadVectorFromYaml<float>(config["default_dof_pos"])).view({1, -1});
    this->params.default_joint_angles = ReadVectorFromYaml<float>(config["default_joint_angles"]);
    this->params.joint_names = ReadVectorFromYaml<std::string>(config["joint_names"]);
}

torch::Tensor Agent::ComputeObservation()
{
    torch::Tensor obs = torch::cat({//(this->QuatRotateInverse(this->base_quat, this->lin_vel)) * this->params.lin_vel_scale,
                                    (this->quat_rotate_inverse(this->obs.base_quat, this->obs.ang_vel)) * this->params.ang_vel_scale,
                                    this->quat_rotate_inverse(this->obs.base_quat, this->obs.gravity_vec),
                                    //this->obs.commands * this->params.commands_scale,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions},
                                   1);

    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);

    printf("observation size: %lld, %lld\n", (long long)obs.sizes()[0], (long long)obs.sizes()[1]);


    return obs;
}

torch::Tensor Agent::Forward()
{
    torch::Tensor obs = this->ComputeObservation();

    torch::Tensor actor_input = torch::cat({obs}, 1);

    //torch::Tensor action = this->model.forward({actor_input}).toTensor();
    torch::Tensor action = this->module.forward({actor_input}).toTensor();

    this->obs.actions = action;
    torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions);

    return clamped;
}