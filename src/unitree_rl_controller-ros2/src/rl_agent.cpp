#include <torch/torch.h>
#include "unitree_rl_controller/rl_agent.h"
#include <map>
#include "std_msgs/msg/string.hpp"


torch::Tensor quat_rotate_inverse(torch::Tensor q, torch::Tensor v) {
    torch::Tensor q_w = q.index({3});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(torch::indexing::None, 3)});
    torch::Tensor a = v * (2.0 * q_w * q_w - 1.0);
    torch::Tensor b = torch::cross(q_vec, v) * q_w * 2.0;
    torch::Tensor c = q_vec * torch::matmul(q_vec.view({1, 3}), v.view({3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

Agent::Agent()
{
    dof_pos = torch::zeros({12});
    dof_vel = torch::zeros({12});
    base_linear_velocity = torch::zeros({3});
    base_angular_velocity = torch::zeros({3});
    base_linear_acceleration = torch::zeros({3});
    orientation = torch::tensor({0.0, 0.0, 0.0, 1.0});
    commands = torch::zeros({3});
    mode = "trotting";
}


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

bool Agent::load_model_dict(std::map<std::string, std::string> model_paths)
{
    try {
        for (const auto& [k, v] : model_paths) {
            module_dict[k] = torch::jit::load(v);
        }
    }
    catch (const c10::Error& e) {
        return false;
    }
    return true;
}

torch::Tensor Agent::get_observations()
{
    projected_gravity = quat_rotate_inverse(orientation, _gravity_vector);
    torch::Tensor observations = torch::cat({
        (base_linear_acceleration + projected_gravity * 9.81) * scales.lin_vel,
        base_angular_velocity * scales.ang_vel,
        projected_gravity,
        commands * scales.commands,
        dof_pos * scales.dof_pos,
        dof_vel * scales.dof_vel,
        _previous_actions
    });

    std::cout << "observations " << observations << std::endl;
    return observations;
}

torch::Tensor Agent::act()
{
    torch::Tensor observations = get_observations();
    // std::cout << "GRAVITY: " << projected_gravity << std::endl;
    // Create a vector of inputs.
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(observations);
    // Execute the model and turn its output into a tensor.
    std::cout << "mode " << mode << std::endl;
    torch::Tensor output = module_dict[mode].forward(inputs).toTensor();
    // Save the output to use as previous_actions.
    _previous_actions = output;
    return output * scales.actions;
}

void Agent::set_mode(const std_msgs::msg::String::SharedPtr& msg)
{
    mode = msg->data;
    
}