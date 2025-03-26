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
    obs.dof_pos = torch::zeros({12});
    obs.dof_vel = torch::zeros({12});
    obs.base_linear_velocity = torch::zeros({3});
    obs.base_angular_velocity = torch::zeros({3});
    obs.base_linear_acceleration = torch::zeros({3});
    obs.orientation = torch::tensor({0.0, 0.0, 0.0, 1.0});
    obs.commands = torch::zeros({3});
    obs._gravity_vector = torch::tensor({0.0, 0.0, -1.0});
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


torch::Tensor Agent::get_observations()
{
    //obs.projected_gravity = quat_rotate_inverse(obs.orientation, obs._gravity_vector);
    torch::Tensor observations = torch::cat({
        obs.base_angular_velocity * params.ang_vel_scale,
        quat_rotate_inverse(obs.orientation, obs._gravity_vector),
        obs.dof_pos * params.dof_pos_scale,
        obs.dof_vel * params.dof_vel_scale,
        _previous_actions
    });

    //std::cout << "observations " << observations << std::endl;
    return observations;
}

torch::Tensor Agent::act()
{
    torch::Tensor observations = get_observations();
    std::cout << "OBS" << observations << std::endl;
    // std::cout << "GRAVITY: " << projected_gravity << std::endl;
    // Create a vector of inputs.
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(observations);
    // Execute the model and turn its output into a tensor.
   // std::cout << "mode " << mode << std::endl;
   //std::cout << "OBS" << inputs << std::endl;
    torch::Tensor output = module.forward(inputs).toTensor();
    // Save the output to use as previous_actions.
    _previous_actions = output;
    std::cout << "ACT" << output << std::endl;
    return output * params.action_scale;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    if (node.IsSequence()) { // Проверяем, что это список
        for(const auto& val : node)
        {
            values.push_back(val.as<T>());
        }
    } else {
        std::cout << "Expected a sequence (list) in YAML, but got something else." << std::endl;
        // Можно выбросить исключение, чтобы сообщить об ошибке выше
        // throw YAML::Exception(YAML::Mark(), "Expected a sequence (list).");
    }
    return values;
}

void Agent::ReadYaml(std::string &robot_name,std::string &config_path)
{

	YAML::Node config;
	try
	{
		config = YAML::LoadFile(config_path)[robot_name];
	} catch(YAML::BadFile &e)
	{

		std::cout << "The file '" << config_path << "' does not exist" << std::endl;
		return;
	}

    this->params.model_name = config["model_name"].as<std::string>();
    this->params.clip_obs = config["clip_obs"].as<float>();
    this->params.clip_actions = config["clip_actions"].as<float>();
    this->params.action_scale = config["action_scale"].as<float>();
    this->params.ang_vel_scale = config["ang_vel_scale"].as<float>();
    this->params.dof_pos_scale = config["dof_pos_scale"].as<float>();
    this->params.dof_vel_scale = config["dof_vel_scale"].as<float>();
    this->params.default_dof_pos = torch::tensor(ReadVectorFromYaml<float>(config["default_dof_pos"])).view({1, -1});
   // std::cout << "this->params.default_dof_pos.scalar_type(): " << this->params.default_dof_pos.scalar_type() << std::endl;
    this->params.joint_names = ReadVectorFromYaml<std::string>(config["joint_names"]);
}