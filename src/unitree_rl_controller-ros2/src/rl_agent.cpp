#include <torch/torch.h>
#include "unitree_rl_controller/rl_agent.h"
#include <map>
#include "std_msgs/msg/string.hpp"

torch::Tensor Agent::Quat_rotate_inverse(torch::Tensor q, torch::Tensor v) {
    c10::IntArrayRef shape = q.sizes();
    torch::Tensor q_w = q.index({torch::indexing::Slice(), -1});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    torch::Tensor b = torch::cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    torch::Tensor c = q_vec * torch::bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

bool Agent::Load_model(std::string model_path)
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

torch::Tensor Agent::Act()
{
    torch::Tensor actions = this->Forward();

    output_dof_pos = this->ComputePosition(actions);

    return output_dof_pos;
  
}

void Agent::InitObservations()
{
    this->obs.ang_vel = torch::zeros({1, 3}, torch::kFloat);
    this->obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}}, torch::kFloat);
    this->obs.time = torch::tensor({{0.0}}, torch::kFloat);
    this->obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}}, torch::kFloat);// Явное указание типа
    this->obs.dof_pos = torch::zeros({1, 12}, torch::kFloat);
    this->obs.dof_vel = torch::zeros({1, 12}, torch::kFloat);
    this->obs.actions = torch::zeros({1, 12}, torch::kFloat);
}

void Agent::InitOutputs()
{
    output_dof_pos = torch::zeros({1, 12});
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

void Agent::ReadYaml(std::string robot_name,std::string config_path)
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

torch::Tensor Agent::ComputeObservation()
{
    torch::Tensor obs = torch::cat({this->obs.ang_vel * this->params.ang_vel_scale,
                                    this->Quat_rotate_inverse(this->obs.base_quat, this->obs.gravity_vec),
                                    this->obs.time,
                                    (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
                                    this->obs.dof_vel * this->params.dof_vel_scale,
                                    this->obs.actions},
                                   1);

    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);

    return obs;
}

torch::Tensor Agent::Forward()
{
    this->obs.time += 0.02;
    this->obs.time = torch::clip(this->obs.time, 0., 3.);
    
    torch::Tensor obs = this->ComputeObservation();

    std::cout << obs << std::endl;

    torch::Tensor actor_input = torch::cat({obs}, 1);

    torch::Tensor action = this->module.forward({actor_input}).toTensor();

    this->obs.actions = action;
    torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions);

    return clamped;
}