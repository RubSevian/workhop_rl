#include <torch/torch.h>
#include "rl_agent.h"
#include <map>
#include "std_msgs/msg/string.hpp"


torch::Tensor Agent::QuatRotateInverse(torch::Tensor q, torch::Tensor v) {
    torch::Tensor q_w = q.index({3});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(torch::indexing::None, 3)});
    torch::Tensor a = v * (2.0 * q_w * q_w - 1.0);
    torch::Tensor b = torch::cross(q_vec, v) * q_w * 2.0;
    torch::Tensor c = q_vec * torch::matmul(q_vec.view({1, 3}), v.view({3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

Agent::Agent()
{
    InitObservations();
}   

void Agent::InitObservations()
{
    obs.dof_pos = torch::zeros({12});
    obs.dof_vel = torch::zeros({12});
    obs.ang_vel = torch::zeros({3});
    obs.base_quat = torch::tensor({0.0, 0.0, 0.0, 1.0});
    obs.gravity_vec = torch::tensor({0.0, 0.0, -1.0});
    obs.action = torch::zeros({12});
}

bool Agent::Load_Model(const std::string &model_path)
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

    this->obs.action = this->Forward();

    output_dof_pos = this->ComputePosition(obs.action);

    return output_dof_pos;
}

torch::Tensor Agent::ComputePosition(torch::Tensor &actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    return actions_scaled + this->params.default_dof_pos;
}

torch::Tensor Agent::ComputeObservation()
{
    torch::Tensor obs = torch::cat({
        this->obs.ang_vel * this->params.ang_vel_scale,
        QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
        (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
        this->obs.dof_vel * this->params.dof_vel_scale,
        this->obs.action
    });       
    
    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return obs;
}

torch::Tensor Agent::Forward()
{
    torch::Tensor obs = this->ComputeObservation();
    std::cout<<"Obs"<<obs<<std::endl;
    torch::Tensor action = this->module.forward({obs}).toTensor();
    std::cout<<"Action"<<action<<std::endl;
    torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions); 

    return clamped;
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

void Agent::ReadYaml(const std::string &robot_name,const std::string &config_path)
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
    this->params.default_dof_pos = torch::tensor(ReadVectorFromYaml<float>(config["default_dof_pos"]));
    this->params.joint_names = ReadVectorFromYaml<std::string>(config["joint_names"]);
}