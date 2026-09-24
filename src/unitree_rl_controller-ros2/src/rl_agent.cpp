#include <torch/torch.h>
#include "rl_agent.h"
#include <map>
#include <cmath>
#include <stdexcept>
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
    obs.lin_vel = torch::zeros({12});
    obs.dof_pos = torch::zeros({12});
    obs.dof_vel = torch::zeros({12});
    obs.ang_vel = torch::zeros({3});
    obs.command = torch::zeros({3});
    obs.base_quat = torch::tensor({0.0, 0.0, 0.0, 1.0});
    obs.gravity_vec = torch::tensor({0.0, 0.0, -1.0});
    obs.action = torch::zeros({12});
    obs.sin=torch::zeros({1});
    obs.cos=torch::zeros({1});
    obs.height_map = torch::zeros({187});
    obs.arm_pos = torch::zeros({go2_rars01::Dimensions::ArmPos});
    obs.arm_vel = torch::zeros({go2_rars01::Dimensions::ArmVel});
    obs.arm_target = torch::zeros({go2_rars01::Dimensions::ArmTarget});
}
bool Agent::UsesGo2Rars01UnifiedLayout() const
{
    return params.observation_layout == "go2_rars01_unified_v1";
}

go2_rars01::UnifiedObservationState Agent::UnifiedState() const
{
    return {obs.ang_vel, obs.command, obs.base_quat, obs.dof_pos, obs.dof_vel,
            obs.action, obs.arm_pos, obs.arm_vel, obs.arm_target};
}

torch::Tensor Agent::ComputeGo2Rars01ActorFrame()
{
    return unified_observation_.BuildFrame(UnifiedState());
}

void Agent::ResetPolicyState()
{
    if (!UsesGo2Rars01UnifiedLayout()) return;

    // The [33:45] frame slice is the previous policy action. Training clears
    // both action tensors at reset, so a re-entry into RL must not carry an
    // output from the previous control session into f0.
    obs.action = torch::zeros({go2_rars01::Dimensions::PreviousAction},
                              obs.action.options());
    unified_observation_.Reset(ComputeGo2Rars01ActorFrame());
}

torch::Tensor Agent::UnifiedActorHistory() const
{
    return unified_observation_.History();
}

void Agent::InitRL()
{
    if (UsesGo2Rars01UnifiedLayout()) {
        ResetPolicyState();
        return;
    }

    if (this->params.observations_history.empty())
        return;

    // 1) compute obs once to fill obs_dims [file:28]
    torch::Tensor clamped_obs = this->ComputeObservation();

    // 2) history length [file:28]
    int history_length =
        *std::max_element(this->params.observations_history.begin(),
                          this->params.observations_history.end()) + 1;

    // 3) create buffer [file:22]
    this->history_obs_buf = ObservationBuffer(
        1, this->obs_dims, history_length, this->params.observations_history_priority
    );

    // 4) fill history with current obs so first steps are not zeros [file:22]
    this->history_obs_buf.reset({0}, clamped_obs.view({1, -1}));

}
void Agent::UpdatePhase(float time) {
    if (UsesGo2Rars01UnifiedLayout()) return;
    float phase = time / params.cycle_time; // Фаза в [0, 1] и далее (не ограничена)
    float angle = 2 * M_PI * phase;
    obs.sin.index({0}) = std::sin(angle);
    obs.cos.index({0}) = std::cos(angle);
}

bool Agent::Load_Model(const std::string &model_path)
{
    if (!std::filesystem::exists(model_path)) {
        std::cerr << "Error: Model file in path not found: " << model_path << std::endl;
        return false;
    }

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

    output_dof_pos = this->ComputePosition(this->obs.action);
    //std::cout << "Action_output before clamp: " << output_dof_pos << std::endl;
    torch::Tensor clamped_output = torch::clamp(output_dof_pos, -3.5, 3.5); // Ручной клиппинг [-3, 3]
    //std::cout << "Action_output after clamp: " << clamped_output << std::endl;
    return clamped_output;
}

torch::Tensor Agent::ComputePosition(torch::Tensor &actions)
{
    torch::Tensor actions_scaled = actions * this->params.action_scale;
    return actions_scaled + this->params.default_dof_pos;
}

torch::Tensor Agent::ComputeObservation()
{
    if (UsesGo2Rars01UnifiedLayout()) {
        return ComputeGo2Rars01ActorFrame();
    }
    std::vector<torch::Tensor> obs_model_list;

    for (const std::string &obs_name : this->params.obs_model){
        if (obs_name == "lin_vel")
        {
            obs_model_list.push_back(this->obs.lin_vel * this->params.lin_vel_scale);
        }
        else if (obs_name == "ang_vel")
        {
            obs_model_list.push_back(this->obs.ang_vel * this->params.ang_vel_scale);
        }
        else if (obs_name == "gravity_vec")
        {
            obs_model_list.push_back(this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (obs_name == "command")
        {
            obs_model_list.push_back(this->obs.command * this->params.command_scale);
        }
        else if (obs_name == "dof_pos")
        {
            obs_model_list.push_back((this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale);
        }
        else if (obs_name == "dof_vel")
        {
            obs_model_list.push_back(this->obs.dof_vel * this->params.dof_vel_scale);
        }
        else if (obs_name == "action")
        {
            obs_model_list.push_back(this->obs.action);
        }
        else if (obs_name == "sin")
        {
            obs_model_list.push_back((this->obs.sin));
        }
        else if (obs_name == "cos")
        {
            obs_model_list.push_back((this->obs.cos));
        }
        else if (obs_name == "height_map")
        {
            obs_model_list.push_back(this->obs.height_map);
        }

    }

    this->obs_dims.clear();
    for (const auto& t : obs_model_list) {
        this->obs_dims.push_back((int)t.numel());
    }

    // keep your original 1D layout 
    torch::Tensor obs = torch::cat(obs_model_list, 0);
    obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return obs; // [N]
    // torch::Tensor obs = torch::cat({obs_model_list},0);       
    // obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    // return obs;
}

torch::Tensor Agent::Forward()
{
    if (UsesGo2Rars01UnifiedLayout()) {
        const auto frame = ComputeGo2Rars01ActorFrame();
        if (!unified_observation_.initialized()) {
            unified_observation_.Reset(frame);
        } else {
            unified_observation_.Insert(frame);
        }
        const auto actor_input = unified_observation_.History();
        if (actor_input.dim() != 2 || actor_input.size(0) != 1 ||
            actor_input.size(1) != go2_rars01::Dimensions::ActorInput) {
            throw std::runtime_error("Unified actor input must have shape [1, 315]");
        }
        auto actions = module.forward({actor_input}).toTensor().reshape({-1});
        if (actions.numel() != go2_rars01::Dimensions::ActorOutput ||
            !torch::isfinite(actions).all().item<bool>()) {
            throw std::runtime_error("Unified policy must return 12 finite leg actions");
        }
        return torch::clamp(actions, -this->params.clip_actions, this->params.clip_actions);
    }


    torch::Tensor obs = this->ComputeObservation(); // [N]

    torch::Tensor actions;
    if (!this->params.observations_history.empty())
    {
        this->history_obs_buf.insert(obs.view({1, -1}));
        torch::Tensor hist = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->module.forward({hist}).toTensor();
    }
    else
    {
        actions = this->module.forward({obs}).toTensor();
    }

    actions = actions.squeeze(0);
    return torch::clamp(actions, -this->params.clip_actions, this->params.clip_actions);
    // torch::Tensor obs = this->ComputeObservation();
    // std::cout<<"Obs"<<obs<<std::endl;
    // torch::Tensor action = this->module.forward({obs}).toTensor();
    // std::cout<<"Action"<<action<<std::endl;
    // torch::Tensor clamped = torch::clamp(action, -this->params.clip_actions, this->params.clip_actions); 

    // return clamped;
}

torch::Tensor Agent::ComputeTorque(const torch::Tensor &actions_scaled) {
    torch::Tensor target_pos = actions_scaled * this->params.action_scale + this->params.default_dof_pos;
    torch::Tensor output_dof_tau = this->params.rl_kp * (target_pos - this->obs.dof_pos) - this->params.rl_kd * this->obs.dof_vel;
    //std::cout << "Agent::ComputeTorque before clamp: " << output_dof_tau << std::endl;
    output_dof_tau = torch::clamp(output_dof_tau, -this->params.torque_limits, this->params.torque_limits);
    return output_dof_tau;
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
    if(!std::filesystem::exists(config_path))
    {
        std::cout<< "Error: Config file" << config_path << std::endl;
        return;
    }

	YAML::Node config;

	try
	{
		auto document = YAML::LoadFile(config_path);
        config = document[robot_name];
        // RobotController keeps its legacy robot-name argument. A supplied
        // unified config is self-describing and remains usable via
        // initializeRL(config_path, model_path).
        if (!config && document["go2_rars01"])
            config = document["go2_rars01"];
	} catch(YAML::BadFile &e)
	{

		std::cout << "The file '" << config_path << "' does not exist" << std::endl;
		return;
	}
    // history params (optional)
    if (config["observations_history"] && config["observations_history"].IsSequence())
        this->params.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    else
        this->params.observations_history.clear();

    if (config["observations_history_priority"])
        this->params.observations_history_priority = config["observations_history_priority"].as<std::string>();
    else
        this->params.observations_history_priority = "time";

    this->params.model_name = config["model_name"] ? config["model_name"].as<std::string>() : "";
    this->params.observation_layout = config["observation_layout"] ?
        config["observation_layout"].as<std::string>() : "legacy";
    this->params.clip_obs = config["clip_obs"].as<float>();
    this->params.clip_actions = config["clip_actions"].as<float>();
    this->params.action_scale = config["action_scale"].as<float>();
    this->params.rl_kp =  torch::tensor(ReadVectorFromYaml<float>(config["rl_kp"]));
    this->params.rl_kd =  torch::tensor(ReadVectorFromYaml<float>(config["rl_kd"]));
    this->params.fixed_kp =  torch::tensor(ReadVectorFromYaml<float>(config["fixed_kp"]));
    this->params.fixed_kd =  torch::tensor(ReadVectorFromYaml<float>(config["fixed_kd"]));
    this->params.torque_limits = torch::tensor(ReadVectorFromYaml<float>(
        config["torque_limits"] ? config["torque_limits"] : config["ftorque_limits"]));
    this->params.ang_vel_scale = config["ang_vel_scale"].as<float>();
    this->params.dof_pos_scale = config["dof_pos_scale"].as<float>();
    this->params.dof_vel_scale = config["dof_vel_scale"].as<float>();
    this->params.decimation = config["decimation"].as<int>();
    this->params.frequency = config["frequency"] ? config["frequency"].as<float>() : 0.0F;
    this->params.cycle_time = config["cycle_time"].as<float>();
    this->params.lin_vel_scale = config["lin_vel_scale"].as<float>();
    this->params.obs_model = ReadVectorFromYaml<std::string>(config["observations"]);
    this->params.command_scale = torch::tensor(ReadVectorFromYaml<float>(config["commands_scale"]));
    this->params.default_dof_pos = torch::tensor(ReadVectorFromYaml<float>(config["default_dof_pos"]));
    this->params.joint_names = ReadVectorFromYaml<std::string>(config["joint_names"]);
    if (config["arm_joint_names"])
        this->params.arm_joint_names = ReadVectorFromYaml<std::string>(config["arm_joint_names"]);
    else
        this->params.arm_joint_names.clear();

    const auto require_12 = [](const torch::Tensor& values, const char* name) {
        if (values.numel() != 12) {
            throw std::runtime_error(std::string("Expected 12 values for '") + name +
                                     "', got " + std::to_string(values.numel()));
        }
    };
    require_12(this->params.rl_kp, "rl_kp");
    require_12(this->params.rl_kd, "rl_kd");
    require_12(this->params.fixed_kp, "fixed_kp");
    require_12(this->params.fixed_kd, "fixed_kd");
    require_12(this->params.torque_limits, "torque_limits");
    require_12(this->params.default_dof_pos, "default_dof_pos");

    if (UsesGo2Rars01UnifiedLayout()) {
        if (this->params.observations_history != std::vector<int>({4, 3, 2, 1, 0}) ||
            this->params.observations_history_priority != "time") {
            throw std::runtime_error("go2_rars01_unified_v1 requires chronological history [4,3,2,1,0] with time priority");
        }
        go2_rars01::UnifiedObservationScales scales;
        scales.ang_vel = this->params.ang_vel_scale;
        scales.dof_pos = this->params.dof_pos_scale;
        scales.dof_vel = this->params.dof_vel_scale;
        scales.clip_observations = this->params.clip_obs;
        const auto same = [](float actual, float expected) {
            return std::abs(actual - expected) <= 1.0e-6F;
        };
        if (this->params.command_scale.numel() != go2_rars01::Dimensions::Command ||
            this->params.decimation != 4 || !same(this->params.frequency, 50.0F) ||
            !same(this->params.action_scale, 0.25F) || !same(this->params.ang_vel_scale, 0.25F) ||
            !same(this->params.dof_pos_scale, 1.0F) || !same(this->params.dof_vel_scale, 0.05F) ||
            !same(this->params.clip_obs, 100.0F) || !same(this->params.clip_actions, 100.0F)) {
            throw std::runtime_error("go2_rars01_unified_v1 has an incompatible scale, clip, decimation, or frequency");
        }
        for (int i = 0; i < go2_rars01::Dimensions::Command; ++i) {
            scales.command[i] = this->params.command_scale.index({i}).item<float>();
        }
        if (!same(scales.command[0], 2.0F) || !same(scales.command[1], 2.0F) ||
            !same(scales.command[2], 0.25F)) {
            throw std::runtime_error("go2_rars01_unified_v1 requires command scales [2, 2, 0.25]");
        }
        if (this->params.joint_names.size() != go2_rars01::kLegJointNames.size()) {
            throw std::runtime_error("go2_rars01_unified_v1 requires 12 ordered leg joint names");
        }
        for (std::size_t i = 0; i < this->params.joint_names.size(); ++i) {
            if (this->params.joint_names[i] != go2_rars01::kLegJointNames[i]) {
                throw std::runtime_error("go2_rars01_unified_v1 leg joint order differs from policy order");
            }
        }
        if (this->params.arm_joint_names.size() != go2_rars01::kArmJointNames.size()) {
            throw std::runtime_error("go2_rars01_unified_v1 requires 6 ordered arm joint names");
        }
        for (std::size_t i = 0; i < this->params.arm_joint_names.size(); ++i) {
            if (this->params.arm_joint_names[i] != go2_rars01::kArmJointNames[i]) {
                throw std::runtime_error("go2_rars01_unified_v1 arm joint order differs from policy order");
            }
        }
        unified_observation_.Configure(this->params.default_dof_pos, scales);
    }
}
