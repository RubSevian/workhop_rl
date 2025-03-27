#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <memory>
#include <map>
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>


struct Observations
{
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor base_angular_velocity;
    torch::Tensor orientation;
    torch::Tensor gravity_vector;
    torch::Tensor action;
};

struct ModelParams
{
    std::string model_name;
    float action_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    torch::Tensor default_dof_pos;
    std::vector<std::string> joint_names;
};

class Agent
{
    private:
        torch::Tensor output_dof_pos = torch::zeros({12});
        torch::jit::script::Module module;
        //torch::Tensor get_observations();
        torch::Tensor ComputeObservation();

    public:
        ModelParams params;
        Observations obs;
        Agent();
        bool load_model(std::string model_path);
        torch::Tensor act();
        void ReadYaml(std::string &robot_name,std::string &config_path);
        void InitObservations();
        //void InitOutputs();
        torch::Tensor ComputePosition(torch::Tensor actions);
       // torch::Tensor ComputeObservation();
        torch::Tensor Forward();
};

