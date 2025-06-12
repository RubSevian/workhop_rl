#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <memory>
#include <map>
#include <filesystem>
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>


struct Observations
{
    torch::Tensor ang_vel;
    torch::Tensor lin_vel;
    torch::Tensor command;
    torch::Tensor gravity_vec;
    torch::Tensor base_quat;
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor action;
};

struct ModelParams
{
    std::string model_name;
    float action_scale;
    float lin_vel_scale;
    int decimation;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    torch::Tensor default_dof_pos;
    torch::Tensor command_scale;
    std::vector<std::string> obs_model;
    std::vector<std::string> joint_names;
};

class Agent
{
    private:
        torch::Tensor output_dof_pos = torch::zeros({12});
        torch::jit::script::Module module;
        torch::Tensor ComputeObservation();
        void InitObservations();
        torch::Tensor ComputePosition(torch::Tensor &actions);
        torch::Tensor Forward();
        torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v);

    public:
        ModelParams params;
        Observations obs;
        Agent();
        bool Load_Model(const std::string &model_path);
        torch::Tensor Act();
        void ReadYaml(const std::string &robot_name,const std::string &config_path);

};

