//#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <string>
#include <memory>
#include <map>
#include "std_msgs/msg/string.hpp"

#include <yaml-cpp/yaml.h>


struct Observations
{         
    torch::Tensor ang_vel;      
    torch::Tensor gravity_vec;
    torch::Tensor time;
    torch::Tensor base_quat;   
    torch::Tensor dof_pos;           
    torch::Tensor dof_vel;           
    torch::Tensor actions;
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
        torch::jit::script::Module module;
    public:
        ModelParams params;
        Observations obs;

        bool Load_model(std::string model_path);
        torch::Tensor Act();
        void ReadYaml(std::string robot_name,std::string config_path);
        torch::Tensor Quat_rotate_inverse(torch::Tensor q, torch::Tensor v);
        void InitObservations();
        void InitOutputs();
        torch::Tensor ComputePosition(torch::Tensor actions);
        torch::Tensor ComputeObservation();
        torch::Tensor Forward();
        torch::Tensor output_dof_pos;
};

