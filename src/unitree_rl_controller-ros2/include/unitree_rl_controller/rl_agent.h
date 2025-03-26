#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <memory>
#include <map>
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>

// struct Actuator
// {
//     std::string name;
//     int index;
//     int net_index;
//     double position;
//     double velocity;
//     double stiffness;
//     double damping;
// };

struct Observations
{
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor base_linear_velocity;
    torch::Tensor base_angular_velocity;
    torch::Tensor base_linear_acceleration;
    torch::Tensor orientation;
    torch::Tensor commands;
    torch::Tensor _gravity_vector; 
};

// struct Scales
// {
//     double lin_vel = 1.0;
//     double ang_vel = 0.25;
//     torch::Tensor commands = torch::tensor({lin_vel, lin_vel, ang_vel});
//     double actions = 0.25;
//     double dof_pos = 1.0;
//     double dof_vel = 0.05;
// };

// struct ModelParams
// {
//     std::string model_name;
//     float action_scale;
//     float ang_vel_scale;
//     float dof_pos_scale;
//     float dof_vel_scale;
//     float clip_obs;
//     float clip_actions;
//     torch::Tensor default_dof_pos;
//     std::vector<std::string> joint_names;
// };
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
        torch::Tensor _previous_actions = torch::zeros({12});
        torch::jit::script::Module module;
        std::map<std::string, torch::jit::script::Module> module_dict;
        torch::Tensor get_observations();
        //Scales scales = Scales();

        //torch::Tensor _gravity_vector = torch::tensor({0.0, 0.0, -1.0});
    public:
        ModelParams params;
        Observations obs;
        Agent();
        bool load_model(std::string model_path);
        torch::Tensor act();
        void ReadYaml(std::string &robot_name,std::string &config_path);
        //void set_scales();
        //std::string mode; 
        //std::map<std::string, Actuator> joints;
        //void set_mode(const std_msgs::msg::String::SharedPtr& msg);
        
};

