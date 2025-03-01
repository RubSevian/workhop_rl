#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <string>
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

// struct Scales
// {
//     double lin_vel = 1.0;
//     double ang_vel = 0.25;
//     torch::Tensor commands = torch::tensor({lin_vel, lin_vel, ang_vel});
//     double actions = 0.25;
//     double dof_pos = 1.0;
//     double dof_vel = 0.05;
// };

struct Observations
{
    torch::Tensor lin_vel;           
    torch::Tensor ang_vel;      
    torch::Tensor gravity_vec;
    torch::Tensor commands;          
    torch::Tensor base_quat;   
    torch::Tensor dof_pos;           
    torch::Tensor dof_vel;           
    torch::Tensor actions;
};

struct ModelParams
{
    std::string model_name;
    int num_observations;
    float hip_scale_reduction;
    std::vector<int> hip_scale_reduction_indices;
    int num_of_dofs;
    float action_scale;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    std::vector<float> default_joint_angles;
    torch::Tensor commands_scale;
    torch::Tensor rl_kp;
    torch::Tensor rl_kd;
    torch::Tensor torque_limits;
    torch::Tensor default_dof_pos;
    //std::vector<std::string> default_dof_pos;
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
        //Agent();
        bool load_model(std::string model_path);
        bool load_model_dict(std::map<std::string, std::string> model_paths);
        torch::Tensor act();
        //void set_scales();
        std::string mode; 
        //std::map<std::string, Actuator> joints;
        // torch::Tensor dof_pos;
        // torch::Tensor dof_vel;
        // torch::Tensor base_linear_velocity;
        // torch::Tensor base_angular_velocity;
        // torch::Tensor base_linear_acceleration;
        // torch::Tensor orientation;
        //torch::Tensor commands;

        //torch::Tensor projected_gravity = _gravity_vector;
        // void set_mode(const std_msgs::msg::String::SharedPtr& msg);
        //~~~~~~~~~~~MY_CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        // yaml params

        // rl module
        torch::jit::script::Module model;
        // observation buffer
        torch::Tensor lin_vel;           
        torch::Tensor ang_vel;      
        torch::Tensor gravity_vec;      
        torch::Tensor commands;        
        torch::Tensor base_quat;   
        torch::Tensor dof_pos;           
        torch::Tensor dof_vel;           
        torch::Tensor actions;
        // output buffer
        torch::Tensor output_torques;
        torch::Tensor output_dof_pos;


        void ReadYaml(std::string robot_name);
        torch::Tensor quat_rotate_inverse(torch::Tensor q, torch::Tensor v);
        void InitObservations();
        void InitOutputs();
        torch::Tensor ComputeTorques(torch::Tensor actions);
        torch::Tensor ComputePosition(torch::Tensor actions);
        torch::Tensor ComputeObservation();
        torch::Tensor Forward();
};

