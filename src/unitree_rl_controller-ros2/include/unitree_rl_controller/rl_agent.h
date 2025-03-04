//#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <string>
#include <memory>
#include <map>
#include "std_msgs/msg/string.hpp"

#include <yaml-cpp/yaml.h>
#define CONFIG_PATH  "/home/ruben/workhop_rl/src/unitree_rl_controller-ros2/weights/config.yaml"


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
    //torch::Tensor lin_vel;           
    torch::Tensor ang_vel;      
    torch::Tensor gravity_vec;
    //torch::Tensor commands;          
    torch::Tensor base_quat;   
    torch::Tensor dof_pos;           
    torch::Tensor dof_vel;           
    torch::Tensor actions;
};

struct ModelParams
{
    std::string model_name;
    int num_observations;
    double hip_scale_reduction;
    std::vector<int> hip_scale_reduction_indices;
    int num_of_dofs;
    double action_scale;
    //double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double clip_obs;
    double clip_actions;
    std::vector<double> default_joint_angles;
    //torch::Tensor commands_scale;
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
        torch::jit::script::Module module;
    public:
        ModelParams params;
        Observations obs;

        torch::Tensor get_observations();

        //Agent();
        bool load_model(std::string model_path);
        torch::Tensor act();
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

