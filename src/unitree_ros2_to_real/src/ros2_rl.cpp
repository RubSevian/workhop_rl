#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>

#include <torch/torch.h>
#include "unitree_rl_controller/rl_agent.h"


#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <mean_smoothing.h>
#include <map>

#define DIMENSION 3


using namespace UNITREE_LEGGED_SDK;

enum CONTROL_MODE
{
    CM_UNDEFINED,
    CM_POSTITION,
    CM_TORQUE
};

enum ROBOT_STATE
{
    STATE_INIT,
    STATE_READY,
    STATE_WAITING,
    STATE_FALLEN
};

Agent agent;

// const std::vector<std::string> joint_names = {
//     "FR_hip", "FR_thigh", "FR_calf",
//     "FL_hip", "FL_thigh", "FL_calf",
//     "RR_hip", "RR_thigh", "RR_calf",
//     "RL_hip", "RL_thigh", "RL_calf"};
const std::vector<std::string> joint_names = agent.params.joint_names;

const std::vector<float> default_joint_angles = agent.params.default_joint_angles;
    // -0., 0.8, -1.3,
    // 0., 0.8, -1.3,
    // -0.0, 0.8, -1.3,
    // 0.0, 0.8, -1.3};

const std::vector<int> net2joint_indexes = {
    3, 4, 5,
    0, 1, 2,
    9, 10, 11,
    6, 7, 8};
// Я бы убрал это 
const std::vector<double> stiffness = {
    20., 20., 20.,
    20., 20., 20.,
    20., 20., 20.,
    20., 20., 20.};

const std::vector<double> damping = {
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5};

//

const std::vector<std::string> urdf_feet_names = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};



std::string model_path = {"/home/ruben/workhop_rl/src/unitree_rl_controller-ros2/weights/policy_1.pt"}; // add my learn model

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void update_dof_state(const ros2_unitree_legged_msgs::msg::LowState &state, Agent &agent)
{
    for (int i = 0; i < 12; i++)
    {
        agent.dof_pos.index({net2joint_indexes[i]}) = state.motor_state[i].q - default_joint_angles[i];
        agent.dof_vel.index({net2joint_indexes[i]}) = state.motor_state[i].dq;
    }
}

void update_commands(const geometry_msgs::msg::Twist commands) // В новом коде это не будет использоваться 
    {
        agent.commands.index({0}) = commands.linear.x;
        agent.commands.index({1}) = commands.linear.y;
        agent.commands.index({2}) = commands.angular.z;
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    auto node = rclcpp::Node::make_shared("node_ros2_rl");

    int rate_value = 1000;
    int net_rate_value = 50;
    rclcpp::WallRate loop_rate(rate_value);

    int currentControlMode = CM_UNDEFINED;
    int robot_state = STATE_INIT;
    bool fallen = true;
    long fallen_pause_time;

    long motiontime = 0;
    int rate_count = 0;

    float qInit[12] = {0};
    float qDes[12] = {0};
    float Kp[12] = {0};
    float Kd[12] = {0};

    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;
    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    UDP state_udp(LOWLEVEL);
    LowCmd cmd = {0};
    LowState state = {0};
    state_udp.InitCmdData(cmd);
   
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_jointstates;
    pub_jointstates = node->create_publisher<sensor_msgs::msg::JointState>("/go1/joint_states", 1000);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("/go1/imu0", 1000);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_filter;
    pub_imu_filter = node->create_publisher<sensor_msgs::msg::Imu>("/go1/imu0_filter", 1000);

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_FL_force, pub_FR_force, pub_RL_force, pub_RR_force;
    pub_FR_force = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legFR/force_torque_states", 1000);
    pub_FL_force = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legFL/force_torque_states", 1000);
    pub_RR_force = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legRR/force_torque_states", 1000);
    pub_RL_force = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legRL/force_torque_states", 1000);

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_wireless_remote;
    pub_wireless_remote = node->create_publisher<std_msgs::msg::UInt8MultiArray>("/go1/remote", 100);

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1000);

    if (!agent.load_model(model_path))
        RCLCPP_ERROR(node->get_logger(), "Error loading the model\n");
    else
        RCLCPP_INFO(node->get_logger(), "Model loaded successfully\n");


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands_sub;
    commands_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(update_commands, std::placeholders::_1));

    bool initiated_flag = false; // initiate need time
    int count = 0;

    low_cmd_ros.level_flag = LOWLEVEL;

    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motor_cmd[i].q = PosStopF; // 禁止位置环
        low_cmd_ros.motor_cmd[i].kp = 0;
        low_cmd_ros.motor_cmd[i].dq = VelStopF; // 禁止速度环
        low_cmd_ros.motor_cmd[i].kd = 0;
        low_cmd_ros.motor_cmd[i].tau = 0;
    }

    mean_smoothing<double, DIMENSION> meansmth; 

    while (rclcpp::ok())
    {
        state_udp.Recv();
        state_udp.GetRecv(state);
        low_state_ros = state2rosMsg(state);

        if (initiated_flag == true)
        {
            motiontime ++;

            auto joint_state = sensor_msgs::msg::JointState();
            joint_state.header.stamp = node->get_clock()->now();
            joint_state.name.resize(joint_names.size());
            joint_state.position.resize(joint_names.size());
            joint_state.velocity.resize(joint_names.size());
            for (size_t i = 0; i < joint_names.size(); i++)
                {
                    size_t j = net2joint_indexes[i];
                    joint_state.name[j] = joint_names[j];
                    joint_state.position[j] = low_state_ros.motor_state[i].q; // Position here is a servo angle in rads
                    joint_state.velocity[j] = low_state_ros.motor_state[i].dq;
                }

            pub_jointstates->publish(joint_state);

            sensor_msgs::msg::Imu imu_state;
            imu_state.header.stamp = node->get_clock()->now();
            imu_state.header.frame_id = "imu_link";
            imu_state.orientation.w = low_state_ros.imu.quaternion[0];
            imu_state.orientation.x = low_state_ros.imu.quaternion[1];
            imu_state.orientation.y = low_state_ros.imu.quaternion[2];
            imu_state.orientation.z = low_state_ros.imu.quaternion[3];
            imu_state.linear_acceleration.x = low_state_ros.imu.accelerometer[0];
            imu_state.linear_acceleration.y = low_state_ros.imu.accelerometer[1];
            imu_state.linear_acceleration.z = low_state_ros.imu.accelerometer[2];
            imu_state.angular_velocity.x = low_state_ros.imu.gyroscope[0];
            imu_state.angular_velocity.y = low_state_ros.imu.gyroscope[1];
            imu_state.angular_velocity.z = low_state_ros.imu.gyroscope[2];
            pub_imu->publish(imu_state);

            es_vec<double, DIMENSION> curr_query;
            for (size_t i = 0; i < DIMENSION; ++i) curr_query[i] = low_state_ros.imu.accelerometer[i];
            es_vec<double, DIMENSION> acc_filter = meansmth.push_to_pop(curr_query);

            sensor_msgs::msg::Imu imu_state_filter;
            imu_state_filter.header.stamp = node->get_clock()->now();
            imu_state_filter.header.frame_id = "imu_link";
            imu_state_filter.orientation.w = low_state_ros.imu.quaternion[0];
            imu_state_filter.orientation.x = low_state_ros.imu.quaternion[1];
            imu_state_filter.orientation.y = low_state_ros.imu.quaternion[2];
            imu_state_filter.orientation.z = low_state_ros.imu.quaternion[3];
            imu_state_filter.linear_acceleration.x = acc_filter[0];
            imu_state_filter.linear_acceleration.y = acc_filter[1];
            imu_state_filter.linear_acceleration.z = acc_filter[2];
            imu_state_filter.angular_velocity.x = low_state_ros.imu.gyroscope[0];
            imu_state_filter.angular_velocity.y = low_state_ros.imu.gyroscope[1];
            imu_state_filter.angular_velocity.z = low_state_ros.imu.gyroscope[2];
            pub_imu_filter->publish(imu_state_filter);

            agent.lin_vel.index({0}) = acc_filter[0];
            agent.lin_vel.index({1}) = acc_filter[1];
            agent.lin_vel.index({2}) = acc_filter[2];

            agent.ang_vel.index({0}) = low_state_ros.imu.gyroscope[0];
            agent.ang_vel.index({1}) = low_state_ros.imu.gyroscope[1];
            agent.ang_vel.index({2}) = low_state_ros.imu.gyroscope[2];
            agent.base_quat.index({0}) = low_state_ros.imu.quaternion[1];
            agent.base_quat.index({1}) = low_state_ros.imu.quaternion[2];
            agent.base_quat.index({2}) = low_state_ros.imu.quaternion[3];
            agent.base_quat.index({3}) = low_state_ros.imu.quaternion[0];

            std::vector<geometry_msgs::msg::WrenchStamped> feet_forces;
            for (size_t k = 0; k < 4; k++)
            {
                geometry_msgs::msg::WrenchStamped msg;
                msg.header.frame_id = urdf_feet_names[k];
                msg.header.stamp = node->get_clock()->now();
                msg.wrench.force.x = 0;
                msg.wrench.force.y = 0;
                msg.wrench.force.z = low_state_ros.foot_force[k] * 0.1; // Unitree foot pressure units is unknown. x0.1 makes it looks like newtons
                msg.wrench.torque.x = 0;
                msg.wrench.torque.y = 0;
                msg.wrench.torque.z = 0;
                feet_forces.push_back(msg);
            }

            pub_FR_force->publish(feet_forces[0]);
            pub_FL_force->publish(feet_forces[1]);
            pub_RR_force->publish(feet_forces[2]);
            pub_RL_force->publish(feet_forces[3]);

            std_msgs::msg::UInt8MultiArray remote_array;
            remote_array.data.clear();
            for (size_t k = 0; k < low_state_ros.wireless_remote.size(); k++)
            {
                remote_array.data.push_back(low_state_ros.wireless_remote[k]);
            }
            pub_wireless_remote->publish(remote_array);

            update_dof_state(low_state_ros, agent);

            if (motiontime >= 0)
            {
                // Get record initial position
                if (motiontime >= 0 && motiontime < 10)
                {
                    for (size_t k = 0; k < 12; k++)
                    {
                        qInit[k] = low_state_ros.motor_state[k].q;
                    }
                }

                // Move to the origin point with soft Kp/Kd
                if (motiontime >= 1 && motiontime < 1000)
                {
                    rate_count++;
                    double rate = rate_count / (1000.0 - 1.0);

                    for (size_t k = 0; k < 12; k++)
                    {
                        Kp[k] = 50.0;
                        Kd[k] = 2.0;
                    }

                    for (size_t k = 0; k < 12; k++)
                    {
                        qDes[k] = jointLinearInterpolation(qInit[k], default_joint_angles[k], rate);
                    }
                }

                if (motiontime == 1000)
                {
                    for (size_t k = 0; k < 12; k++)
                    {
                        Kp[k] = stiffness[k];
                        Kd[k] = damping[k];
                    }
                    robot_state = STATE_READY;
                }

                torch::Tensor actions;
                if (motiontime > 3000)
                {
                    if (motiontime % (rate_value / net_rate_value) == 0)
                    {
                        auto actions = agent.act();
                        if (agent.gravity_vec.index({2}).item().to<double>() >= -0.7)
                            robot_state = STATE_FALLEN;
                        else if (robot_state == STATE_FALLEN)
                            robot_state = STATE_WAITING;
                        fallen_pause_time = motiontime;

                        if (robot_state == STATE_WAITING && (motiontime - fallen_pause_time > 1000))
                            robot_state = STATE_READY;

                        if (robot_state == STATE_FALLEN)
                            actions = torch::tensor({0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     0.,
                                                     -0.});

                        std::cout << "actions " << actions << std::endl;
                    
                        for (size_t k = 0; k < 12; k++)
                        {
                            qDes[k] = default_joint_angles[k] + actions.index({net2joint_indexes[k]}).item().to<double>(); // Подправить под output_dof_pose
                            // qDes[k] = default_joint_angles[k];
                      
                        }
                    }
                }
                if (currentControlMode != CM_POSTITION)
                {
                    currentControlMode = CM_POSTITION;
                    std::cout << "ros2real switching to POSITION control" << std::endl;
                }
                for (size_t k = 0; k < 12; k++)
                {

                    low_cmd_ros.motor_cmd[k].q = qDes[k];
                    low_cmd_ros.motor_cmd[k].dq = 0; // dqDes[k]; // Seems that Unitree doesn't need the velocity value here in position control mode
                    low_cmd_ros.motor_cmd[k].kp = Kp[k];
                    low_cmd_ros.motor_cmd[k].kd = Kd[k];
                    low_cmd_ros.motor_cmd[k].tau = 0.0f;
                }
            }
        }
        cmd = rosMsg2Cmd(low_cmd_ros);
        state_udp.SetSend(cmd);
        state_udp.Send();
        rclcpp::spin_some(node);
        loop_rate.sleep();

        count++;
        if (count > 10)
        {
            count = 10;
            initiated_flag = true;
        }
    }

    return 0;
}

