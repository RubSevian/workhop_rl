#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>

#include <torch/torch.h>
#include "rl_agent.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>



#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <map>


// using namespace unitree::common;
// using namespace unitree::robot;

// #define TOPIC_LOWCMD "rt/lowcmd"
// #define TOPIC_LOWSTATE "rt/lowstate"

// constexpr double PosStopF = (2.146E+9f);
// constexpr double VelStopF = (16000.0f);




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
    STATE_READY
};

const std::vector<int> net2joint_indexes = {
    3, 4, 5,
    0, 1, 2,
    9, 10, 11,
    6, 7, 8};

const std::vector<float> stiffness = {
    20., 20., 20.,
    20., 20., 20.,
    20., 20., 20.,
    20., 20., 20.};

const std::vector<float> damping = {
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5};


float jointLinearInterpolation(float initPos, float targetPos, float rate)
{
    float p;
    rate = std::min(std::max(rate, 0.0f), 1.0f);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void update_dof_state(const ros2_unitree_legged_msgs::msg::LowState &state, Agent &agent)
{
    for (int i = 0; i < 12; i++)
    {
        agent.obs.dof_pos.index({net2joint_indexes[i]}) = state.motor_state[i].q ;
        agent.obs.dof_vel.index({net2joint_indexes[i]}) = state.motor_state[i].dq;
    }
}


int main(int argc, char **argv)
{
    Agent agent;
    const std::string ROBOT_NAME = "go1";

    const std::string CONFIG_PATH = std::string(CONFIG_BASE_DIR) + "/weights/" + ROBOT_NAME + "/config.yaml";
    try {
        agent.ReadYaml(ROBOT_NAME,CONFIG_PATH);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::vector<std::string> joint_names;

    const std::string model_path =std::string(CONFIG_BASE_DIR) + "/weights/" + ROBOT_NAME + "/" + std::string(agent.params.model_name); 

    joint_names = agent.params.joint_names;
    
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    auto node = rclcpp::Node::make_shared("node_ros2_rl");

    int rate_value = 1000;
    int net_rate_value = 50;
    int control_period = rate_value / net_rate_value;
    rclcpp::WallRate loop_rate(rate_value);

    int currentControlMode = CM_UNDEFINED;
    int robot_state = STATE_INIT;
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
   
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("/go1/imu0", 1000);

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1000);

    if (!agent.Load_Model(model_path))
        RCLCPP_ERROR(node->get_logger(), "Error loading the model\n");
    else
        RCLCPP_INFO(node->get_logger(), "Model loaded successfully\n");

    bool initiated_flag = false; // initiate need time
    int count = 0;

    low_cmd_ros.level_flag = LOWLEVEL;

    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motor_cmd[i].q = PosStopF; // Forbidden position
        low_cmd_ros.motor_cmd[i].kp = 0;
        low_cmd_ros.motor_cmd[i].dq = VelStopF; // Forbidden speed 
        low_cmd_ros.motor_cmd[i].kd = 0;
        low_cmd_ros.motor_cmd[i].tau = 0;
    }

    // Switch to POSITION control mode only once at the beginning
    
    if (currentControlMode != CM_POSTITION) {
        currentControlMode = CM_POSTITION;
        std::cout << "ros2real switching to POSITION control" << std::endl;
    }

    while (rclcpp::ok())
    {
        state_udp.Recv();
        state_udp.GetRecv(state);
        low_state_ros = state2rosMsg(state);

        if (initiated_flag == true)
        {
            motiontime ++;

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

            agent.obs.ang_vel.index({0}) = low_state_ros.imu.gyroscope[0];
            agent.obs.ang_vel.index({1}) = low_state_ros.imu.gyroscope[1];
            agent.obs.ang_vel.index({2}) = low_state_ros.imu.gyroscope[2];
            agent.obs.base_quat.index({0}) = low_state_ros.imu.quaternion[1];
            agent.obs.base_quat.index({1}) = low_state_ros.imu.quaternion[2];
            agent.obs.base_quat.index({2}) = low_state_ros.imu.quaternion[3];
            agent.obs.base_quat.index({3}) = low_state_ros.imu.quaternion[0];

            update_dof_state(low_state_ros, agent);
            

            // Get record initial position
            if (motiontime >= 0 && motiontime < 10)
            {
                for (int k = 0; k < 12; k++)
                {
                    qInit[k] = low_state_ros.motor_state[k].q;
                }
            }

            // Move to the origin point with soft Kp/Kd
            if (motiontime >= 1 && motiontime < 1000)
            {
                rate_count++;
                float rate = rate_count / (1000.0 - 1.0);

                for (int k = 0; k < 12; k++)
                {
                    Kp[k] = 50.0;
                    Kd[k] = 2.0;
                }

                for (int k = 0; k < 12; k++)
                {

                    qDes[k] = jointLinearInterpolation(qInit[k], agent.params.default_dof_pos.index({k}).item<float>(), rate);
                    std::cout << "k: " << k << ", qDes[k]: " << qDes[k] << std::endl; // Добавляем вывод
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

            // if (motiontime > 3000)
            // {
            //     if (motiontime % (control_period) == 0)
            //     {
            //         torch::Tensor actions = agent.Act();
                    
            //         for (size_t k = 0; k < 12; k++)
            //         {
            //             qDes[k] = actions.index({net2joint_indexes[k]}).item().to<float>();
                    
            //         }
            //     }
            // }

            for (size_t k = 0; k < 12; k++)
            {

                low_cmd_ros.motor_cmd[k].q = qDes[k];
                low_cmd_ros.motor_cmd[k].dq = 0; // dqDes[k]; // Seems that Unitree doesn't need the velocity value here in position control mode
                low_cmd_ros.motor_cmd[k].kp = Kp[k];
                low_cmd_ros.motor_cmd[k].kd = Kd[k];
                low_cmd_ros.motor_cmd[k].tau = 0.0f;
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

