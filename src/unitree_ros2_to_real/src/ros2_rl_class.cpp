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
#include <chrono>


#define DIMENSION 3


using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

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

const std::vector<std::string> joint_names = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

const std::vector<double> default_joint_angles = {
    -0., 0.8, -1.3,
    0., 0.8, -1.3,
    -0.0, 0.8, -1.3,
    0.0, 0.8, -1.3};

const std::vector<int> net2joint_indexes = {
    3, 4, 5,
    0, 1, 2,
    9, 10, 11,
    6, 7, 8};

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

const std::vector<std::string> urdf_feet_names = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};


std::map<std::string, std::string> model_paths = {
            { "trotting", "/home/a/ros2_ws/src/unitree_rl_controller/weights/noise_batchsize_feetContact_go1.pt"},
            { "gallop", "/home/a/ros2_ws/src/unitree_rl_controller/weights/policy_gallop_v21_10.pt"}
        };

class RLController : public rclcpp::Node
{
  public:
    RLController()
    : Node("node_ros2_rl")
    {
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

        UDP state_udp(LOWLEVEL);
        // LowCmd cmd = {0};
        // LowState state = {0};
        state_udp.InitCmdData(cmd);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&RLController::timer_callback, this));
        pub_jointstates = this->create_publisher<sensor_msgs::msg::JointState>("/go1/joint_states", 1000);
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/go1/imu0", 1000);
        pub_FR_force = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legFR/force_torque_states", 1000);
        pub_FL_force = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legFL/force_torque_states", 1000);
        pub_RR_force = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legRR/force_torque_states", 1000);
        pub_RL_force = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/go1/legRL/force_torque_states", 1000);
        pub_wireless_remote = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/go1/remote", 100);
        auto pub = this->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1000);

        Agent agent;

        if (!agent.load_model_dict(model_paths))
            RCLCPP_ERROR(this->get_logger(), "Error loading the model\n");
        else
            RCLCPP_INFO(this->get_logger(), "Model loaded successfully\n");
        }

        bool initiated_flag = false; // initiate need time
        int count = 0;        
        mean_smoothing<double, DIMENSION> meansmth; 

            


//    private:
        void set_init_cmd()
        {
            RLController::low_cmd_ros.level_flag = LOWLEVEL;
            for (int i = 0; i < 12; i++)
            {
                RLController::low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
                RLController::low_cmd_ros.motor_cmd[i].q = PosStopF; // 禁止位置环
                RLController::low_cmd_ros.motor_cmd[i].kp = 0;
                RLController::low_cmd_ros.motor_cmd[i].dq = VelStopF; // 禁止速度环
                RLController::low_cmd_ros.motor_cmd[i].kd = 0;
                RLController::low_cmd_ros.motor_cmd[i].tau = 0;
            }
        }

        void timer_callback()
            {
                RLController::set_init_cmd();
                RLController::state_udp.Recv();
                state_udp.GetRecv(RLController::state);
                RLController::low_state_ros = state2rosMsg(RLController::state);
            }

       

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_jointstates;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_filter;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_FL_force, pub_FR_force, pub_RL_force, pub_RR_force;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_wireless_remote;

        ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;
        ros2_unitree_legged_msgs::msg::LowState low_state_ros;

        rclcpp::TimerBase::SharedPtr timer_;

        LowCmd cmd = {0};
        LowState state = {0};

        
        

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  auto node = std::make_shared<RLController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}