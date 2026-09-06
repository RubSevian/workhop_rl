#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
// #include <unitree/idl/go2/WirelessController_.hpp>
// #include <unitree/robot/channel/channel_subscriber.hpp>
// #include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/motor_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/motor_cmd.hpp>
#include <unitree_go/msg/bms_cmd.hpp>
#include <torch/torch.h>
#include "rl_agent.h"
#include "motor_crc.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <cmath>


class RobotController {
public:
    RobotController();
    void initializeRL(const std::string& config_path, const std::string& model_path);
    unitree_go::msg::LowCmd update(const unitree_go::msg::LowState& state);
    void initial_positions(const std::array<unitree_go::msg::MotorState, 20>& motor_state);
    void update_dof_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state);
    float jointLinearInterpolation(float initPos, float targetPos, float rate);
    std::string get_model_name() const;
    std::string get_robot_name() const;
    int get_num_motors() const;
    void set_robot_name(const std::string& robot_name);
    void set_command(float x, float y, float z);
    void start_autonomy();
    // enum StateID {
    //     STATE_INIT,
    //     STATE_READY
    // };
    enum ControlMode {
        MODE_START = 0,
        MODE_STANDUP,
        MODE_DAMPING,
        MODE_RL
    };

    ControlMode mode_ = MODE_START;
    int last_key_ = 0;
    bool standup_done_ = false;

    int init_count;
    int motiontime;
    float runing_time;
    //StateID robot_state;
    const double dt;
    const int Go2_NUM_MOTOR;
    std::string ROBOT_NAME;
    float qInit[12];
    float qDes[12];
    const std::vector<int> net2joint_indexes;
    const std::vector<float> stiffness;
    const std::vector<float> damping;
    bool rl_inited_ = false;
    bool autonomous_requested_ = false;

private:
    Agent agent;

};

class InterfaceRos : public rclcpp::Node {
public:
    InterfaceRos(const std::string& network_interface);
    ~InterfaceRos();
private:
    void LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg);
    void CmdVelHandler(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void timer_callback_cmd();
    void send_command(unitree_go::msg::LowCmd& cmd);
    void init_cmd();
    void publish_imu(const unitree_go::msg::IMUState& imu_state);
    void publish_motor_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub;
    unitree_go::msg::LowCmd low_cmd;
    unitree_go::msg::LowState::SharedPtr latest_state;
    bool cmd_vel_received_ = false;
    std::chrono::steady_clock::time_point last_cmd_vel_time_{};
    double cmd_vel_timeout_sec_ = 0.25;
    double max_linear_x_ = 0.4;
    double max_linear_y_ = 0.2;
    double max_yaw_rate_ = 0.8;
    bool autostart_ = false;
    bool model_loaded_ = false;
    RobotController controller;

    // xKeySwitchUnion unitree_joy;
};
