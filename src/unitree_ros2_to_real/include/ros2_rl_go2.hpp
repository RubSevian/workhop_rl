#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/motor_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/motor_cmd.hpp>
#include <unitree_go/msg/bms_cmd.hpp>
#include <torch/torch.h>
#include "rl_agent.h"
#include "motor_crc.h"

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
    enum StateID {
        STATE_INIT,
        STATE_READY
    };

    int init_count;
    int motiontime;
    float runing_time;
    StateID robot_state;
    const double dt;
    const int Go2_NUM_MOTOR;
    const std::string ROBOT_NAME;
    float qInit[12];
    float qDes[12];
    float Kp[12];
    float Kd[12];
    const std::vector<int> net2joint_indexes;
    const std::vector<float> stiffness;
    const std::vector<float> damping;

private:
    Agent agent;

};

class InterfaceRos : public rclcpp::Node {
public:
    InterfaceRos();
private:
    void LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg);
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
    unitree_go::msg::LowCmd low_cmd;
    unitree_go::msg::LowState::SharedPtr latest_state;
    RobotController controller;
};
