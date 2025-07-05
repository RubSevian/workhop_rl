/**
 * This example demonstrates how to use ROS2 to send low-level motor commands to Unitree Go2 robot
 */
#include "ros2_rl_go2.hpp"

#define INFO_IMU 1
#define INFO_MOTOR 1
#define INFO_FOOT_FORCE 1
#define INFO_BATTERY 1
#define HIGH_FREQ 1
#define QUAT_WXYZ 0
using std::placeholders::_1;

RobotController::RobotController() :
    init_count(0), motiontime(0), runing_time(0.0), robot_state(STATE_INIT),
    dt(0.02), Go2_NUM_MOTOR(12), ROBOT_NAME("go1"),
    net2joint_indexes({3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8})
    {
    std::fill(std::begin(qInit), std::end(qInit), 0.0f);
    std::fill(std::begin(qDes), std::end(qDes), 0.0f);
    std::fill(std::begin(Kp), std::end(Kp), 50.0f);
    std::fill(std::begin(Kd), std::end(Kd), 2.0f);
}

std::string RobotController::get_model_name() const {
    return std::string(agent.params.model_name);
}

std::string RobotController::get_robot_name() const {
    return ROBOT_NAME;
}

int RobotController::get_num_motors() const {
    return Go2_NUM_MOTOR;
}

void RobotController::initializeRL(const std::string& config_path, const std::string& model_path) {
    try {
        if (!config_path.empty()) {
            agent.ReadYaml(ROBOT_NAME, config_path);
        }
        if (!model_path.empty()) {
            if (!agent.Load_Model(model_path)) {
                throw std::runtime_error("Failed to load model from " + model_path);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        throw;
    }
}

unitree_go::msg::LowCmd RobotController::update(const unitree_go::msg::LowState& state, const unitree_go::msg::dds_::WirelessController_& joystick,const xKeySwitchUnion& joy) {
    unitree_go::msg::LowCmd cmd;
    initial_positions(state.motor_state);
    update_dof_state(state.motor_state);

    bool valid_imu = (state.imu_state.quaternion[0] != 0.0 || state.imu_state.quaternion[1] != 0.0 ||
                      state.imu_state.quaternion[2] != 0.0 || state.imu_state.quaternion[3] != 0.0);
    if (!valid_imu) {
        std::cerr << "Warning: Invalid IMU data (zero quaternion)" << std::endl;
    }

    agent.obs.ang_vel.index({0}) = state.imu_state.gyroscope[0];
    agent.obs.ang_vel.index({1}) = state.imu_state.gyroscope[1];
    agent.obs.ang_vel.index({2}) = state.imu_state.gyroscope[2];
    agent.obs.base_quat.index({0}) = state.imu_state.quaternion[1]; // x
    agent.obs.base_quat.index({1}) = state.imu_state.quaternion[2]; // y
    agent.obs.base_quat.index({2}) = state.imu_state.quaternion[3]; // z
    agent.obs.base_quat.index({3}) = state.imu_state.quaternion[0]; // w

        // Обработка данных джойстика
    if (joystick.lx() != 0.0f || joystick.ly() != 0.0f || joystick.rx() != 0.0f || joy.components.X) {
        agent.obs.command.index({0}) = joystick.ly();
        agent.obs.command.index({1}) = -joystick.rx();
        agent.obs.command.index({2}) = -joystick.lx(); 
        if (joy.components.X) {
            agent.obs.command.index({0}) = 0.0f;       
            agent.obs.command.index({1}) = 0.0f;
            agent.obs.command.index({2}) = 0.0f;
        }
        std::cout << "Command: ly=" << agent.obs.command.index({0}).item<float>()
                  << ", -rx=" << agent.obs.command.index({1}).item<float>()
                  << ", -lx=" << agent.obs.command.index({2}).item<float>() << std::endl;
    } else {
        agent.obs.command.index({0}) = 0.0f;             
        agent.obs.command.index({1}) = 0.0f;
        agent.obs.command.index({2}) = 0.0f;
        std::cout << "No joystick data, stopping: command=[0, 0, 0]" << std::endl;
    }

    if (motiontime < 2000) {
        float rate = motiontime / 2000.0f;
        for (int i = 0; i < Go2_NUM_MOTOR; i++) {
            qDes[i] = jointLinearInterpolation(qInit[i], agent.params.default_dof_pos.index({i}).item<float>(), rate);
            cmd.motor_cmd[i].mode = 0x01; // Torque mode
            cmd.motor_cmd[i].q = qDes[i];
            cmd.motor_cmd[i].dq = 0;
            cmd.motor_cmd[i].kp = Kp[i];
            cmd.motor_cmd[i].kd = Kd[i];
            cmd.motor_cmd[i].tau = 0;
        }
        if (motiontime % 100 == 0) {
            std::stringstream ss;
            ss << "Default_dof_pos: ";
            for (int i = 0; i < Go2_NUM_MOTOR; i++) ss << agent.params.default_dof_pos.index({i}).item<float>() << " ";
            std::cout << ss.str() << std::endl;
        }
    } else {
        robot_state = STATE_READY;
        torch::Tensor actions = agent.Act();
        for (int i = 0; i < Go2_NUM_MOTOR; i++) {
            cmd.motor_cmd[i].mode = 0x01; // Torque mode
            cmd.motor_cmd[i].q = actions.index({net2joint_indexes[i]}).item<float>();
            cmd.motor_cmd[i].dq = 0;
            cmd.motor_cmd[i].kp = agent.params.stiffness;
            cmd.motor_cmd[i].kd = agent.params.damping;
            cmd.motor_cmd[i].tau = 0;
        }
        if (motiontime % 100 == 0) {
            std::stringstream ss;
            ss << "Actions: ";
            for (int i = 0; i < Go2_NUM_MOTOR; i++) {
                ss << actions.index({net2joint_indexes[i]}).item<float>() << " ";
            }
            std::cout << ss.str() << std::endl;
        }
    }

    motiontime++;
    runing_time += dt;
    return cmd;
}

void RobotController::initial_positions(const std::array<unitree_go::msg::MotorState, 20>& motor_state) {
    if (init_count < 10) {
        for (int i = 0; i < Go2_NUM_MOTOR; i++) {
            qInit[i] = motor_state[i].q;
        }
        if (init_count == 0) {
            std::stringstream ss;
            ss << "Initial positions: ";
            for (int i = 0; i < Go2_NUM_MOTOR; i++) ss << qInit[i] << " ";
            std::cout << ss.str() << std::endl;
        }
        init_count++;
    }
}

void RobotController::update_dof_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state) {
    for (int i = 0; i < Go2_NUM_MOTOR; i++) {
        agent.obs.dof_pos.index({net2joint_indexes[i]}) = motor_state[i].q;
        agent.obs.dof_vel.index({net2joint_indexes[i]}) = motor_state[i].dq;
    }
}

float RobotController::jointLinearInterpolation(float initPos, float targetPos, float rate) {
    rate = std::min(std::max(rate, 0.0f), 1.0f);
    return initPos * (1 - rate) + targetPos * rate;
}

InterfaceRos::InterfaceRos(const std::string& network_interface) : Node("low_level_cmd_sender") {
    cmd_puber = create_publisher<unitree_go::msg::LowCmd>("lowcmd", 10);
    imu_pub = create_publisher<sensor_msgs::msg::Imu>("go2/imu", 10);
    motor_state_pub = create_publisher<sensor_msgs::msg::JointState>("go2/motor_state", 10);
    state_sub = create_subscription<unitree_go::msg::LowState>(
        "lowstate", 10, std::bind(&InterfaceRos::LowStateHandler, this, _1));
    joystick_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>("rt/wirelesscontroller"));
    joystick_sub->InitChannel(std::bind(&InterfaceRos::JoystickHandler, this, std::placeholders::_1), 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&InterfaceRos::timer_callback_cmd, this));
    init_cmd();
    try {
        std::string config_path = std::string(CONFIG_BASE_DIR) + "/weights/" + controller.get_robot_name() + "/config.yaml";
        controller.initializeRL(config_path, "");
        std::string model_path = std::string(CONFIG_BASE_DIR) + "/weights/" + controller.get_robot_name() + "/" + controller.get_model_name();
        RCLCPP_INFO(this->get_logger(), "CONFIG_PATH: %s", config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "MODEL_PATH: %s", model_path.c_str());
        controller.initializeRL("", model_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing RL: %s", e.what());
    }
}

InterfaceRos::~InterfaceRos() {
    
}

void InterfaceRos::JoystickHandler(const void* msg){
    joystick = *(unitree_go::msg::dds_::WirelessController_ *)msg;
    this->unitree_joy.value = joystick.keys();
     RCLCPP_INFO(this->get_logger(), "Joystick: lx=%f, ly=%f, rx=%f, btn_x=%d",
                joystick.ly(),joystick.rx(),joystick.lx(), unitree_joy.components.X);

}


void InterfaceRos::init_cmd() {
    // low_cmd.head[0] = 0xFE;
    // low_cmd.head[1] = 0xEF;
    // low_cmd.level_flag = 0x00;
    // low_cmd.frame_reserve = 0;
    for (int i = 0; i < 20; i++) {
        low_cmd.motor_cmd[i].mode = 0x01;
        low_cmd.motor_cmd[i].q = PosStopF;
        low_cmd.motor_cmd[i].dq = VelStopF;
        low_cmd.motor_cmd[i].kp = 0.0;
        low_cmd.motor_cmd[i].kd = 0.0;
        low_cmd.motor_cmd[i].tau = 0.0;
    }
    low_cmd.crc = 0;
}

void InterfaceRos::LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg) {
    latest_state = msg;
    publish_imu(msg->imu_state);
    publish_motor_state(msg->motor_state);
    if (INFO_IMU) {
        RCLCPP_INFO(this->get_logger(), "IMU: gyro = [%f, %f, %f], quat = [%f, %f, %f, %f]",
                    msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1], msg->imu_state.gyroscope[2],
                    msg->imu_state.quaternion[0], msg->imu_state.quaternion[1],
                    msg->imu_state.quaternion[2], msg->imu_state.quaternion[3]);
    }
    if (INFO_MOTOR) {
        for (int i = 0; i < controller.get_num_motors(); i++) {
            RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; tau: %f",
                        i, msg->motor_state[i].q, msg->motor_state[i].dq, msg->motor_state[i].tau_est);
        }
    }
    if (INFO_FOOT_FORCE) {
        for (int i = 0; i < 4; i++) {
            RCLCPP_INFO(this->get_logger(), "Foot force -- foot%d: %d", i, msg->foot_force[i]);
            RCLCPP_INFO(this->get_logger(), "Estimated foot force -- foot%d: %d", i, msg->foot_force_est[i]);
        }
    }
    if (INFO_BATTERY) {
        RCLCPP_INFO(this->get_logger(), "Battery state -- current: %f; voltage: %f",
                    msg->power_a, msg->power_v);
    }
}

void InterfaceRos::timer_callback_cmd() {
    if (!latest_state) {
        RCLCPP_WARN(this->get_logger(), "Waiting for first state message");
        return;
    }
    low_cmd = controller.update(*latest_state,joystick, unitree_joy);
    send_command(low_cmd);
}

void InterfaceRos::publish_imu(const unitree_go::msg::IMUState& imu_state) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    msg.angular_velocity.x = imu_state.gyroscope[0];
    msg.angular_velocity.y = imu_state.gyroscope[1];
    msg.angular_velocity.z = imu_state.gyroscope[2];
    msg.linear_acceleration.x = imu_state.accelerometer[0];
    msg.linear_acceleration.y = imu_state.accelerometer[1];
    msg.linear_acceleration.z = imu_state.accelerometer[2];
    msg.orientation.x = imu_state.quaternion[1];
    msg.orientation.y = imu_state.quaternion[2];
    msg.orientation.z = imu_state.quaternion[3];
    msg.orientation.w = imu_state.quaternion[0];
    imu_pub->publish(msg);
}

void InterfaceRos::publish_motor_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base";
    msg.name.resize(controller.get_num_motors());
    msg.position.resize(controller.get_num_motors());
    msg.velocity.resize(controller.get_num_motors());
    msg.effort.resize(controller.get_num_motors());
    for (int i = 0; i < controller.get_num_motors(); i++) {
        msg.name[i] = "motor_" + std::to_string(i);
        msg.position[i] = motor_state[i].q;
        msg.velocity[i] = motor_state[i].dq;
        msg.effort[i] = motor_state[i].tau_est;
    }
    motor_state_pub->publish(msg);
}

void InterfaceRos::send_command(unitree_go::msg::LowCmd& cmd) {
    get_crc(cmd);
    cmd_puber->publish(cmd);
}

int main(int argc, char** argv) {

    // std::cout << "Press enter to start";
    // std::cin.get();
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<InterfaceRos>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }
    ChannelFactory::Instance()->Init(0, argv[1]);
    RCLCPP_INFO(rclcpp::get_logger("main"), "ChannelFactory initialized");
    std::cout << "Press enter to start";
    std::cin.get();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceRos>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



