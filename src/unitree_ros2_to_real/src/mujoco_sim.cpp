/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of unitree go2 robot
 **/
// Это комментарий, описывающий основную цель программы: продемонстрировать, как отправлять команды моторам Unitree Go2 с использованием ROS2.

#include "mujoco_sim.hpp"
// Create a low_level_cmd_sender class for low state receive
// Это комментарий, указывающий на цель создания класса `low_level_cmd_sender`.

#define INFO_IMU 1   // Set 1 to info IMU states - Если установлено в 1, программа будет выводить информацию о состоянии IMU.
#define INFO_MOTOR 1 // Set 1 to info motor states - Если установлено в 1, программа будет выводить информацию о состоянии моторов.
#define HIGH_FREQ 1  // Set 1 to subscribe to low states with high frequencies (500Hz) - Не используется в коде.  Предположительно, указывает на необходимость подписки на сообщения с высокой частотой.
using std::placeholders::_1; //Для использования placeholders в лямбда-функциях (например, для std::bind)

RobotController::RobotController():
    init_count(0),motiontime(0),runing_time(0.0),robot_state(STATE_INIT),
    dt(0.02),Go2_NUM_MOTOR(12),ROBOT_NAME("go1"),
    net2joint_indexes({3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}) {
    std::fill(std::begin(qInit), std::end(qInit), 0.0f);
    std::fill(std::begin(qDes), std::end(qDes), 0.0f);
    }
std::string RobotController::get_model_name() const {
    return std::string(agent.params.model_name);
}
std::string RobotController::get_robot_name() const {
    return ROBOT_NAME;
}

void RobotController::set_command(float x, float y, float z) {
    agent.obs.command = torch::tensor({x, y, z});
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

unitree_go::msg::LowCmd RobotController::update(const unitree_go::msg::LowState& state) {
    unitree_go::msg::LowCmd cmd;
    initial_positions(state.motor_state);
    update_dof_state(state.motor_state);

    
    agent.obs.ang_vel.index({0}) = state.imu_state.gyroscope[0];
    agent.obs.ang_vel.index({1}) = state.imu_state.gyroscope[1];
    agent.obs.ang_vel.index({2}) = state.imu_state.gyroscope[2];
    agent.obs.base_quat.index({0}) = state.imu_state.quaternion[1];
    agent.obs.base_quat.index({1}) = state.imu_state.quaternion[2];
    agent.obs.base_quat.index({2}) = state.imu_state.quaternion[3];
    agent.obs.base_quat.index({3}) = state.imu_state.quaternion[0];

    if (motiontime < 500) {
        float rate = motiontime / 300.0f;
        for (int i = 0; i < Go2_NUM_MOTOR; i++) {
            qDes[i] = jointLinearInterpolation(qInit[i], agent.params.default_dof_pos.index({i}).item<float>(), rate);
            cmd.motor_cmd[i].q = qDes[i];
            cmd.motor_cmd[i].dq = 0;
            cmd.motor_cmd[i].kp = agent.params.fixed_kp.index({i}).item<float>();
            cmd.motor_cmd[i].kd = agent.params.fixed_kd.index({i}).item<float>();
            cmd.motor_cmd[i].tau = 0;
        }
    } else {
        robot_state = STATE_READY;
        agent.UpdatePhase(runing_time);
        torch::Tensor actions = agent.Act();
        for (int i = 0; i < Go2_NUM_MOTOR; i++) {
            cmd.motor_cmd[i].q = actions.index({net2joint_indexes[i]}).item<float>();
            cmd.motor_cmd[i].dq = 0;
            cmd.motor_cmd[i].kp = agent.params.rl_kp.index({i}).item<float>();
            cmd.motor_cmd[i].kd = agent.params.rl_kd.index({i}).item<float>();
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


// InterfaceRos implementation
InterfaceRos::InterfaceRos() : Node("low_level_cmd_sender") {
    cmd_puber = create_publisher<unitree_go::msg::LowCmd>("lowcmd", 10);
    low_cmd_pub = create_publisher<unitree_go::msg::LowCmd>("go2/low_cmd", 10);
    imu_pub = create_publisher<sensor_msgs::msg::Imu>("go2/imu", 10);
    motor_state_pub = create_publisher<sensor_msgs::msg::JointState>("go2/motor_state", 10);
    state_sub = create_subscription<unitree_go::msg::LowState>(
        "lowstate", 10, std::bind(&InterfaceRos::LowStateHandler, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&InterfaceRos::timer_callback_cmd, this));
    init_cmd();
    init_glfw();
    try {
        std::string CONFIG_PATH = std::string(CONFIG_BASE_DIR) + "/weights/" + controller.get_robot_name() + "/config.yaml";
        controller.initializeRL(CONFIG_PATH, "");
        std::string model_path = std::string(CONFIG_BASE_DIR) + "/weights/" + controller.get_robot_name() + "/" + controller.get_model_name();
        RCLCPP_INFO(this->get_logger(), "CONFIG_PATH: %s", CONFIG_PATH.c_str());
        RCLCPP_INFO(this->get_logger(), "model_path: %s", model_path.c_str());
        controller.initializeRL("", model_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing RL: %s", e.what());
    }
}

InterfaceRos::~InterfaceRos() {
    if (window) {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
}

void InterfaceRos::init_glfw() {
    if (!glfwInit()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW");
        throw std::runtime_error("GLFW initialization failed");
    }

    window = glfwCreateWindow(640, 480, "MuJoCo Simulation - Keyboard Control", NULL, NULL);
    if (!window) {
        glfwTerminate();
        RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window");
        throw std::runtime_error("GLFW window creation failed");
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, &keyboard_state);
    glfwSetKeyCallback(window, key_callback);
}

void InterfaceRos::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    KeyboardState* state = static_cast<KeyboardState*>(glfwGetWindowUserPointer(window));
    bool pressed = (action == GLFW_PRESS || action == GLFW_REPEAT);

    if (key == GLFW_KEY_W) state->w_pressed = pressed;
    if (key == GLFW_KEY_S) state->s_pressed = pressed;
    if (key == GLFW_KEY_A) state->a_pressed = pressed;
    if (key == GLFW_KEY_D) state->d_pressed = pressed;
    if (key == GLFW_KEY_SPACE) state->space_pressed = pressed;
}

void InterfaceRos::init_cmd() {
    low_cmd.head[0] = 0xFE;
    low_cmd.head[1] = 0xEF;
    low_cmd.level_flag = 0x00;
    low_cmd.frame_reserve = 0;
    for (int i = 0; i < 20; i++) {
        low_cmd.motor_cmd[i].mode = 0x01;
        low_cmd.motor_cmd[i].q = 0.0;
        low_cmd.motor_cmd[i].dq = 0.0;
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

    // RCLCPP_INFO(this->get_logger(), "IMU: gyro = [%f, %f, %f], quat = [%f, %f, %f, %f]",
    //             msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1], msg->imu_state.gyroscope[2],
    //             msg->imu_state.quaternion[0], msg->imu_state.quaternion[1],
    //             msg->imu_state.quaternion[2], msg->imu_state.quaternion[3]);
}
void InterfaceRos::timer_callback_cmd() {
    if (!latest_state) {
        RCLCPP_WARN(this->get_logger(), "Waiting for first 10 iterations to initialize");
        return;
    }
// Обновляем команду только при активных клавишах или сбросе
    bool command_changed = false;
    float x = last_command[0];
    float y = last_command[1];
    float z = last_command[2];

    if (keyboard_state.w_pressed) {
        x = 1.0f;
        command_changed = true;
    } else if (keyboard_state.s_pressed) {
        x = -1.0f;
        command_changed = true;
    }
    if (keyboard_state.a_pressed) {
        y = 1.0f;
        command_changed = true;
    } else if (keyboard_state.d_pressed) {
        y = -1.0f;
        command_changed = true;
    }
    if (keyboard_state.space_pressed) {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        command_changed = true;
    }

    // Сохраняем команду, если она изменилась
    if (command_changed) {
        last_command = {x, y, z};
        controller.set_command(x, y, z);
        RCLCPP_INFO(this->get_logger(), "Command updated: x=%f, y=%f, z=%f", x, y, z);
    }

    low_cmd = controller.update(*latest_state);
    send_command(low_cmd);

    glfwPollEvents();
    // float x = 0.0f, y= 0.0f;
    // if (keyboard_state.w_pressed) x += 0.5f;
    // if (keyboard_state.s_pressed) x -= 0.5f;
    // if (keyboard_state.a_pressed) y += 0.5f;
    // if (keyboard_state.d_pressed) y -= 0.5f;
    // controller.set_command(x, y, 0.0f);
    // RCLCPP_INFO(this->get_logger(), "Command: x=%f, y=%f", x, y);

    // low_cmd = controller.update(*latest_state);
    // send_command(low_cmd);
    // glfwPollEvents();
}
void InterfaceRos::publish_imu(const unitree_go::msg::IMUState& imu_state) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    msg.orientation.x = imu_state.quaternion[1]; // x
    msg.orientation.y = imu_state.quaternion[2]; // y
    msg.orientation.z = imu_state.quaternion[3]; // z
    msg.orientation.w = imu_state.quaternion[0]; // w
    msg.angular_velocity.x = imu_state.gyroscope[0];
    msg.angular_velocity.y = imu_state.gyroscope[1];
    msg.angular_velocity.z = imu_state.gyroscope[2];
    msg.linear_acceleration.x = imu_state.accelerometer[0];
    msg.linear_acceleration.y = imu_state.accelerometer[1];
    msg.linear_acceleration.z = imu_state.accelerometer[2];
    imu_pub->publish(msg);
}

void InterfaceRos::publish_motor_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base";
    for (int i = 0; i < 12; i++) {
        msg.name.push_back("joint_" + std::to_string(i));
        msg.position.push_back(motor_state[i].q);
        msg.velocity.push_back(motor_state[i].dq);
        msg.effort.push_back(motor_state[i].tau_est);
    }
    motor_state_pub->publish(msg);
}

void InterfaceRos::send_command(unitree_go::msg::LowCmd& cmd) {
    get_crc(cmd);
    cmd_puber->publish(cmd);
    low_cmd_pub->publish(cmd);
}
// Главная функция программы
int main(int argc, char** argv) {
    std::cout << "Press enter to start";
    std::cin.get();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceRos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}