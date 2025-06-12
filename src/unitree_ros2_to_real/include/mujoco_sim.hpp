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


// #include <iostream>  // Для ввода/вывода в консоль (например, std::cout)
// #include <stdio.h>   // Стандартная библиотека ввода/вывода C (например, printf)
// #include <stdint.h>  // Для определения целочисленных типов фиксированного размера (например, uint8_t, int32_t)
// #include <math.h>    // Математические функции (например, sin, cos, sqrt)
// #include <cmath>     // Дополнительные математические функции (например, std::tanh)
// #include "rclcpp/rclcpp.hpp" // Основной заголовочный файл ROS2 C++
// #include <string>
// #include <vector>

// #include <torch/torch.h> //Библиотека для работы с тензорами и нейронными сетями
// #include "rl_agent.h"    //Предположительно, кастомный класс для reinforcement learning agent

// #include "std_msgs/msg/string.hpp" //Стандартные сообщения ROS2 (строки)
// #include "sensor_msgs/msg/joint_state.hpp" //Сообщения ROS2 для информации о состоянии суставов
// #include "sensor_msgs/msg/imu.hpp"   //Сообщения ROS2 для данных с IMU (инерциальный измерительный блок)
// #include "geometry_msgs/msg/wrench_stamped.hpp" //Сообщения ROS2 для информации о силе/моменте
// #include "geometry_msgs/msg/twist.hpp"  //Сообщения ROS2 для информации о линейной и угловой скорости
// #include "std_msgs/msg/u_int8_multi_array.hpp" //Сообщения ROS2 для массивов uint8_t

// #include "unitree_go/msg/low_state.hpp" //Определения сообщений ROS2, специфичных для робота Unitree Go2
// #include "unitree_go/msg/imu_state.hpp"   //Сообщения ROS2 для состояния IMU (внутри LowState)
// #include "unitree_go/msg/motor_state.hpp" //Сообщения ROS2 для состояния моторов (внутри LowState)
// #include "unitree_go/msg/low_cmd.hpp"   //Сообщения ROS2 для команд управления низкого уровня
// #include "unitree_go/msg/motor_cmd.hpp" //Сообщения ROS2 для команд управления отдельным мотором
// #include "unitree_go/msg/bms_cmd.hpp"   //Сообщения ROS2 для команд управления BMS (Battery Management System)
// #include "motor_crc.h" //Функции для вычисления CRC (Cyclic Redundancy Check) для проверки целостности данных

// using std::placeholders::_1;
// class RobotController {
// public:
//     RobotController();
//     void initializeRL(const std::string& config_path, const std::string& model_path);
//     unitree_go::msg::LowCmd update(const unitree_go::msg::LowState& state); 
//     std::string get_model_name() const;
//     std::string get_robot_name() const;
// private:    
//     void update_dof_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state);
//     float jointLinearInterpolation(float initPos, float targetPos, float rate);
//     void initial_positions(const std::array<unitree_go::msg::MotorState, 20>& motor_state);

//     Agent agent; // RL-агент
//     float qInit[12]; // Начальные позиции суставов
//     float qDes[12]; // Желаемые позиции суставов
//     float Kp[12]; // Жёсткость
//     float Kd[12]; // Демпфирование
//     int init_count; // Счётчик для записи qInit
//     int motiontime; // Счётчик итераций
//     double runing_time; // Время работы
//     int robot_state; // Состояние робота (STATE_INIT, STATE_READY)
//     const double dt; // Шаг времени (0.002 с)
//     const int Go2_NUM_MOTOR; // Количество моторов (12)
//     const std::string ROBOT_NAME; //Имя робота
//     const std::vector<int> net2joint_indexes; // Индексы для RL-агента
//     const std::vector<float> stiffness; // Жёсткость для RL
//     const std::vector<float> damping; // Демпфирование для RL

//     enum { STATE_INIT = 0, STATE_READY = 1 }; // Состояния робота

// };



// // Класс InterfaceRos: обрабатывает ROS2-топики и взаимодействует с RobotController
// class InterfaceRos: public rclcpp::Node{

// public:
//     InterfaceRos(); // Конструктор класса
// private:
//     void LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg);
//     void timer_callback_cmd();
//     void publish_imu(const unitree_go::msg::IMUState& imu_state);
//     void publish_motor_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state_array);
//     void send_command( unitree_go::msg::LowCmd& cmd);
//     void init_cmd();


//     RobotController controller;

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
//     rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub;
//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub;
//     rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub;
//     unitree_go::msg::LowState::SharedPtr latest_state;
//     unitree_go::msg::LowCmd low_cmd;
//     unitree_go::msg::IMUState imu;
//     unitree_go::msg::MotorState motor[12];
// };