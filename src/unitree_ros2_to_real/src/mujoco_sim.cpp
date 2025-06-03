/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of unitree go2 robot
 **/
// Это комментарий, описывающий основную цель программы: продемонстрировать, как отправлять команды моторам Unitree Go2 с использованием ROS2.

#include <iostream>  // Для ввода/вывода в консоль (например, std::cout)
#include <stdio.h>   // Стандартная библиотека ввода/вывода C (например, printf)
#include <stdint.h>  // Для определения целочисленных типов фиксированного размера (например, uint8_t, int32_t)
#include <math.h>    // Математические функции (например, sin, cos, sqrt)
#include <cmath>     // Дополнительные математические функции (например, std::tanh)
#include "rclcpp/rclcpp.hpp" // Основной заголовочный файл ROS2 C++

#include <torch/torch.h> //Библиотека для работы с тензорами и нейронными сетями
#include "rl_agent.h"    //Предположительно, кастомный класс для reinforcement learning agent

#include "std_msgs/msg/string.hpp" //Стандартные сообщения ROS2 (строки)
#include "sensor_msgs/msg/joint_state.hpp" //Сообщения ROS2 для информации о состоянии суставов
#include "sensor_msgs/msg/imu.hpp"   //Сообщения ROS2 для данных с IMU (инерциальный измерительный блок)
#include "geometry_msgs/msg/wrench_stamped.hpp" //Сообщения ROS2 для информации о силе/моменте
#include "geometry_msgs/msg/twist.hpp"  //Сообщения ROS2 для информации о линейной и угловой скорости
#include "std_msgs/msg/u_int8_multi_array.hpp" //Сообщения ROS2 для массивов uint8_t

#include "unitree_go/msg/low_state.hpp" //Определения сообщений ROS2, специфичных для робота Unitree Go2
#include "unitree_go/msg/imu_state.hpp"   //Сообщения ROS2 для состояния IMU (внутри LowState)
#include "unitree_go/msg/motor_state.hpp" //Сообщения ROS2 для состояния моторов (внутри LowState)
#include "unitree_go/msg/low_cmd.hpp"   //Сообщения ROS2 для команд управления низкого уровня
#include "unitree_go/msg/motor_cmd.hpp" //Сообщения ROS2 для команд управления отдельным мотором
#include "unitree_go/msg/bms_cmd.hpp"   //Сообщения ROS2 для команд управления BMS (Battery Management System)
#include "motor_crc.h" //Функции для вычисления CRC (Cyclic Redundancy Check) для проверки целостности данных

// Create a low_level_cmd_sender class for low state receive
// Это комментарий, указывающий на цель создания класса `low_level_cmd_sender`.

#define INFO_IMU 1   // Set 1 to info IMU states - Если установлено в 1, программа будет выводить информацию о состоянии IMU.
#define INFO_MOTOR 1 // Set 1 to info motor states - Если установлено в 1, программа будет выводить информацию о состоянии моторов.
#define HIGH_FREQ 1  // Set 1 to subscribe to low states with high frequencies (500Hz) - Не используется в коде.  Предположительно, указывает на необходимость подписки на сообщения с высокой частотой.
using std::placeholders::_1; //Для использования placeholders в лямбда-функциях (например, для std::bind)

const int Go2_NUM_MOTOR = 12; //Определение константы для количества моторов у робота Go2

// Объявление класса low_level_cmd_sender
// Этот класс отвечает за отправку команд управления низкого уровня на робота Unitree Go2.
class low_level_cmd_sender : public rclcpp::Node //Наследуется от rclcpp::Node, что делает его узлом ROS2
{
public:
    low_level_cmd_sender() : Node("low_level_cmd_sender") //Конструктор класса
    {
        cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10); //Создает Publisher для отправки сообщений типа LowCmd на топик "/lowcmd"
        state_sub = this->create_subscription<unitree_go::msg::LowState>( //Создает Subscriber для получения сообщений типа LowState с топика "/lowstate"
            "/lowstate", 10, std::bind(&low_level_cmd_sender::LowStateHandler, this, _1)); //При получении сообщения вызывается функция LowStateHandler
        low_cmd_pub = this->create_publisher<unitree_go::msg::LowCmd>("/go2/low_cmd", 10);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/go2/imu",10); //Публикуем sensor_msgs::msg::Imu а не unitree_go::msg::IMUState, чтобы другие узлы могли это переиспользовать
        motor_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/go2/motor_state",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), //Создает таймер, который будет вызывать функцию timer_callback_cmd с частотой 200 Гц
                                         std::bind(&low_level_cmd_sender::timer_callback_cmd, this));
        
        const std::string ROBOT_NAME = "go1";

        const std::string CONFIG_PATH = std::string(CONFIG_BASE_DIR) + "/weights/" + ROBOT_NAME + "/config.yaml";
        try {
            agent.ReadYaml(ROBOT_NAME,CONFIG_PATH);

        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            
        }
        std::string model_path = std::string(CONFIG_BASE_DIR) + "/weights/" + ROBOT_NAME + "/" + std::string(agent.params.model_name);
        if (!agent.Load_Model(model_path)) {
            RCLCPP_ERROR(this->get_logger(), "Error loading model from %s", model_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Model loaded successfully from %s", model_path.c_str());
        }

        init_cmd(); //Вызывает функцию init_cmd для инициализации команды управления (low_cmd)
    }

private:
    void LowStateHandler(unitree_go::msg::LowState::SharedPtr data)
    {
        if (INFO_IMU) //Если определена константа INFO_IMU
        {
        // Publish IMU data
        publish_imu(data->imu_state);
        // Publish motor state data
        publish_motor_state(data->motor_state);

            RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]); //Выводит углы Эйлера
            RCLCPP_INFO(this->get_logger(), "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f", //Выводит кватернион
                        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]);
            RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]); //Выводит данные гироскопа
            RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f", //Выводит данные акселерометра
                        imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]);
        }
        if (INFO_MOTOR) //Если определена константа INFO_MOTOR
        {
            for (int i = 0; i < Go2_NUM_MOTOR; i++) //Проходит по всем моторам
            {
                motor[i] = data->motor_state[i]; //Получает состояние мотора из сообщения
                RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f", //Выводит информацию о состоянии мотора
                            i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
                    
            }
            
        }
                                // Запись начальных позиций в первые 10 итераций
        static int count = 0;
        if (motiontime >= 0 && motiontime < 10) {
            for (int i = 0; i < Go2_NUM_MOTOR; i++) {
                qInit[i] = data->motor_state[i].q;
            }
        }
        // Обновление состояния RL-агента
        agent.obs.ang_vel.index({0}) = data->imu_state.gyroscope[0];
        agent.obs.ang_vel.index({1}) = data->imu_state.gyroscope[1];
        agent.obs.ang_vel.index({2}) = data->imu_state.gyroscope[2];
        agent.obs.base_quat.index({0}) = data->imu_state.quaternion[1];
        agent.obs.base_quat.index({1}) = data->imu_state.quaternion[2];
        agent.obs.base_quat.index({2}) = data->imu_state.quaternion[3];
        agent.obs.base_quat.index({3}) = data->imu_state.quaternion[0];


        update_dof_state(agent, data->motor_state);

    }

    // Функция, которая вызывается таймером с частотой 200 Гц
    void timer_callback_cmd()
    {
        //Увеличивает время работы
        motiontime++;
        runing_time += dt;


        if (robot_state == STATE_INIT) {
            // Фаза инициализации (1 секунда = 200 итераций при 200 Гц)
            if (motiontime < 2000) {
                float rate = motiontime / 2000.0f;
                for (int i = 0; i < Go2_NUM_MOTOR; i++) {
                    qDes[i] = jointLinearInterpolation(qInit[i], agent.params.default_dof_pos.index({i}).item<float>(), rate);
                    Kp[i] = 50.0;
                    Kd[i] = 2.0;
                    low_cmd.motor_cmd[i].q = qDes[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = Kp[i];
                    low_cmd.motor_cmd[i].kd = Kd[i];
                    low_cmd.motor_cmd[i].tau = 0;
                }
            } else {
                // Переход в STATE_READY
                for (int i = 0; i < Go2_NUM_MOTOR; i++) {
                    Kp[i] = stiffness[i];
                    Kd[i] = damping[i];
                }
                robot_state = STATE_READY;
            }
        } else if (robot_state == STATE_READY) {
            // Вызов RL-агента
            torch::Tensor actions = agent.Act();
            for (int i = 0; i < Go2_NUM_MOTOR; i++) {
                low_cmd.motor_cmd[i].q = actions.index({net2joint_indexes[i]}).item<float>();
                low_cmd.motor_cmd[i].dq = 0;
                low_cmd.motor_cmd[i].kp = Kp[i];
                low_cmd.motor_cmd[i].kd = Kd[i];
                low_cmd.motor_cmd[i].tau = 0;
            }
        }

        get_crc(low_cmd);            // Check motor cmd crc - Вычисляет CRC для команды управления (для проверки целостности)

        // Publish low_cmd
        publish_low_cmd(low_cmd);
        cmd_puber->publish(low_cmd); // Publish lowcmd message - Отправляет команду управления на топик "/lowcmd"
    }

    // Функция для инициализации команды управления (low_cmd)
    void init_cmd()
    {
        // Инициализирует motor_cmd для каждого мотора
        for (int i = 0; i < 20; i++) //Проходит по всем моторам (почему 20?? должно быть 12)
        {
            low_cmd.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode - Устанавливает режим управления: 0x01 - режим управления моментом, 0x00 - пассивный режим
            low_cmd.motor_cmd[i].q = PosStopF; //Устанавливает позицию остановки (forbidden position, PosStopF - константа, не определена в коде)
            low_cmd.motor_cmd[i].kp = 0; //Устанавливает коэффициент P регулятора равным 0
            low_cmd.motor_cmd[i].dq = VelStopF; //Устанавливает скорость остановки (forbidden velocity, VelStopF - константа, не определена в коде)
            low_cmd.motor_cmd[i].kd = 0; //Устанавливает коэффициент D регулятора равным 0
            low_cmd.motor_cmd[i].tau = 0; //Устанавливает момент равным 0
        }
    }

    void publish_imu (const unitree_go::msg::IMUState& imu_state){

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link"; // Замените на нужный frame_id

        // Copy data from unitree_go::msg::IMUState to sensor_msgs::msg::Imu
        imu_msg.orientation.x = imu_state.quaternion[1];
        imu_msg.orientation.y = imu_state.quaternion[2];
        imu_msg.orientation.z = imu_state.quaternion[3];
        imu_msg.orientation.w = imu_state.quaternion[0]; // Обратите внимание на порядок!

        imu_msg.angular_velocity.x = imu_state.gyroscope[0];
        imu_msg.angular_velocity.y = imu_state.gyroscope[1];
        imu_msg.angular_velocity.z = imu_state.gyroscope[2];

        imu_msg.linear_acceleration.x = imu_state.accelerometer[0];
        imu_msg.linear_acceleration.y = imu_state.accelerometer[1];
        imu_msg.linear_acceleration.z = imu_state.accelerometer[2];

        imu_pub->publish(imu_msg);

    }
    void publish_motor_state(const std::array<unitree_go::msg::MotorState, 20>& motor_state_array)
    {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.header.frame_id = "base"; //Замените на нужный frame_id

        //Заполняем имена joint-ов. Тут нужно убедиться, что порядок joint-ов совпадает с порядком в массиве motor_state_array
        joint_state_msg.name.resize(Go2_NUM_MOTOR);
        joint_state_msg.position.resize(Go2_NUM_MOTOR);
        joint_state_msg.velocity.resize(Go2_NUM_MOTOR);
        joint_state_msg.effort.resize(Go2_NUM_MOTOR);

        for (int i = 0; i < Go2_NUM_MOTOR; ++i)
        {
            joint_state_msg.name[i] = "motor_" + std::to_string(i); //Замените на правильные имена
            joint_state_msg.position[i] = motor_state_array[i].q;
            joint_state_msg.velocity[i] = motor_state_array[i].dq;
            joint_state_msg.effort[i] = motor_state_array[i].tau_est;
        }

        motor_state_pub->publish(joint_state_msg);
    }

    void publish_low_cmd(const unitree_go::msg::LowCmd& low_cmd) {
        low_cmd_pub->publish(low_cmd);
    }

    float jointLinearInterpolation(float initPos, float targetPos, float rate) {
        rate = std::min(std::max(rate, 0.0f), 1.0f);
        return initPos * (1 - rate) + targetPos * rate;
    }

    void update_dof_state(Agent &agent, const std::array<unitree_go::msg::MotorState, 20> &motor_states) {
        for (int i = 0; i < Go2_NUM_MOTOR; i++) {
            agent.obs.dof_pos.index({net2joint_indexes[i]}) = motor_states[i].q;
            agent.obs.dof_vel.index({net2joint_indexes[i]}) = motor_states[i].dq;
        }
    }

    enum CONTROL_MODE {
        CM_UNDEFINED,
        CM_POSITION,
        CM_TORQUE
    };

    enum ROBOT_STATE {
        STATE_INIT,
        STATE_READY
    };

    const std::vector<int> net2joint_indexes = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    const std::vector<float> stiffness = {20., 20., 20., 20., 20., 20., 20., 20., 20., 20., 20., 20.};
    const std::vector<float> damping = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    Agent agent;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub;
    unitree_go::msg::LowCmd low_cmd;
    unitree_go::msg::IMUState imu;
    unitree_go::msg::MotorState motor[12];
    float qInit[12] = {0};
    float qDes[12] = {0};
    float Kp[12] = {0};
    float Kd[12] = {0};
    int motiontime = 0;
    int robot_state = STATE_INIT;
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    
};






    // rclcpp::TimerBase::SharedPtr timer_;                             // ROS2 timer - Указатель на таймер ROS2
    // rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber; // ROS2 Publisher - Указатель на Publisher для отправки команд управления
    // rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub; // ROS2 Subscription - Указатель на Subscriber для получения сообщений о состоянии робота

    // unitree_go::msg::IMUState imu;         // Unitree go2 IMU message - Переменная для хранения состояния IMU
    // unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message - Массив для хранения состояния моторов
    // unitree_go::msg::LowCmd low_cmd;       // Переменная для хранения команды управления

    // double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763, //Позиции суставов для положения "стоя"
    //                                  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    // double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455, //Позиции суставов для положения "сидя"
    //                                    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    // double dt = 0.002;       // Time step - Шаг времени (0.002 секунды, соответствует частоте 500 Гц)
    // double runing_time = 0.0; // Running time - Время работы программы
    // double phase = 0.0;      // Phase - Фаза движения (используется для интерполяции)

// Главная функция программы
int main(int argc, char **argv)
{
    std::cout << "Press enter to start"; //Выводит сообщение в консоль
    std::cin.get(); //Ожидает нажатия Enter

    rclcpp::init(argc, argv);                             // Initialize rclcpp - Инициализирует ROS2
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals - Создает объект таймера (не используется напрямую)
    Agent agent;
    auto node = std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make share with low_level_cmd_sender class - Создает узел ROS2 и передает его в класс low_level_cmd_sender
    rclcpp::spin(node);                                   // Run ROS2 node - Запускает узел ROS2 (начинает обработку сообщений и таймеров)
    rclcpp::shutdown();                                   // Exit - Завершает работу ROS2
    return 0; //Выход из программы
}