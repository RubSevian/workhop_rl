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
        // the cmd_puber is set to subscribe "/lowcmd" topic
        cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10); //Создает Publisher для отправки сообщений типа LowCmd на топик "/lowcmd"

        state_sub = this->create_subscription<unitree_go::msg::LowState>( //Создает Subscriber для получения сообщений типа LowState с топика "/lowstate"
            "/lowstate", 10, std::bind(&low_level_cmd_sender::LowStateHandler, this, _1)); //При получении сообщения вызывается функция LowStateHandler

        // The timer is set to 200Hz, and bind to low_level_cmd_sender::timer_callback function
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), //Создает таймер, который будет вызывать функцию timer_callback_cmd с частотой 200 Гц
                                         std::bind(&low_level_cmd_sender::timer_callback_cmd, this));

        // Initialize lowcmd
        init_cmd(); //Вызывает функцию init_cmd для инициализации команды управления (low_cmd)
    }

private:
    // Функция, которая вызывается при получении сообщения с топика "/lowstate"
    void LowStateHandler(unitree_go::msg::LowState::SharedPtr data)
    {
        if (INFO_IMU) //Если определена константа INFO_IMU
        {
            // Info IMU states - Выводит информацию о состоянии IMU в консоль
            // RPY euler angle(ZYX order respected to body frame)
            // Quaternion
            // Gyroscope (raw data)
            // Accelerometer (raw data)
            imu = data->imu_state; //Получает состояние IMU из сообщения
            RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]); //Выводит углы Эйлера
            RCLCPP_INFO(this->get_logger(), "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f", //Выводит кватернион
                        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]);
            RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]); //Выводит данные гироскопа
            RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f", //Выводит данные акселерометра
                        imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]);
        }
        if (INFO_MOTOR) //Если определена константа INFO_MOTOR
        {
            // Info motor states - Выводит информацию о состоянии моторов в консоль
            // q: angluar (rad)
            // dq: angluar velocity (rad/s)
            // ddq: angluar acceleration (rad/(s^2))
            // tau_est: Estimated external torque
            for (int i = 0; i < Go2_NUM_MOTOR; i++) //Проходит по всем моторам
            {
                motor[i] = data->motor_state[i]; //Получает состояние мотора из сообщения
                RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f", //Выводит информацию о состоянии мотора
                            i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
            }
        }
    }

    // Функция, которая вызывается таймером с частотой 200 Гц
    void timer_callback_cmd()
    {
        //Увеличивает время работы
        runing_time += dt;
        if (runing_time < 3.0)
        {
            // Stand up in first 3 second - Робот встает в течение первых 3 секунд

            // Total time for standing up or standing down is about 1.2s - Общее время для вставания или приседания составляет около 1.2 секунды
            phase = tanh(runing_time / 1.2); //Вычисляет фазу движения с помощью гиперболического тангенса
            for (int i = 0; i < 12; i++) //Проходит по всем моторам
            {
                low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]; //Устанавливает целевую позицию мотора, интерполируя между позицией стоя и позицией сидя
                low_cmd.motor_cmd[i].dq = 0; //Устанавливает целевую скорость мотора равной 0
                low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0; //Устанавливает коэффициент P регулятора
                low_cmd.motor_cmd[i].kd = 3.5; //Устанавливает коэффициент D регулятора
                low_cmd.motor_cmd[i].tau = 0; //Устанавливает момент равным 0
            }
        }
        else //После 3 секунд
        {
            // Then stand down - Робот садится
            phase = tanh((runing_time - 3.0) / 1.2); //Вычисляет фазу движения
            for (int i = 0; i < 12; i++) //Проходит по всем моторам
            {
                low_cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i]; //Устанавливает целевую позицию мотора, интерполируя между позицией сидя и позицией стоя
                low_cmd.motor_cmd[i].dq = 0; //Устанавливает целевую скорость мотора равной 0
                low_cmd.motor_cmd[i].kp = 50; //Устанавливает коэффициент P регулятора
                low_cmd.motor_cmd[i].kd = 3.5; //Устанавливает коэффициент D регулятора
                low_cmd.motor_cmd[i].tau = 0; //Устанавливает момент равным 0
            }
        }

        get_crc(low_cmd);            // Check motor cmd crc - Вычисляет CRC для команды управления (для проверки целостности)
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

    rclcpp::TimerBase::SharedPtr timer_;                             // ROS2 timer - Указатель на таймер ROS2
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber; // ROS2 Publisher - Указатель на Publisher для отправки команд управления
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_sub; // ROS2 Subscription - Указатель на Subscriber для получения сообщений о состоянии робота

    unitree_go::msg::IMUState imu;         // Unitree go2 IMU message - Переменная для хранения состояния IMU
    unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message - Массив для хранения состояния моторов
    unitree_go::msg::LowCmd low_cmd;       // Переменная для хранения команды управления

    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763, //Позиции суставов для положения "стоя"
                                     0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455, //Позиции суставов для положения "сидя"
                                       1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double dt = 0.002;       // Time step - Шаг времени (0.002 секунды, соответствует частоте 500 Гц)
    double runing_time = 0.0; // Running time - Время работы программы
    double phase = 0.0;      // Phase - Фаза движения (используется для интерполяции)
};

// Главная функция программы
int main(int argc, char **argv)
{
    std::cout << "Press enter to start"; //Выводит сообщение в консоль
    std::cin.get(); //Ожидает нажатия Enter

    rclcpp::init(argc, argv);                             // Initialize rclcpp - Инициализирует ROS2
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals - Создает объект таймера (не используется напрямую)
    auto node = std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make share with low_level_cmd_sender class - Создает узел ROS2 и передает его в класс low_level_cmd_sender
    rclcpp::spin(node);                                   // Run ROS2 node - Запускает узел ROS2 (начинает обработку сообщений и таймеров)
    rclcpp::shutdown();                                   // Exit - Завершает работу ROS2
    return 0; //Выход из программы
}