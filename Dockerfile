# syntax=docker/dockerfile:1
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# Базовые инструменты
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common \
 && add-apt-repository universe \
 && apt-get update

# Пакеты из README (без MuJoCo)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglfw3-dev \
    libyaml-cpp-dev liblcm-dev \
    ros-humble-std-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

# Подготовка workspace (пустой, код будет монтироваться)
# WORKDIR /ws
# RUN mkdir -p src

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "cd /ws" >> /root/.bashrc

CMD ["bash"]
