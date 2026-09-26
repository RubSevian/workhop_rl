cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
    src/unitree_sdk2_bridge/*.cc)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2)



add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})
ament_target_dependencies(unitree_mujoco rclcpp nav_msgs geometry_msgs tf2_ros std_msgs sensor_msgs std_srvs unitree_go)

add_executable(test_unitree_sdk2 test/test_unitree_sdk2.cpp)
target_link_libraries(test_unitree_sdk2 unitree_sdk2)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

set(mujoco_dir "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(unitree_mujoco PRIVATE "mujoco_dir=\"${mujoco_dir}\"")

install(TARGETS
    unitree_mujoco
    DESTINATION lib/${PROJECT_NAME}
)

set(CMAKE_BUILD_TYPE Release)
ament_package()
