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
find_package(rosgraph_msgs REQUIRED)

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
target_include_directories(unitree_mujoco PRIVATE include)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})
ament_target_dependencies(unitree_mujoco rclcpp nav_msgs geometry_msgs tf2_ros std_msgs sensor_msgs std_srvs rosgraph_msgs unitree_go)

add_executable(stage4d_arm_shadow_checker src/stage4d_arm_shadow_checker.cc)
target_link_libraries(stage4d_arm_shadow_checker mujoco::mujoco)
ament_target_dependencies(stage4d_arm_shadow_checker rclcpp nav_msgs sensor_msgs std_msgs)

add_executable(test_unitree_sdk2 test/test_unitree_sdk2.cpp)
target_link_libraries(test_unitree_sdk2 unitree_sdk2)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

set(mujoco_dir "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(unitree_mujoco PRIVATE "mujoco_dir=\"${mujoco_dir}\"")
target_compile_definitions(stage4d_arm_shadow_checker PRIVATE "mujoco_dir=\"${mujoco_dir}\"")

install(TARGETS
    unitree_mujoco
    stage4d_arm_shadow_checker
    DESTINATION lib/${PROJECT_NAME}
)

set(CMAKE_BUILD_TYPE Release)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/go2_rars01/generated/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/go2_rars01/assets/ DESTINATION share/${PROJECT_NAME}/assets)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/go2/assets/ DESTINATION share/${PROJECT_NAME}/assets/go2)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/go2_rars01/scene_stage4d.xml DESTINATION share/${PROJECT_NAME}/scene)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/rars01_arm_sim.yaml DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_arm_release_hold test/test_arm_release_hold.cpp)
  target_include_directories(test_arm_release_hold PRIVATE include)
  add_test(NAME arm_release_hold COMMAND test_arm_release_hold)
  add_executable(test_virtual_payload test/test_virtual_payload.cpp)
  target_include_directories(test_virtual_payload PRIVATE include)
  target_link_libraries(test_virtual_payload mujoco::mujoco)
  add_test(NAME virtual_payload COMMAND test_virtual_payload
    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/go2_rars01/scene_stage4d.xml)
  add_executable(test_go2_torque_hud test/test_go2_torque_hud.cpp)
  target_include_directories(test_go2_torque_hud PRIVATE include)
  target_link_libraries(test_go2_torque_hud mujoco::mujoco)
  add_test(NAME go2_torque_hud COMMAND test_go2_torque_hud
    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/go2_rars01/scene_stage4d.xml)
endif()

ament_package()
