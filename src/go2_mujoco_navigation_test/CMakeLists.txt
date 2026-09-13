cmake_minimum_required(VERSION 3.16)
project(go2_mujoco_navigation_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(mujoco_ground_truth_odom src/mujoco_ground_truth_odom.cpp)
ament_target_dependencies(mujoco_ground_truth_odom rclcpp nav_msgs unitree_go)

add_executable(test_path_publisher src/test_path_publisher.cpp)
ament_target_dependencies(test_path_publisher rclcpp nav_msgs geometry_msgs std_msgs)

install(TARGETS mujoco_ground_truth_odom test_path_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
