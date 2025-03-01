cmake_minimum_required(VERSION 3.5)
project(unitree_rl_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

set(CMAKE_CXX_FLAGS "-O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

## set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")

set(CMAKE_PREFIX_PATH ./libtorch)
list(APPEND CMAKE_PREFIX_PATH "/home/ruben/Desktop/ros2_ws/src/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

 include_directories(
  include
 ${TORCH_INCLUDE_DIRS}
)


link_directories(${CMAKE_SOURCE_DIR}/src/rl_agent.cpp)

add_library(${PROJECT_NAME} src/rl_agent.cpp)
## target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} 
  std_msgs rclcpp 
 )




ament_export_dependencies(rclcpp ros2_unitree_legged_msgs geometry_msgs std_msgs)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(unitree_rl_controller)

install(TARGETS unitree_rl_controller)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)



ament_package()
