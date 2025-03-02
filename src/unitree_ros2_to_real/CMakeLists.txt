cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(unitree_rl_controller REQUIRED)


list(APPEND CMAKE_PREFIX_PATH "/home/ruben/workhop_rl/src/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
find_library( TORCH_LIBRARIES libtorch_cpu.so libc10.so:
              PATHS
              /home/ruben/workhop_rl/src/libtorch/lib/
              NO_DEFAULT_PATH )
## set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(
 include
 ${TORCH_INCLUDE_DIRS}
)


message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(CMAKE_CXX_FLAGS "-O3")


include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

install(
 DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
 DESTINATION lib
)


link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib)

## add_executable(ros2_udp src/ros2_udp.cpp)
## target_link_libraries(ros2_udp ${EXTRA_LIBS})
## ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(lcm_server unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_test src/ros2_test.cpp)
target_link_libraries(ros2_test ${EXTRA_LIBS})
ament_target_dependencies(ros2_test rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_rl src/ros2_rl.cpp)
target_link_libraries(ros2_rl ${EXTRA_LIBS} ${TORCH_LIBRARIES})
ament_target_dependencies(ros2_rl rclcpp ros2_unitree_legged_msgs unitree_rl_controller sensor_msgs)



install(TARGETS
    ros2_walk_example
    ros2_position_example
    ros2_rl
    ros2_test
    lcm_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()




