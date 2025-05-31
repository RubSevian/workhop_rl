cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

# add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


# set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(unitree_rl_controller REQUIRED)
find_package(unitree_sdk2  REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/home/ruben/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
find_library( TORCH_LIBRARIES libtorch_cpu.so libc10.so:
              PATHS
              /home/ruben/libtorch/lib/
              NO_DEFAULT_PATH )

include_directories(
 include
 ${TORCH_INCLUDE_DIRS}
)

find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set (
  DEPENDENCY_LIST
  unitree_go
  rclcpp
  std_msgs
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

# Unitree Go2
include_directories(library/unitree_sdk2/include)
link_directories(library/unitree_sdk2/lib/x86_64)
include_directories(library/unitree_sdk2/thirdparty/include)
include_directories(library/unitree_sdk2/thirdparty/include/ddscxx)
link_directories(library/unitree_sdk2/thirdparty/lib/x86_64)
set(UNITREE_GO2_LIBS -pthread unitree_sdk2 ddsc ddscxx)
file(GLOB GLOB_UNITREE_SDK2 "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/lib/x86_64/*.so")
file(GLOB GLOB_UNITREE_SDK2_THIRDPARTY "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/thirdparty/lib/x86_64/*.so")
install(FILES
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  DESTINATION lib/
)

add_executable(lcm_server unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})

add_executable(ros2_rl src/ros2_rl.cpp)
target_link_libraries(ros2_rl ${UNITREE_GO2_LIBS} ${EXTRA_LIBS} ${TORCH_LIBRARIES})
ament_target_dependencies(ros2_rl rclcpp ros2_unitree_legged_msgs unitree_rl_controller sensor_msgs)


add_executable(mujoco_sim src/mujoco_sim.cpp src/motor_crc.cpp)
target_link_libraries(ros2_rl ${UNITREE_GO2_LIBS} ${EXTRA_LIBS} ${TORCH_LIBRARIES} )
ament_target_dependencies(mujoco_sim rclcpp ros2_unitree_legged_msgs unitree_rl_controller sensor_msgs ${DEPENDENCY_LIST})

add_executable(ros2_rl_go2 src/ros2_rl_go2.cpp)
target_link_libraries(ros2_rl_go2 ${UNITREE_GO2_LIBS} ${EXTRA_LIBS} ${TORCH_LIBRARIES})
ament_target_dependencies(ros2_rl_go2 rclcpp unitree_rl_controller sensor_msgs)

set(CONFIG_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(ros2_rl PRIVATE "CONFIG_BASE_DIR=\"${CONFIG_BASE_DIR}\"")




install(TARGETS
    ros2_rl
    mujoco_sim
    ros2_rl_go2
    lcm_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()




