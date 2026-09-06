cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)

# The native JetPack 7 PyTorch 2.13 CMake package requires C++20.  SDK2 is
# C++17-compatible and builds correctly as a dependency with this standard.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


# set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(unitree_rl_controller REQUIRED)
# Prefer an SDK installed on the robot.  For a fresh Jetson build, use the
# vendored SDK2 source and its aarch64 libraries without an extra install step.
find_package(unitree_sdk2 QUIET)
if(NOT TARGET unitree_sdk2)
  set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build Unitree SDK examples" FORCE)
  add_subdirectory(library/unitree_sdk2 EXCLUDE_FROM_ALL)
endif()
option(BUILD_LEGACY_SIM "Build MuJoCo/LCM simulation executables" OFF)
if(BUILD_LEGACY_SIM)
  find_package(glfw3 REQUIRED)
endif()

# Set Torch_DIR (or CMAKE_PREFIX_PATH) externally to the Jetson/aarch64
# libtorch installation.  Keeping the path out of the source tree makes the
# package build on both the development PC and the robot.
if(DEFINED ENV{Torch_DIR} AND NOT Torch_DIR)
  set(Torch_DIR $ENV{Torch_DIR})
endif()
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
  # Jetson Orin Nano is compute capability 8.7.
  set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "CUDA architectures" FORCE)
endif()
if(EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler" FORCE)
endif()
find_package(Torch REQUIRED)

include_directories(
 include
 ${TORCH_INCLUDE_DIRS}
)

find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

set (
  DEPENDENCY_LIST
  unitree_go
  rclcpp
  std_msgs
  tf2
  tf2_ros
)


message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(LEGACY_SIM_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/unitree_legged_sdk/include
)

install(
 DIRECTORY ${PROJECT_SOURCE_DIR}/unitree_legged_sdk/lib/
 DESTINATION lib
)

link_directories(${PROJECT_SOURCE_DIR}/unitree_legged_sdk/lib)

# Unitree Go2
include_directories(library/unitree_sdk2/include)
include_directories(library/unitree_sdk2/thirdparty/include)
include_directories(library/unitree_sdk2/thirdparty/include/ddscxx)
# The SDK target carries its platform-specific DDS dependencies transitively.
set(UNITREE_GO2_LIBS unitree_sdk2)
file(GLOB GLOB_UNITREE_SDK2 "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/lib/${CMAKE_SYSTEM_PROCESSOR}/*")
file(GLOB GLOB_UNITREE_SDK2_THIRDPARTY "${PROJECT_SOURCE_DIR}/library/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/*")
install(FILES
  ${GLOB_UNITREE_SDK2}
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  ${GLOB_UNITREE_SDK2_THIRDPARTY}
  DESTINATION lib/
)

if(BUILD_LEGACY_SIM)
  add_executable(lcm_server unitree_legged_sdk/examples/lcm_server.cpp)
  target_link_libraries(lcm_server ${LEGACY_SIM_LIBS})

  add_executable(mujoco_sim src/mujoco_sim.cpp src/motor_crc.cpp)
  target_link_libraries(mujoco_sim ${UNITREE_GO2_LIBS} ${LEGACY_SIM_LIBS} ${TORCH_LIBRARIES} glfw)
  ament_target_dependencies(mujoco_sim rclcpp unitree_rl_controller sensor_msgs ${DEPENDENCY_LIST})
endif()

add_executable(go2_heightmap_node
  src/go2_heightmap_node.cpp
)

ament_target_dependencies(go2_heightmap_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)


add_executable(ros2_rl_go2 src/ros2_rl_go2.cpp src/motor_crc.cpp)
target_link_libraries(ros2_rl_go2 ${UNITREE_GO2_LIBS} ${TORCH_LIBRARIES})
ament_target_dependencies(ros2_rl_go2 rclcpp geometry_msgs unitree_rl_controller sensor_msgs ${DEPENDENCY_LIST})
set_target_properties(ros2_rl_go2 PROPERTIES
  BUILD_RPATH "${TORCH_INSTALL_PREFIX}/lib"
  INSTALL_RPATH "${TORCH_INSTALL_PREFIX}/lib")

set(CONFIG_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(ros2_rl_go2 PRIVATE "CONFIG_BASE_DIR=\"${CONFIG_BASE_DIR}\"")

if(BUILD_LEGACY_SIM)
  set(CONFIG_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(mujoco_sim PRIVATE "CONFIG_BASE_DIR=\"${CONFIG_BASE_DIR}\"")
endif()


install(TARGETS
    ros2_rl_go2 
    go2_heightmap_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_LEGACY_SIM)
  install(TARGETS mujoco_sim lcm_server DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
