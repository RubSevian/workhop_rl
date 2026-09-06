cmake_minimum_required(VERSION 3.5)
project(unitree_rl_controller)

# PyTorch 2.13 distributed with the JetPack 7 environment requires C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

set(CMAKE_CXX_FLAGS "-O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)  

# Torch_DIR points to a native aarch64 LibTorch installation.  The GraspNet
# virtualenv already contains one at torch/share/cmake/Torch.
if(DEFINED ENV{Torch_DIR} AND NOT Torch_DIR)
  set(Torch_DIR $ENV{Torch_DIR})
endif()
# Jetson Orin is Ampere SM 8.7.  PyTorch's CUDA CMake module rejects an empty
# cached architecture value on CMake 3.28, so make the native target explicit.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "CUDA architectures" FORCE)
endif()
if(EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler" FORCE)
endif()
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
)

# Создаём библиотеку
add_library(${PROJECT_NAME} SHARED src/rl_agent.cpp src/observation_buffer.cpp)
# Линкуем с библиотеками
target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES} yaml-cpp)  
# Keep the native CUDA LibTorch location available to the installed ROS node.
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${TORCH_INSTALL_PREFIX}/lib"
  INSTALL_RPATH "${TORCH_INSTALL_PREFIX}/lib")

ament_target_dependencies(${PROJECT_NAME} std_msgs rclcpp)

ament_export_dependencies(rclcpp ros2_unitree_legged_msgs geometry_msgs std_msgs yaml-cpp)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Установка библиотеки в `lib/`
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Установка заголовочных файлов
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
