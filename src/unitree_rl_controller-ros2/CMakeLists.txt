cmake_minimum_required(VERSION 3.5)
project(unitree_rl_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
find_package(yaml-cpp REQUIRED)  # ✅ Добавляем поддержку YAML

set(CMAKE_PREFIX_PATH "/home/ruben/Desktop/ros2_ws/src/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
)

# Создаём библиотеку
add_library(${PROJECT_NAME} SHARED src/rl_agent.cpp)

# Передаём CONFIG_PATH в компилятор
target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_PATH="\"${CMAKE_CURRENT_SOURCE_DIR}/config.yaml\"")

# Линкуем с библиотеками
target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES} yaml-cpp)  # ✅ Добавляем yaml-cpp

ament_target_dependencies(${PROJECT_NAME} std_msgs rclcpp)

ament_export_dependencies(rclcpp ros2_unitree_legged_msgs geometry_msgs std_msgs yaml-cpp)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

# Установка библиотеки в `lib/`
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Установка заголовочных файлов
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
