cmake_minimum_required(VERSION 3.5)
project(unitree_rl_controller)

# Installed native Torch requires C++20.
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

if(DEFINED ENV{Torch_DIR} AND NOT Torch_DIR)
  set(Torch_DIR "$ENV{Torch_DIR}")
endif()
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
)

# Создаём библиотеку
add_library(${PROJECT_NAME} SHARED
  src/rl_agent.cpp
  src/observation_buffer.cpp
  src/unified_observation_contract.cpp)
# Линкуем с библиотеками
target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES} yaml-cpp)  

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

if(BUILD_TESTING)
  enable_testing()
  add_executable(unified_observation_contract_test
    tests/unified_observation_contract_test.cpp)
  target_link_libraries(unified_observation_contract_test ${PROJECT_NAME} ${TORCH_LIBRARIES})
  add_test(NAME unified_observation_contract_test
    COMMAND unified_observation_contract_test)

  add_executable(agent_unified_reset_test
    tests/agent_unified_reset_test.cpp)
  target_link_libraries(agent_unified_reset_test ${PROJECT_NAME} ${TORCH_LIBRARIES})
  add_test(NAME agent_unified_reset_test
    COMMAND agent_unified_reset_test)

  add_executable(agent_config_loading_test
    tests/agent_config_loading_test.cpp)
  target_compile_definitions(agent_config_loading_test PRIVATE
    UNITREE_RL_CONTROLLER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(agent_config_loading_test ${PROJECT_NAME} ${TORCH_LIBRARIES})
  add_test(NAME agent_config_loading_test
    COMMAND agent_config_loading_test)
endif()

ament_package()
