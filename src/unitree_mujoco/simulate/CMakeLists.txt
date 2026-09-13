cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
# The simulator UI is from MuJoCo 3.3.1.  Ubuntu's `libmujoco-dev` is 2.2.x
# and lacks the scene-state API used by this source tree.
if(DEFINED ENV{MUJOCO_ROOT} AND NOT "$ENV{MUJOCO_ROOT}" STREQUAL "")
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "$ENV{MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
  find_library(MUJOCO_LIBRARY mujoco PATHS "$ENV{MUJOCO_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
  add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
else()
  find_package(mujoco REQUIRED)
endif()
find_package(ament_cmake REQUIRED)

# The navigation workspace vendors the exact SDK2 DDS ABI used by Go2.  This
# keeps MuJoCo and ros2_rl_go2 on one SDK release without a system-wide install.
find_package(unitree_sdk2 QUIET)
if(NOT TARGET unitree_sdk2)
  set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build SDK examples" FORCE)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_ros2_to_real/library/unitree_sdk2
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2)
endif()

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
ament_target_dependencies(unitree_mujoco ${DEPENDENCY_LIST})

add_executable(test_unitree_sdk2 test/test_unitree_sdk2.cpp)
target_link_libraries(test_unitree_sdk2 unitree_sdk2)

# jstest is a standalone legacy joystick diagnostic and is not part of the
# simulator motor loop.  Do not compile it for the headless navigation test.

set(mujoco_dir "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(unitree_mujoco PRIVATE "mujoco_dir=\"${mujoco_dir}\"")

install(TARGETS
    unitree_mujoco
    DESTINATION lib/${PROJECT_NAME}
)

set(CMAKE_BUILD_TYPE Release)
ament_package()
