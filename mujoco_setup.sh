#!/usr/bin/env bash
# ROS 2 Jazzy environment shared by the SDK2 MuJoCo process and ROS nodes.
set -eo pipefail

MUJOCO_WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MUJOCO_ROOT="${MUJOCO_ROOT:-/opt/mujoco-3.3.1}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>}"

TORCH_LIB_DIR="${TORCH_LIB_DIR:-/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch/lib}"
SDK_DDS_LIB_DIR="${UNITREE_SDK_DDS_LIB_DIR:-${MUJOCO_WORKSPACE_DIR}/src/unitree_ros2_to_real/library/unitree_sdk2/thirdparty/lib/aarch64}"
export LD_LIBRARY_PATH="${MUJOCO_ROOT}/lib:${SDK_DDS_LIB_DIR}:${TORCH_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if [ -f "${MUJOCO_WORKSPACE_DIR}/../autonomy_nav_go2/install/setup.bash" ]; then
  source "${MUJOCO_WORKSPACE_DIR}/../autonomy_nav_go2/install/setup.bash"
fi
if [ -f "${MUJOCO_WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${MUJOCO_WORKSPACE_DIR}/install/setup.bash"
fi
