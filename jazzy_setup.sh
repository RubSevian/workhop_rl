#!/usr/bin/env bash

# Source this script before building or launching the native JetPack 7 stack.
JAZZY_WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JAZZY_NAV_WORKSPACE_DIR="$(cd "${JAZZY_WORKSPACE_DIR}/../autonomy_nav_go2" 2>/dev/null && pwd)"
JAZZY_TORCH_PREFIX="/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export Torch_DIR="${JAZZY_TORCH_PREFIX}/share/cmake/Torch"
export LD_LIBRARY_PATH="${JAZZY_TORCH_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# Set GO2_NETWORK_INTERFACE to the Ethernet interface connected to the robot,
# for example: export GO2_NETWORK_INTERFACE=enP8p1s0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -n "${GO2_NETWORK_INTERFACE:-}" ]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${GO2_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
fi

if [ -f "${JAZZY_NAV_WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${JAZZY_NAV_WORKSPACE_DIR}/install/setup.bash"
fi

if [ -f "${JAZZY_WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${JAZZY_WORKSPACE_DIR}/install/setup.bash"
fi
