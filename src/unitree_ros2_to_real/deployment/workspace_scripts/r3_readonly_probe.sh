#!/usr/bin/env bash
set -eo pipefail
R3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$R3_ROOT/repos/workhop_rl/jazzy_setup.sh"
export ROS_LOG_DIR="$R3_ROOT/runtime/ros_logs"
export ROS_HOME="$R3_ROOT/runtime/ros_home"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# The R2 executable has no LowCmd publisher, SDK2 channel, or serial owner.
# Bounded observation only; timeout 124 is expected after the capture window.
timeout --signal=INT --kill-after=2s 10s ros2 run unitree_legged_real ros2_rl_go2 --ros-args \
  -p config_path:="$R3_ROOT/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml" \
  -p model_path:="$R3_ROOT/weights/policy_2.pt" \
  -p enable_actuator_output:=false
