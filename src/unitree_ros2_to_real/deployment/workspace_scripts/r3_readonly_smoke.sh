#!/usr/bin/env bash
set -eo pipefail
R3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$R3_ROOT/repos/workhop_rl/jazzy_setup.sh"
export ROS_DOMAIN_ID=223
export ROS_LOG_DIR="$R3_ROOT/runtime/ros_logs"
export ROS_HOME="$R3_ROOT/runtime/ros_home"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
python3 "$R3_ROOT/scripts/r3_readonly_smoke.py"
