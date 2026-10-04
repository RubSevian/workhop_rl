#!/usr/bin/env bash
set -eo pipefail
: "${SIM2REAL_ROOT:?Set SIM2REAL_ROOT}"
: "${RARS_SDK_CONFIG_PATH:?Set the existing SDK configuration}"
: "${RARS_OWNER_LOCK_DIRECTORY:?Set shared owner lock directory}"
: "${RARS_OWNER_JOURNAL:?Set persistent owner journal}"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
  echo "Jazzy runtime required" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$SIM2REAL_ROOT/install_r1/setup.bash"
RARS_OWNER_ARGS=(--ros-args -p "sdk_config_path:=$RARS_SDK_CONFIG_PATH"
  -p "config_path:=$SIM2REAL_ROOT/install_r1/unitree_legged_real/share/unitree_legged_real/config/go2_rars01_real.yaml"
  -p connect_serial:=true -p read_only:=false
  -p "lock_directory:=$RARS_OWNER_LOCK_DIRECTORY" -p "journal_path:=$RARS_OWNER_JOURNAL")
if [[ -n "${RARS_DEVICE_PATH:-}" ]]; then
  RARS_OWNER_ARGS+=(-p "device_path:=$RARS_DEVICE_PATH")
fi
exec "$SIM2REAL_ROOT/install_r1/unitree_legged_real/lib/unitree_legged_real/rars_r3_owner" "${RARS_OWNER_ARGS[@]}"
