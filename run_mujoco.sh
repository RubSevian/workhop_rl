#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_SDK_DDS_LIB_DIR="${UNITREE_SDK_DDS_LIB_DIR:-/usr/local/lib}"

if [ ! -f "$UNITREE_SDK_DDS_LIB_DIR/libddsc.so" ] || [ ! -f "$UNITREE_SDK_DDS_LIB_DIR/libddscxx.so" ]; then
    echo "Unitree SDK CycloneDDS libraries were not found in $UNITREE_SDK_DDS_LIB_DIR." >&2
    echo "Set UNITREE_SDK_DDS_LIB_DIR to the directory containing libddsc.so and libddscxx.so." >&2
    exit 1
fi

source "$PROJECT_DIR/env_setup.sh"

# unitree_sdk2 requires libddsc and libddscxx from the same SDK release.
# Put its directory before the ROS CycloneDDS overlay to avoid mixing 0.10.2 and 0.10.5.
export LD_LIBRARY_PATH="$UNITREE_SDK_DDS_LIB_DIR:$LD_LIBRARY_PATH"

exec ros2 run unitree_mujoco unitree_mujoco "$@"
