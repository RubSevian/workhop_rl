#!/bin/bash
if command -v conda >/dev/null 2>&1; then
    conda deactivate >/dev/null 2>&1 || true
fi

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TORCH_LIB_DIR="${TORCH_LIB_DIR:-/home/ruben/libtorch/lib}"

echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1

if [ ! -f "$PROJECT_DIR/install/setup.bash" ]; then
    echo "Workspace is not built: run 'colcon build' in $PROJECT_DIR first." >&2
    return 1 2>/dev/null || exit 1
fi

source "$PROJECT_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH="$TORCH_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
