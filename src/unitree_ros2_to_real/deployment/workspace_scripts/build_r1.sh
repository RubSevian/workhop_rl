#!/usr/bin/env bash
set -eo pipefail
R1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$R1_ROOT/repos/workhop_rl/jazzy_setup.sh"
set -u
cd "$R1_ROOT"
python3 "$R1_ROOT/scripts/verify_r1.py"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
# Explicit package roots prevent discovery of unrelated nav/sim/SDK projects.
colcon --log-base "$R1_ROOT/log_r1" build \
  --base-paths \
  repos/autonomy_nav_go2/src/utilities/unitree_pkgs/unitree_go \
  repos/autonomy_nav_go2/src/utilities/unitree_pkgs/unitree_api \
  repos/workhop_rl/src/ros2_unitree_legged_msgs \
  repos/workhop_rl/src/unitree_rl_controller-ros2 \
  repos/workhop_rl/src/unitree_ros2_to_real \
  --build-base "$R1_ROOT/build_r1" --install-base "$R1_ROOT/install_r1" \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_LEGACY_SIM=OFF \
  -DCMAKE_CUDA_COMPILER="${CUDACXX:-/usr/local/cuda/bin/nvcc}" -DCMAKE_CUDA_ARCHITECTURES=87 -DTORCH_CUDA_ARCH_LIST=8.7 \
  -DR1_POLICY_PATH="$R1_ROOT/weights/policy_2.pt" \
  -DR1_CONFIG_PATH="$R1_ROOT/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml"
