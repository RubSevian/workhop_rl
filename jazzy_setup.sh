#!/usr/bin/env bash
# Source from a clean shell; only the R1 Jazzy overlay is admitted.
R1_WORKHOP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R1_ROOT_DIR="$(cd "$R1_WORKHOP_DIR/../.." && pwd)"
case "${AMENT_PREFIX_PATH:-}:${CMAKE_PREFIX_PATH:-}:${COLCON_PREFIX_PATH:-}:${LD_LIBRARY_PATH:-}:${PYTHONPATH:-}:${PATH:-}" in
  *sim2sim_reference*|*humble*|*cyclonedds_ws/install*)
    echo "Refusing mixed reference/Humble environment; use a fresh shell." >&2
    return 1 2>/dev/null || exit 1 ;;
esac
if [ -z "${Torch_DIR:-}" ] || [ ! -f "$Torch_DIR/TorchConfig.cmake" ]; then
  echo "Set Torch_DIR explicitly to the native Torch CMake directory." >&2
  return 1 2>/dev/null || exit 1
fi
# Reject unexpected ROS/CMake overlays even if their path does not say Humble.
R1_PREFIX_REJECTED=0
for R1_PREFIX_LIST in "${AMENT_PREFIX_PATH:-}" "${CMAKE_PREFIX_PATH:-}" "${COLCON_PREFIX_PATH:-}"; do
  IFS=: read -r -a R1_PREFIX_ENTRIES <<< "$R1_PREFIX_LIST"
  for R1_PREFIX_ENTRY in "${R1_PREFIX_ENTRIES[@]}"; do
    case "$R1_PREFIX_ENTRY" in
      ""|/opt/ros/jazzy|"$R1_ROOT_DIR/install_r1"|"$R1_ROOT_DIR/install_r1/"*|"$Torch_DIR") ;;
      *) echo "Unexpected overlay: $R1_PREFIX_ENTRY; use a clean shell." >&2; R1_PREFIX_REJECTED=1 ;;
    esac
  done
done
if [ "$R1_PREFIX_REJECTED" = 1 ]; then
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
export TORCH_CUDA_ARCH_LIST=8.7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
R1_TORCH_PREFIX="$(cd "$Torch_DIR/../../.." && pwd)"
export LD_LIBRARY_PATH="$R1_TORCH_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ -f "$R1_ROOT_DIR/install_r1/local_setup.bash" ]; then
  source "$R1_ROOT_DIR/install_r1/local_setup.bash"
fi
# No DDS robot NIC configuration or arming is performed by environment setup.
