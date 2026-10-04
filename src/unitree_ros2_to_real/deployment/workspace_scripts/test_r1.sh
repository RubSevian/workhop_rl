#!/usr/bin/env bash
set -eo pipefail
R1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$R1_ROOT/repos/workhop_rl/jazzy_setup.sh"
cd "$R1_ROOT"
python3 scripts/verify_r1.py
colcon --log-base "$R1_ROOT/log_r1_test" test \
  --build-base "$R1_ROOT/build_r1" --install-base "$R1_ROOT/install_r1" \
  --base-paths repos/workhop_rl/src/unitree_rl_controller-ros2 repos/workhop_rl/src/unitree_ros2_to_real \
  --packages-select unitree_rl_controller unitree_legged_real \
  --executor sequential --event-handlers console_direct+ \
  --ctest-args -V
colcon test-result --test-result-base "$R1_ROOT/build_r1" --verbose
python3 repos/workhop_rl/tests/reference_unified_observation.py \
  --cpp-test "$R1_ROOT/build_r1/unitree_rl_controller/unified_observation_contract_test"
