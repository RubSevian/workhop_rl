#!/usr/bin/env bash
# Source this file to use the installed sim2real packages in a Bash terminal.
# Environment setup only: no robot RPC, serial access or controller startup.
SIM2REAL_SETUP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export Torch_DIR="${Torch_DIR:-/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch/share/cmake/Torch}"
source "$SIM2REAL_SETUP_ROOT/repos/workhop_rl/jazzy_setup.sh" || return 1
unset SIM2REAL_SETUP_ROOT
