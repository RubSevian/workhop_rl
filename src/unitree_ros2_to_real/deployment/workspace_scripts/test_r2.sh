#!/usr/bin/env bash
set -eo pipefail
R2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$R2_ROOT/scripts/verify_r2.py"
# Runs all R1 and R2 CTest entries plus the unchanged observation parity check.
bash "$R2_ROOT/scripts/test_r1.sh"
