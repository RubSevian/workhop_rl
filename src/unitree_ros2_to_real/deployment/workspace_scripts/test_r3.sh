#!/usr/bin/env bash
set -eo pipefail
R3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$R3_ROOT/scripts/verify_r3.py"
bash "$R3_ROOT/scripts/test_r2.sh"
