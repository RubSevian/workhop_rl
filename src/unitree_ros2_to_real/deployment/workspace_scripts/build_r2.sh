#!/usr/bin/env bash
set -eo pipefail
R2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$R2_ROOT/scripts/verify_r2.py"
# Reuse the clean Jazzy-only R1 build/install trees; no reference overlay.
bash "$R2_ROOT/scripts/build_r1.sh"
