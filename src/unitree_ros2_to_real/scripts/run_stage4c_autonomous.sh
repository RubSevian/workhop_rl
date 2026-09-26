#!/usr/bin/env bash

SCRIPT_PATH=$(readlink -f "${BASH_SOURCE[0]}")
WORKSHOP_ROOT=$(cd "$(dirname "$SCRIPT_PATH")/../../.." && pwd)
cd "$WORKSHOP_ROOT"
source /opt/ros/humble/setup.bash
source ../autonomy_nav_go2/install/setup.bash
source install/setup.bash
set -u
WORKSPACE_ROOT=$(cd ../.. && pwd)
RUN_DIR=${STAGE4C_RUN_DIR:-"$WORKSPACE_ROOT/stage4c_runs/$(date +%Y%m%d_%H%M%S)"}
mkdir -p "$RUN_DIR"

for CASE in C0 C1 C2; do
  case "$CASE" in
    C0) GX=1.0; GY=0.0 ;;
    C1) GX=0.7; GY=0.5 ;;
    C2) GX=0.8; GY=-0.3 ;;
  esac
  REPORT="$RUN_DIR/${CASE}.json"
  LOG="$RUN_DIR/${CASE}.log"
  echo "[$CASE] starting goal=($GX,$GY)"
  timeout --signal=INT --kill-after=8s 60s ros2 launch unitree_legged_real stage4c_pointlio_navigation.launch.py \
    rl_config_path:="$PWD/src/unitree_ros2_to_real/config/go2_rars01_unified.yaml" \
    policy_path:="$WORKSPACE_ROOT/weights/policy_1.pt" \
    mujoco_config:=config_go2_rars01_pointlio.yaml \
    goal_x:="$GX" goal_y:="$GY" auto_start_route:=true \
    report_path:="$REPORT" >"$LOG" 2>&1 &
  LAUNCH_PID=$!
  for _ in $(seq 1 55); do
    [[ -s "$REPORT" ]] && break
    sleep 1
  done
  if [[ -s "$REPORT" ]]; then
    echo "[$CASE] report written: $REPORT"
  else
    echo "[$CASE] timed out; inspect $LOG" >&2
  fi
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  sleep 2
 done

python3 - "$RUN_DIR" <<'PY'
import json, pathlib, sys
run_dir = pathlib.Path(sys.argv[1])
summary = {"run_dir": str(run_dir), "cases": {}}
for case in ("C0", "C1", "C2"):
    path = run_dir / f"{case}.json"
    if path.exists():
        summary["cases"][case] = json.loads(path.read_text())
    else:
        summary["cases"][case] = {"status": "missing_report"}
(run_dir / "stage4c_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(f"Summary: {run_dir / 'stage4c_summary.json'}")
PY
