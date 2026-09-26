#!/usr/bin/env bash
set -u
set -o pipefail
SCRIPT_PATH=$(readlink -f "${BASH_SOURCE[0]}")
WORKSHOP_ROOT=$(cd "$(dirname "$SCRIPT_PATH")/../../.." && pwd)
cd "$WORKSHOP_ROOT"
source /opt/ros/humble/setup.bash
source ../autonomy_nav_go2/install/setup.bash
source install/setup.bash
WORKSPACE_ROOT=$(cd ../.. && pwd)
RUN_DIR=${STAGE4D_RUN_DIR:-"$WORKSPACE_ROOT/stage4d_runs/$(date +%Y%m%d_%H%M%S)"}
CASE_SECONDS=${STAGE4D_CASE_SECONDS:-25}
RVIZ=${STAGE4D_RVIZ:-true}
mkdir -p "$RUN_DIR"
LOG="$RUN_DIR/stage4d_launch.log"
TRACE="$RUN_DIR/stage4d_trace.json"

echo "Starting Stage4D (RViz=$RVIZ), output=$RUN_DIR"
timeout --signal=INT --kill-after=10s 240s ros2 launch unitree_legged_real stage4d_full_navigation.launch.py \
  rl_config_path:="$PWD/src/unitree_ros2_to_real/config/go2_rars01_unified.yaml" \
  policy_path:="$WORKSPACE_ROOT/weights/policy_1.pt" \
  mujoco_config:=config_go2_rars01_stage4d.yaml rviz:="$RVIZ" report_path:="$TRACE" >"$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() { kill -INT "$LAUNCH_PID" 2>/dev/null || true; wait "$LAUNCH_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

READY=0
for _ in $(seq 1 90); do
  if timeout 2s ros2 topic echo /stage4d/navigation_ready --once 2>/dev/null | grep -q 'data: true'; then READY=1; break; fi
  sleep 1
done
if [[ "$READY" != 1 ]]; then
  echo "Stage4D readiness timeout; inspect $LOG" >&2
  exit 2
fi

echo "Stage4D ready; running D0-D4"
for CASE in D0 D1 D2 D3 D4A D4B; do
  echo "[$CASE] sending goal"
  ros2 run unitree_legged_real stage4d_send_goal.py --case "$CASE" --duration 1 >/dev/null
  sleep "$CASE_SECONDS"
  ros2 service call /stage4d/save_report std_srvs/srv/Trigger '{}' >"$RUN_DIR/${CASE}_save.txt" 2>&1 || true
  [[ -f "$TRACE" ]] && cp "$TRACE" "$RUN_DIR/${CASE}.json"
done

python3 - "$RUN_DIR" <<'PY'
import json, pathlib, sys
run=pathlib.Path(sys.argv[1]); out={'run_dir':str(run),'cases':{}}
for c in ('D0','D1','D2','D3','D4A','D4B'):
 p=run/f'{c}.json'; out['cases'][c]=json.loads(p.read_text()) if p.exists() else {'status':'missing_report'}
(run/'stage4d_summary.json').write_text(json.dumps(out,indent=2)+'\n')
print(f'Summary: {run/"stage4d_summary.json"}')
PY
