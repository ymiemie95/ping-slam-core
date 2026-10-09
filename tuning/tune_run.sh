#!/usr/bin/env bash
# One fully-automated SLAM run: launch -> bag play -> drain -> save_map +
# dump_trajectory -> SIGINT -> archive artifacts.
# Usage: tune_run.sh <LABEL> <RATE> <DATASET>   (env SMOKE=<sec> truncates bag play)
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/common.sh"

LABEL=${1:?usage: tune_run.sh LABEL RATE DATASET}
RATE=${2:?rate}
DS=${3:?dataset}
SMOKE=${SMOKE:-0}

RUN="$WS_HOST/tuning_runs/$DS/$LABEL"
RUN_C="$WS/tuning_runs/$DS/$LABEL"
BAGDIR=$(bag_for "$DS")

if [[ -e "$RUN" ]]; then
  echo "[tune_run] $RUN already exists — refusing to overwrite" >&2
  exit 2
fi
mkdir -p "$RUN/config"
exec > >(tee "$RUN/harness.log") 2>&1
echo "[tune_run] label=$LABEL rate=$RATE dataset=$DS bag=$BAGDIR smoke=$SMOKE"

# ── 1. clean slate ──────────────────────────────────────────────
ros_cleanup
assert_quiet_domain
stash_stale

# ── 2. snapshot active configs ──────────────────────────────────
cp "$WS_HOST"/config/sonar_slam_backend_params.yaml \
   "$WS_HOST"/config/sonar_to_laserscan_params.yaml \
   "$WS_HOST"/config/slamSettings.yaml "$RUN/config/"

# ── 3. detached launch, ts-stamped log on the shared mount ──────
docker exec "$C" bash -c "export ROS_DOMAIN_ID=$TUNE_DOMAIN && cd $WS && $SRC && if command -v ts >/dev/null 2>&1; then setsid bash -c 'stdbuf -oL -eL ros2 launch launch/full_slam_run.py 2>&1 | ts \"%.s\" > $RUN_C/run.log'; else setsid bash -c 'stdbuf -oL -eL ros2 launch launch/full_slam_run.py > $RUN_C/run.log 2>&1'; fi" </dev/null >/dev/null 2>&1 &

# ── 4. readiness: dump service + traj recorder both up ──────────
ready=0
for _ in $(seq 1 45); do
  if dros "ros2 service list 2>/dev/null" | grep -q '/sonar_slam_backend/dump_trajectory' \
     && dros "ros2 node list 2>/dev/null" | grep -q 'sonar_slam_backend_traj_recorder'; then
    ready=1; break
  fi
  sleep 2
done
if [[ $ready -ne 1 ]]; then
  echo "[tune_run] READINESS TIMEOUT — run.log tail:"; tail -30 "$RUN/run.log" || true
  ros_cleanup || true
  exit 1
fi
sleep 5   # discovery + /tf_static latch
echo "[tune_run] pipeline ready"

# ── 5. play the bag (foreground; returns when done) ─────────────
t0=$(date +%s)
if [[ $SMOKE -gt 0 ]]; then
  dros "timeout -k 5 $SMOKE ros2 bag play $BAGDIR -r $RATE" > "$RUN/bagplay.log" 2>&1 || true
else
  dros "ros2 bag play $BAGDIR -r $RATE" > "$RUN/bagplay.log" 2>&1
fi
t1=$(date +%s)
BAG_WALL=$((t1 - t0))
echo "[tune_run] bag play finished in ${BAG_WALL}s"

# ── 6. quiescence: SEQ:Process count stable 3x10s (cap 300s) ────
sleep 15
prev=-1; stable=0; waited=15
while [[ $stable -lt 3 && $waited -lt 300 ]]; do
  cur=$(grep -c 'SEQ:Process' "$RUN/run.log" || true)
  if [[ "$cur" == "$prev" ]]; then stable=$((stable + 1)); else stable=0; fi
  prev=$cur
  sleep 10; waited=$((waited + 10))
done
echo "[tune_run] quiescent after ${waited}s (SEQ lines=$prev)"

# ── 7. export map + optimized trajectory ────────────────────────
# map_io's ImageMagick intermittently SIGSEGVs mid-write (node survives) —
# retry until the .pgm actually lands
for attempt in 1 2 3; do
  dros "timeout 60 ros2 service call /sonar_slam_backend/save_map sonar_slam_backend/srv/SaveMap '{name: {data: \"map_$LABEL\"}}'" | tail -2 || true
  sleep 3
  [[ -s "$WS_HOST/map_$LABEL.pgm" ]] && break
  echo "[tune_run] save_map attempt $attempt: no map file — retrying"
done
if [[ ! -s "$WS_HOST/map_$LABEL.pgm" ]]; then
  echo "[tune_run] save_map dead — falling back to /map topic dump (no ImageMagick)"
  dros "timeout 40 python3 tuning/map_topic_dump.py map_$LABEL" || true
fi
[[ -s "$WS_HOST/map_$LABEL.pgm" ]] || echo "[tune_run] WARN map export failed (save_map + topic dump)"
dros "timeout 60 ros2 service call /sonar_slam_backend/dump_trajectory std_srvs/srv/Trigger" | tail -2 || echo "[tune_run] WARN dump_trajectory failed"
if [[ ! -s "$WS_HOST/pingslamTraj_optimized.txt" ]]; then
  echo "[tune_run] WARN optimized trajectory missing or empty"
fi

# ── 8. orderly shutdown: ONE SIGINT to ros2 launch only ─────────
dex "pkill -INT -f 'bin/ros2 laun[c]h'" || true
RUNG=INT
alive=""
for _ in $(seq 1 15); do
  sleep 2
  alive=$(dex "pgrep -f '$PROCPAT'" 2>/dev/null || true)
  [[ -z "$alive" ]] && break
done
if [[ -n "$alive" ]]; then
  RUNG=TERM
  dex "pkill -TERM -f '$PROCPAT'" || true
  sleep 10
  alive=$(dex "pgrep -f '$PROCPAT'" 2>/dev/null || true)
fi
if [[ -n "$alive" ]]; then
  RUNG=KILL
  dex "pkill -KILL -f '$PROCPAT'" || true
  sleep 3
fi
echo "[tune_run] shutdown rung: $RUNG"

# ── 9. archive artifacts ────────────────────────────────────────
if [[ -s "$WS_HOST/pingslamTraj_optimized.txt" ]]; then
  mv "$WS_HOST/pingslamTraj_optimized.txt" "$RUN/pingslamTraj_optimized_$LABEL.txt"
fi
for ext in pgm yaml; do
  if [[ -e "$WS_HOST/map_$LABEL.$ext" ]]; then
    mv "$WS_HOST/map_$LABEL.$ext" "$RUN/"
  fi
done
if [[ -e "$WS_HOST/scripts/pingslamTraj_.txt" ]]; then
  mv "$WS_HOST/scripts/pingslamTraj_.txt" "$RUN/pingslamTraj_dense_$LABEL.txt"
fi

{
  echo "label=$LABEL rate=$RATE dataset=$DS smoke=$SMOKE"
  echo "bag_wall_s=$BAG_WALL shutdown_rung=$RUNG"
} > "$RUN/run_report.txt"
"$DIR/log_stats.sh" "$RUN" >> "$RUN/run_report.txt" || true

echo "[tune_run] done -> $RUN"
