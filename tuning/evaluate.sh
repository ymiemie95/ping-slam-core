#!/usr/bin/env bash
# evo_ape evaluation for one finished run; appends a row to the dataset's
# summary.csv and regenerates leaderboard.md.
# Usage: evaluate.sh <LABEL> <DATASET> [overrides-description]
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/common.sh"

LABEL=${1:?usage: evaluate.sh LABEL DATASET [overrides]}
DS=${2:?dataset}
OVR=${3:-default}
OVR=${OVR//,/;}   # keep the CSV parseable

RUN="$WS_HOST/tuning_runs/$DS/$LABEL"
TRAJ_C="tuning_runs/$DS/$LABEL/pingslamTraj_optimized_$LABEL.txt"
GT=$(gt_for "$DS")
SUM="$WS_HOST/tuning_runs/$DS/summary.csv"

if [[ ! -s "$RUN/pingslamTraj_optimized_$LABEL.txt" ]]; then
  echo "[evaluate] $LABEL: no optimized trajectory — recording FAILED"
  [[ -f "$SUM" ]] || echo "label,dataset,rate,rmse_user,rmse_raw,mean_raw,median_raw,max_raw,rmse_aligned,pairs,loops,seq,poses,rung,bag_wall_s,overrides" > "$SUM"
  echo "$LABEL,$DS,,FAILED,FAILED,,,,,,,,,,,$OVR" >> "$SUM"
  exit 0
fi

dros "evo_ape tum $GT $TRAJ_C"                    > "$RUN/evo_user_repro.txt" 2>&1 || true
dros "evo_ape tum $GT $TRAJ_C --t_max_diff 0.06"  > "$RUN/evo_raw.txt"        2>&1 || true
dros "evo_ape tum $GT $TRAJ_C -a --t_max_diff 0.06" > "$RUN/evo_aligned.txt"  2>&1 || true

stat() { awk -v k="$2" '$1==k {print $2; exit}' "$1" 2>/dev/null || true; }
RMSE_USER=$(stat "$RUN/evo_user_repro.txt" rmse)
RMSE_RAW=$(stat "$RUN/evo_raw.txt" rmse)
MEAN_RAW=$(stat "$RUN/evo_raw.txt" mean)
MED_RAW=$(stat "$RUN/evo_raw.txt" median)
MAX_RAW=$(stat "$RUN/evo_raw.txt" max)
RMSE_AL=$(stat "$RUN/evo_aligned.txt" rmse)
PAIRS=$(grep -oE '[0-9]+ +absolute pose pairs' "$RUN/evo_raw.txt" 2>/dev/null | grep -oE '^[0-9]+' | head -1 || true)

rep() { grep -oE "$2=[^ ]*" "$RUN/run_report.txt" 2>/dev/null | head -1 | cut -d= -f2 || true; }
LOOPS=$(rep "$RUN" loops_closed)
SEQ=$(rep "$RUN" seq_processed)
RUNG=$(rep "$RUN" shutdown_rung)
BAGW=$(rep "$RUN" bag_wall_s)
RATE=$(grep -oE 'rate=[^ ]*' "$RUN/run_report.txt" | head -1 | cut -d= -f2 || true)
POSES=$(wc -l < "$RUN/pingslamTraj_optimized_$LABEL.txt")

[[ -f "$SUM" ]] || echo "label,dataset,rate,rmse_user,rmse_raw,mean_raw,median_raw,max_raw,rmse_aligned,pairs,loops,seq,poses,rung,bag_wall_s,overrides" > "$SUM"
echo "$LABEL,$DS,$RATE,$RMSE_USER,$RMSE_RAW,$MEAN_RAW,$MED_RAW,$MAX_RAW,$RMSE_AL,$PAIRS,$LOOPS,$SEQ,$POSES,$RUNG,$BAGW,$OVR" >> "$SUM"
echo "[evaluate] $LABEL: rmse_raw=$RMSE_RAW aligned=$RMSE_AL user=$RMSE_USER loops=$LOOPS seq=$SEQ poses=$POSES"

# leaderboard sorted by rmse_aligned ascending — the unaligned number is
# dominated by the constant mocap<->map frame offset (~5 m on P3), so the
# Umeyama-aligned APE is the meaningful ranking metric
LB="$WS_HOST/tuning_runs/$DS/leaderboard.md"
{
  echo "# $DS leaderboard (rmse_aligned ascending, evo_ape tum -a --t_max_diff 0.06)"
  echo
  echo "| label | rate | rmse_aligned | rmse_raw | rmse_user(0.01s) | loops | seq | poses | overrides |"
  echo "|---|---|---|---|---|---|---|---|---|"
  tail -n +2 "$SUM" | sort -t, -k9 -g | \
    awk -F, '{printf "| %s | %s | %s | %s | %s | %s | %s | %s | %s |\n",$1,$3,$9,$5,$4,$11,$12,$13,$16}'
} > "$LB"
