#!/usr/bin/env bash
# Diagnostic stats from one run dir's run.log — appended to run_report.txt
set -uo pipefail
RUN=${1:?usage: log_stats.sh RUN_DIR}
LOG="$RUN/run.log"
[[ -f "$LOG" ]] || { echo "no run.log"; exit 0; }

echo "loops_closed=$(grep -c 'LOOP:ACCEPT' "$LOG")"  # 补丁版Mapper打印LOOP:ACCEPT, 'Loop closed'不出现
echo "seq_processed=$(grep -c 'SEQ:Process' "$LOG")"
echo "loop_candidates=$(grep -c 'LOOP:nearChain' "$LOG")"
# nonzero = foreign timestamps hit the TF buffer (cross-run pollution sentinel)
echo "tf_old_data=$(grep -c 'TF_OLD_DATA' "$LOG")"

for tag in SEQ:Process LOOP:nearChain; do
  grep "$tag" "$LOG" | grep -o 'response=[0-9.]*' | cut -d= -f2 | sort -n | \
    awk -v t="$tag" '{a[NR]=$1} END{if(NR>0) printf "%s n=%d P50=%.4f P90=%.4f max=%.4f\n", t, NR, a[int(NR*0.5)+1], a[int(NR*0.9)+1], a[NR]; else printf "%s n=0\n", t}'
done

# fake-loop sentinel: loop candidates with |dHeading| > 1.5 rad (180-deg mirror)
grep 'LOOP:' "$LOG" | grep -o 'dHeading=[-0-9.]*' | cut -d= -f2 | \
  awk '($1>1.5||$1<-1.5){n++} END{printf "loop_dheading_gt1.5=%d\n", n+0}'

for f in "$RUN"/pingslamTraj_optimized_*.txt "$RUN"/pingslamTraj_dense_*.txt; do
  [[ -f "$f" ]] && echo "$(basename "$f")=$(wc -l < "$f") poses"
done
exit 0
