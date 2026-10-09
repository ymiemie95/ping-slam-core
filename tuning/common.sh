# shared helpers for the dataset tuning harness — source this, do not execute
# The original harness assumed the author's host and container paths.  Keep
# those values as defaults for the local setup, but allow a clean checkout to
# override them without editing this file.
COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_HOST="${PING_SLAM_HOST_ROOT:-$(cd "$COMMON_DIR/.." && pwd)}"
WS="${PING_SLAM_CONTAINER_ROOT:-/root/codes/ping_slam_ws}"
C="${PING_SLAM_CONTAINER:-scan_slam_foxy}"
SRC="source /opt/ros/foxy/setup.bash && source $WS/install/setup.bash"
DATA_ROOT="${PING_SLAM_DATA_ROOT:-$WS/data}"
# Ground truth is consumed by evo inside the container, so this is a
# container-visible path (normally on the shared workspace mount).
GT_ROOT="${PING_SLAM_GT_ROOT:-$WS/recordresults/comparative}"

# process-match regex for pkill/pgrep -f; bracketed chars keep the pattern
# from matching the docker-exec shell that carries it on its own cmdline.
# Process-name cleanup is container-wide, not ROS-domain-scoped. Use a
# dedicated container: these patterns can also match another experiment.
PROCPAT='ekfNod[e]|sonarToLasersca[n]|sync_sonar_slam_backend_nod[e]|traj_visualize[r]|record_sonar_slam_backend_tra[j]|bin/ros2 laun[c]h'

# ROS_DOMAIN_ID separates topic discovery from domain 0. Process cleanup
# above is still container-wide; use a dedicated container for this harness.
TUNE_DOMAIN=42

dex()  { docker exec "$C" bash -c "$*"; }
dros() { docker exec "$C" bash -c "export ROS_DOMAIN_ID=$TUNE_DOMAIN && cd $WS && $SRC && $*"; }

# 数据集编号 (2026-09-07 起, 与源数据集论文对齐):
#   P1-P13  Bremen MSS 公开集, P<n> 就是该论文的 DataSet<n>, 全部有 mocap 真值
#   B1      Valentin 潜艇堡地下室, 公开但无真值
#   F1      自采实验室水池, 有 TagSLAM 参考轨迹
#   F2-F4   自采甲鱼池 / 两个瓷砖池, 无真值 (只有卷尺尺寸)
bag_for() {
  case "$1" in
    P1)  echo "$DATA_ROOT/P1_DataSet01" ;;
    P2)  echo "$DATA_ROOT/P2_DataSet02" ;;
    P3)  echo "$DATA_ROOT/P3_DataSet03" ;;
    P4)  echo "$DATA_ROOT/P4_DataSet04" ;;
    P5)  echo "$DATA_ROOT/P5_DataSet05" ;;
    P6)  echo "$DATA_ROOT/P6_DataSet06" ;;
    P7)  echo "$DATA_ROOT/P7_DataSet07" ;;
    P8)  echo "$DATA_ROOT/P8_DataSet08" ;;
    P9)  echo "$DATA_ROOT/P9_DataSet09" ;;
    P10) echo "$DATA_ROOT/P10_DataSet10" ;;
    P11) echo "$DATA_ROOT/P11_DataSet11" ;;
    P12) echo "$DATA_ROOT/P12_DataSet12" ;;
    P13) echo "$DATA_ROOT/P13_DataSet13" ;;
    B1)  echo "$DATA_ROOT/B1_ValentinBunkerKellerNew" ;;   # 3160s, 无真值
    F1)     echo "${PING_SLAM_LOCAL_DATA_ROOT:-$DATA_ROOT}/pool_r3_s4" ;;
    F1S1)   echo "${PING_SLAM_LOCAL_DATA_ROOT:-$DATA_ROOT}/pool_r3_s1" ;;
    F1S2)   echo "${PING_SLAM_LOCAL_DATA_ROOT:-$DATA_ROOT}/pool_r3_s2" ;;
    F1R4S2) echo "${PING_SLAM_LOCAL_DATA_ROOT:-$DATA_ROOT}/pool_r4_s2" ;;
    F1R4S4) echo "${PING_SLAM_LOCAL_DATA_ROOT:-$DATA_ROOT}/pool_r4_s4" ;;
    F2)  echo "$DATA_ROOT/F2_fish_20260709_1553" ;;        # 无真值
    F3)  echo "$DATA_ROOT/F3_zjpool_20260712_1505" ;;      # 无真值
    F4)  echo "$DATA_ROOT/F4_zjspool_20260712_1709_s4" ;;  # 无真值
    *) echo "unknown dataset $1" >&2; return 1 ;;
  esac
}

# 只有 P1-P13 与 F1 有真值; B1/F2/F3/F4 走地图质量口径, 不进这张表
gt_for() {
  case "$1" in
    P1)  echo "$GT_ROOT/P1_data01/P1_gt.txt" ;;
    P2)  echo "$GT_ROOT/P2_data02/gtTrajectory_full_Dataset02.txt" ;;
    P3)  echo "$GT_ROOT/P3_data03/P3_gt.txt" ;;
    P4)  echo "$GT_ROOT/P4_data04/gtTrajectory_full_Dataset04.txt" ;;
    P5)  echo "$GT_ROOT/P5_data05/gtTrajectory_full_Dataset05.txt" ;;
    P6)  echo "$GT_ROOT/P6_data06/gtTrajectory_full_Dataset06.txt" ;;
    P7)  echo "$GT_ROOT/P7_data07/gtTrajectory_full_Dataset07.txt" ;;
    P8)  echo "$GT_ROOT/P8_data08/gtTrajectory_full_Dataset08.txt" ;;
    P9)  echo "$GT_ROOT/P9_data09/gtTrajectory_full_Datast09.txt" ;;
    P10) echo "$GT_ROOT/P10_data10/gtTrajectory_full_Dataset10.txt" ;;
    P11) echo "$GT_ROOT/P11_data11/gtTrajectory_full_Dataset11.txt" ;;
    P12) echo "$GT_ROOT/P12_data12/gtTrajectory_full_Dataset12.txt" ;;
    P13) echo "$GT_ROOT/P13_data13/gtTrajectory_full_Dataset13.txt" ;;
    F1)     echo "$GT_ROOT/F1_kcpool/F1_gt.txt" ;;
    F1S1)   echo "${PING_SLAM_LOCAL_GT_ROOT:-$GT_ROOT}/gt_r3_s1.txt" ;;
    F1S2)   echo "${PING_SLAM_LOCAL_GT_ROOT:-$GT_ROOT}/gt_r3_s2.txt" ;;
    F1R4S2) echo "${PING_SLAM_LOCAL_GT_ROOT:-$GT_ROOT}/gt_r4_s2.txt" ;;
    F1R4S4) echo "${PING_SLAM_LOCAL_GT_ROOT:-$GT_ROOT}/gt_r4_s4.txt" ;;
    *) echo "unknown dataset $1" >&2; return 1 ;;
  esac
}

settings_for() {
  case "$1" in
    P1)  echo config/slamSettings_P1.yaml ;;
    P2)  echo config/slamSettings_P1.yaml ;; # no P2 archive; P3-style settings gave mirrored
                                             # trajectory (APE 5.0 vs archived 0.29) — P1-style works
    P3)  echo config/slamSettings_P3.yaml ;;
    P4)  echo config/slamSettings_P4.yaml ;;
    P5)  echo config/slamSettings_P5.yaml ;;
    P6)  echo config/slamSettings_P6.yaml ;;
    P7)  echo config/slamSettings_P7.yaml ;;
    P8)  echo config/slamSettings_P8.yaml ;;
    P9)  echo config/slamSettings_P1.yaml ;; # no P9 archive; same DataSetXX family assumption
    P10) echo config/slamSettings_P10.yaml ;;
    P11) echo config/slamSettings_P11.yaml ;;
    P12) echo config/slamSettings_P12.yaml ;;
    P13) echo config/slamSettings_P13.yaml ;;
    B1)  echo config/slamSettings_B1.yaml ;;
    F1|F1S1|F1S2|F1R4S2|F1R4S4) echo config/slamSettings_F1.yaml ;;
    F2)  echo config/slamSettings_F2.yaml ;;
    F3)  echo config/slamSettings_F3.yaml ;;
    F4)  echo config/slamSettings_F4.yaml ;;
    *) echo "unknown dataset $1" >&2; return 1 ;;   # 无默认分支时未知 ID 会静默返回空串
  esac
}

# INT -> TERM -> KILL ladder; abort if anything survives
ros_cleanup() {
  dex "pkill -INT -f 'bin/ros2 laun[c]h'" >/dev/null 2>&1 || true
  sleep 4
  if dex "pgrep -f '$PROCPAT'" >/dev/null 2>&1; then
    dex "pkill -TERM -f '$PROCPAT'" >/dev/null 2>&1 || true
    sleep 3
  fi
  if dex "pgrep -f '$PROCPAT'" >/dev/null 2>&1; then
    dex "pkill -KILL -f '$PROCPAT'" >/dev/null 2>&1 || true
    sleep 2
  fi
  if dex "pgrep -af '$PROCPAT'" 2>/dev/null; then
    echo "[cleanup] stale ROS processes survived KILL — aborting" >&2
    return 3
  fi
  # fresh CLI daemon per run: a stale daemon after node churn made
  # 'ros2 service list' return nothing and readiness checks time out
  dros "ros2 daemon stop" >/dev/null 2>&1 || true
  sleep 1
  return 0
}

# Check publishers in TUNE_DOMAIN via dros(). This does not check domain 0
# or isolate the container-wide process cleanup above.
assert_quiet_domain() {
  local t p bad=0
  for t in /sonar/intensity /imu/data /scan /tf /clock; do
    p=$(dros "ros2 topic info $t 2>/dev/null" | awk '/Publisher count/{print $3}' || true)
    p=${p:-0}
    if [[ "$p" != "0" ]]; then
      echo "[preflight] $t has $p publisher(s)" >&2
      bad=1
    fi
  done
  if [[ $bad -ne 0 ]]; then
    echo "[preflight] ROS 域 $TUNE_DOMAIN 不安静 — 容器或宿主里还有别的进程在发布(比如手动 ros2 bag play)。" >&2
    echo "[preflight] 为不误杀你的实验, 本轮中止。请等它结束或手动停掉后重跑。" >&2
    dex "pgrep -af 'ros2 ba[g]|launc[h]'" >&2 || true
    return 4
  fi
  return 0
}

# move leftover fixed-name outputs out of the way (never delete)
stash_stale() {
  local ts stamp
  stamp=$(date +%Y%m%d_%H%M%S)
  mkdir -p "$WS_HOST/tuning_runs/_stale"
  local f
  for f in "$WS_HOST/pingslamTraj_optimized.txt" "$WS_HOST/scripts/pingslamTraj_.txt"; do
    if [[ -e "$f" ]]; then
      mv "$f" "$WS_HOST/tuning_runs/_stale/$(basename "$f").$stamp"
      echo "[stash] $(basename "$f") -> _stale/"
    fi
  done
  for f in "$WS_HOST"/map_*.pgm "$WS_HOST"/map_*.yaml; do
    if [[ -e "$f" ]]; then
      mv "$f" "$WS_HOST/tuning_runs/_stale/$(basename "$f").$stamp"
      echo "[stash] $(basename "$f") -> _stale/"
    fi
  done
  return 0
}
