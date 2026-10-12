# Reproducing the paper experiments

This document describes how to run and evaluate the retained Ping-SLAM code.
It separates the paper experiment path from the live visualization path.

## Environment and paths

Use Ubuntu 20.04, ROS 2 Foxy, and the dependencies in
[`dependencies/README.md`](dependencies/README.md). Bags and ground truth are
not committed. The harness accepts these variables:

| Variable | Meaning |
|---|---|
| `PING_SLAM_HOST_ROOT` | workspace path on the host |
| `PING_SLAM_CONTAINER_ROOT` | workspace path inside the ROS container |
| `PING_SLAM_CONTAINER` | running container name |
| `PING_SLAM_DATA_ROOT` | dataset root inside the container |
| `PING_SLAM_GT_ROOT` | ground-truth root visible inside the container |
| `PING_SLAM_LOCAL_DATA_ROOT` | optional root for legacy F1 bag paths |
| `PING_SLAM_LOCAL_GT_ROOT` | optional container-visible root for additional F1 reference trajectories |

`tuning/common.sh` is the single dataset mapping table. P2 and P9 currently
reuse the P1 EKF settings because no separate archived settings were retained.
This is intentional and must be reported with those results.

## One complete run

```bash
export PING_SLAM_HOST_ROOT="$PWD"
export PING_SLAM_CONTAINER_ROOT=/root/codes/ping_slam_ws
export PING_SLAM_CONTAINER=scan_slam_foxy
export PING_SLAM_DATA_ROOT=/root/codes/ping_slam_ws/data
export PING_SLAM_GT_ROOT=/root/codes/ping_slam_ws/recordresults/comparative

python3 scripts/select_dataset.py P6
bash tuning/tune_run.sh release_P6 1 P6
bash tuning/evaluate.sh release_P6 P6 release-check
```

The harness snapshots the active YAML files, launches the pipeline, plays the
bag, waits for processing to become stable, calls `save_map` and
`dump_trajectory`, and archives outputs under `tuning_runs/P6/release_P6/`.
`evaluate.sh` runs raw, aligned, and user-frame APE. Set `SMOKE=30` for a
short launch and message-flow test.

The dataset argument chooses the bag and output location; `tune_run.sh` does
not select configurations automatically. Always run `select_dataset.py` first.
The presets are retained working settings, not a manifest of every historical
paper run. Exact reproduction of those runs also requires their archived
configuration snapshots and code revision.

The container must have ROS, dependencies and this workspace built inside it.
Mount the workspace at `PING_SLAM_CONTAINER_ROOT`; bag and ground-truth paths
must also be visible inside the container. `evo`, NumPy, PyYAML and `moreutils`
(for `ts`) must be installed there. Use a dedicated container because the
harness stops processes by name across the container. Run labels must be new;
existing output directories are not overwritten.

## Data and motion-capture messages

The bag-directory and ground-truth filenames are defined in `tuning/common.sh`.
F1–F4 are locally acquired recordings distributed through GitHub Releases.
Download them using [`DATASETS.md`](DATASETS.md) and extract the ZIP files at
the workspace root. F1 includes its TagSLAM reference trajectory. F2–F4 have
no reference trajectories and are evaluated using map quality.

This checkout provides `mocap4r2_msgs`. Some original bags report
`mocap_msgs4r2/msg/RigidBodies` or `mocap_msgs4r2/msg/Markers`. A bag requiring
those names needs a matching message package or a separately validated type
migration of both its metadata and SQLite topic table. The local P6 used for
development was migrated; this source snapshot does not modify downloaded bags.

## Paper result versus supplementary video

Paper figures use the final optimized trajectory and map. The supplementary
video is a real-time RViz view and may use `scripts/map_recolor.py`; it is not
expected to be pixel-identical to the final paper map.

## Release checklist

Before publishing a paper-results tag:

1. record the commit or tag used for every reported table and figure;
2. verify the three active YAML files are the intended dataset settings;
3. run one smoke test in a clean container;
4. archive the run log, configuration snapshot, map, trajectory, and metrics;
5. check that no host path, bag, token, or generated result is tracked.
