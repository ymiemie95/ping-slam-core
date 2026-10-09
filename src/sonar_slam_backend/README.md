# sonar_slam_backend

Pose-graph SLAM back end for the Ping360 sonar SLAM system: occupancy-grid
correlation scan matching (Karto SDK) plus sparse pose-graph optimization
(Ceres), with a loop-closure decision layer adapted to sparse acoustic
pseudo scans.

This is a modified, trimmed fork of
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
(LGPL-2.1). What was changed and why is listed in `NOTICE`; the upstream
README and feature set do not apply to this package.

The Karto SDK under `lib/karto_sdk` has its own LGPL-3.0 notice, and several
retained upstream headers carry their original CCPL notice. These notices are
component-specific; see `NOTICE` and the files in `lib/karto_sdk`.

Runs as the `sync_sonar_slam_backend_node` executable; see the workspace
`launch/full_slam_run.py` and `config/sonar_slam_backend_params_<dataset>.yaml`
for the per-dataset parameters.
