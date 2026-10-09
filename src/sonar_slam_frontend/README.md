# sonar_slam_frontend

Front end of the Ping360 mechanical-scanning-sonar SLAM system. Pairs with
`sonar_slam_backend` (the patched slam_toolbox/karto pose-graph back end).

## Nodes

| Executable | Role | Parameters |
|---|---|---|
| `ekfNode` | loosely coupled DVL/IMU EKF odometry (constant gyro-bias subtraction, DVL dropouts skipped) | `config/slamSettings.yaml` |
| `sonarToLaserscan` | `ping360_sonar_msgs/SonarEcho` beams → `sensor_msgs/LaserScan` pseudo scans: background-subtracted range extraction, EKF-based motion compensation, segment densification, frame gate | `config/sonar_to_laserscan_params.yaml` |
| `rosGtSaver` | dumps `mocap4r2_msgs/RigidBodies` ground truth to a TUM file | — |

Script: `scripts/analyze_scan_bag.py` (front-end sanity plots from a recorded
`/scan` bag; see `launch/scan_only_run.py`).

Library `generalhelpfullib` provides Eigen/tf2 helpers for the front-end nodes.

## Launch

From the workspace root:

```
ros2 launch launch/full_slam_run.py    # EKF + pseudo scans + sonar_slam_backend
ros2 launch launch/scan_only_run.py    # front end only
ros2 bag play <bag>
```

## License

BSD-3-Clause (`LICENSE`). The EKF and the helper library are derived from
[constructor-robotics/underwaterSLAM_2D_lowCost](https://github.com/constructor-robotics/underwaterSLAM_2D_lowCost);
see `NOTICE` for the provenance and the list of modifications.
