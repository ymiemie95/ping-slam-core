# Dependencies

The reference environment is Ubuntu 20.04 with ROS 2 Foxy. After installing
ROS 2 Foxy desktop, install the build and runtime dependencies:

```bash
source /opt/ros/foxy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-numpy \
  python3-yaml python3-matplotlib build-essential cmake pkg-config \
  libeigen3-dev libboost-all-dev libceres-dev libsuitesparse-dev \
  liblapack-dev libblas-dev libtbb-dev ros-foxy-nav2-map-server moreutils ripgrep
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

The backend uses Ceres with SuiteSparse and the bundled Karto SDK. Karto uses
TBB. The source tree includes the message packages needed by the datasets.
See `src/datasetMessageDefinitions/README.md` for their roles and provenance.
The map export service calls `nav2_map_server`, declared as a runtime dependency.

For trajectory evaluation, also install `evo`:

```bash
python3 -m pip install --user evo
```

Matplotlib is used by optional scan-analysis and map-rendering scripts.
Docker is needed only for the experiment harness in `tuning/`; interactive
launch and playback can run directly in a ROS 2 workspace.

The reference dependency versions are Ceres 1.14.0, Eigen 3.3.7, TBB 2020.1,
and nav2_map_server 0.4.7.
