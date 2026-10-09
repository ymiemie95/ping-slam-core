# Dataset message definitions

These ROS 2 message packages support Ping-SLAM and playback of the recorded bags:

| Package | Role |
|---|---|
| `ping360_sonar_msgs` | Ping360 sonar echoes |
| `waterlinked_a50` | DVL measurements |
| `px4_msgs` | vehicle sensor/state interfaces used by the front end |
| `mocap4r2_msgs` (directory `mocap_msgs`) | motion-capture references |
| `micron_driver_ros` | additional sonar messages recorded in the datasets |

Package metadata and licenses remain in each directory. The motion-capture
message definitions originate from MOCAP4ROS2:
https://github.com/MOCAP4ROS2-Project/mocap_msgs (recorded revision `cfca28f`).
The historical upstream setup also referenced the mocap tools at revision
`1985ac9`; those control and visualization packages are not included here.

Some original bags use a different motion-capture type spelling. See the
message compatibility notes in the root `REPRODUCE.md` before playback.
