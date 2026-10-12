# Field datasets

## F1–F4 field datasets

[Release page](https://github.com/ymiemie95/ping-slam-core/releases/tag/field-datasets-v1.0)

| Dataset | Scene | ZIP | Reference |
|---|---|---|---|
| F1 | Laboratory plastic pool | [Download (32.7 MB)](https://github.com/ymiemie95/ping-slam-core/releases/download/field-datasets-v1.0/Ping-SLAM_F1.zip) | TagSLAM |
| F2 | Outdoor pond | [Download (188.1 MB)](https://github.com/ymiemie95/ping-slam-core/releases/download/field-datasets-v1.0/Ping-SLAM_F2.zip) | None |
| F3 | Rectangular tiled pool | [Download (25.5 MB)](https://github.com/ymiemie95/ping-slam-core/releases/download/field-datasets-v1.0/Ping-SLAM_F3.zip) | None |
| F4 | Irregular tiled pool | [Download (27.2 MB)](https://github.com/ymiemie95/ping-slam-core/releases/download/field-datasets-v1.0/Ping-SLAM_F4.zip) | None |

[SHA-256 checksums](https://github.com/ymiemie95/ping-slam-core/releases/download/field-datasets-v1.0/SHA256SUMS.txt)

Extract the ZIP archives at the workspace root. They provide the bag directories expected by `tuning/common.sh`. F1 also includes `recordresults/comparative/F1_kcpool/F1_gt.txt` in TUM format. F2–F4 have no reference trajectories.

Build the supplied ROS message definitions before replay. The reference environment is ROS 2 Foxy. Set `PING_SLAM_DATA_ROOT` and `PING_SLAM_GT_ROOT` to container-visible paths when using the experiment harness.
