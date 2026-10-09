"""
scan_only_run.py
只跑 /scan 前端：ekfNode + sonarToLaserscan，不启动 sonar_slam_backend。

用途：在 Karto 介入之前，单独验证 /scan 与 odom 的质量。
配合 analyze_scan_bag.py 使用：
    终端1: ros2 launch launch/scan_only_run.py
    终端2: ros2 bag play <bag>
    终端3: ros2 bag record -o scan_check /scan /tf /tf_static
    结束后: python3 src/sonar_slam_frontend/scripts/analyze_scan_bag.py scan_check --tf

检查扫描与里程计在同一坐标系下的几何一致性；形状取决于实际场景和航迹。

陀螺偏置在 ekfDVLnode.cpp 中按 IMU 坐标系扣除；在 slamSettings.yaml 设置
gyroBiasX/Y/Z。设为 0.0 时不扣除对应轴的偏置，无需重编译。
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))

    pkg_share = get_package_share_directory('sonar_slam_frontend')

    def resolve_config(filename):
        cwd_path = str(Path.cwd() / 'config' / filename)
        pkg_path = os.path.join(pkg_share, 'config', filename)
        return cwd_path if os.path.exists(cwd_path) else pkg_path

    slam_settings = resolve_config('slamSettings.yaml')
    s2l_params    = resolve_config('sonar_to_laserscan_params.yaml')

    print(f'[Launch] slam_settings      : {slam_settings}')
    print(f'[Launch] sonar_to_laserscan : {s2l_params}')

    # ── 1. EKF 节点 ───────────────────────────────────────────
    ld.add_action(Node(
        package='sonar_slam_frontend',
        executable='ekfNode',
        name='ekfNode',
        output='screen',
        parameters=[slam_settings],
    ))

    # ── 2. sonarToLaserscan ───────────────────────────────────
    ld.add_action(Node(
        package='sonar_slam_frontend',
        executable='sonarToLaserscan',
        name='sonarToLaserscan',
        output='screen',
        parameters=[s2l_params],
    ))

    # sonar_slam_backend 在"研究 scan"阶段刻意不启动：前端没确认之前跑它只是浪费时间。
    # 确认 /scan 与 odom 都健康后，改用 full_slam_run.py

    return ld
