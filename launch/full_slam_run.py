"""
full_slam_run.py —— 完整管线 launch：一个文件管全部节点

节点清单（按数据流顺序）:
  1. ekfNode            —— IMU+DVL 里程计 (陀螺零偏补偿 / DVL 掉点处理已内置 cpp,
                           零偏在 slamSettings.yaml 的 gyroBiasX/Y/Z 里设)
  2. sonarToLaserscan   —— Ping360 → /scan (max 或 max_bgsub 提取、段滤波、
                           段加密和 validBins 闸门)
  3. sonar_slam_backend —— Karto 扫描匹配与 Ceres 位姿图优化 (sync 模式)
  4. traj visualizer    —— [可选] /slam_traj + /odom_traj 供 RViz 显示
  5. map_recolor        —— [可选] /map → /map_view 显示用二值化及未知区改白
  6. rviz2              —— [可选] 论文视频白底配置 config/paperView.rviz
  7. traj recorder      —— [可选] 在线 map→base_link TF → TUM 轨迹文件，
                           不回溯优化历史点；最终轨迹由 dump_trajectory 服务导出

用法:
  ros2 launch launch/full_slam_run.py
  终端2: ros2 bag play <bag>                # 参考 Foxy 环境不支持 --clock

场景切换(大甲鱼池/小瓷砖池)不改本文件 —— 改 config/ 下三份 yaml:
  sonar_to_laserscan_params.yaml : range 门 / 提取方法(max|max_bgsub)
                                   / 滤波尺度参数 / 段加密(enable_segment_densify)
  sonar_slam_backend_params.yaml : min/max_laser_range / 搜索空间尺度
  slamSettings.yaml              : IMU/DVL 安装旋转和 gyroBiasX/Y/Z 等

启动后核对横幅三行:
  [ekfNode] gyro bias = (...)                ← 与所选数据集标定一致，可为零
  [ekfNode] DVL 有效率 xx%                      ← 播放后周期出现
  [sonarToLaserscan] ... method=xxx             ← 与 yaml 一致
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ══════════════ 开关区: 只改这里 ══════════════
USE_BACKEND      = True    # False = 只跑前端(scan-only 调试模式)
RECORD_TRAJ      = True    # True = 录制 TUM 轨迹
SHOW_TRAJ_RVIZ   = True    # True = 发布 /slam_traj 与 /odom_traj 供 RViz 显示
USE_MAP_RECOLOR  = True    # True = /map → /map_view 中继(未知区灰绿改白, 视频录制用)
LAUNCH_RVIZ      = False   # True = 自动启动 rviz2 (config/paperView.rviz 白底出版配色)
                           # Keep disabled for headless playback and timing measurements.
# ═════════════════════════════════════════════


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))

    pkg_share = get_package_share_directory('sonar_slam_frontend')

    def resolve_config(filename):
        cwd_path = str(Path.cwd() / 'config' / filename)
        pkg_path = os.path.join(pkg_share, 'config', filename)
        return cwd_path if os.path.exists(cwd_path) else pkg_path

    def resolve_script(*candidates):
        for p in candidates:
            if os.path.exists(p):
                return p
        return candidates[0]

    slam_settings = resolve_config('slamSettings.yaml')
    s2l_params    = resolve_config('sonar_to_laserscan_params.yaml')
    stb_params    = resolve_config('sonar_slam_backend_params.yaml')

    print(f'[Launch] slam_settings       : {slam_settings}')
    print(f'[Launch] sonar_to_laserscan  : {s2l_params}')
    print(f'[Launch] sonar_slam_backend  : '
          f'{"ON  params=" + stb_params if USE_BACKEND else "OFF (scan-only 调试)"}')

    # ── 1. EKF 节点 ───────────────────────────────────────────
    ld.add_action(Node(
        package='sonar_slam_frontend',
        executable='ekfNode',
        name='ekfNode',
        output='screen',
        parameters=[slam_settings],
    ))

    # ── 2. sonarToLaserscan (提取 + 段加密都在这里) ───────────
    ld.add_action(Node(
        package='sonar_slam_frontend',
        executable='sonarToLaserscan',
        name='sonarToLaserscan',
        output='screen',
        parameters=[s2l_params],
    ))

    # ── 3. sonar_slam_backend ─────────────────────────────────
    if USE_BACKEND:
        ld.add_action(Node(
            package='sonar_slam_backend',
            executable='sync_sonar_slam_backend_node',
            name='sonar_slam_backend',
            output='screen',
            parameters=[stb_params],
        ))

    # ── 4. [可选] RViz 轨迹可视化 (/slam_traj + /odom_traj) ───
    if SHOW_TRAJ_RVIZ:
        viz_script = resolve_script(
            str(Path.cwd() / 'scripts' / 'traj_visualizer.py'),
            str(Path.cwd() / 'traj_visualizer.py'),
            os.path.join(pkg_share, 'lib', 'sonar_slam_frontend', 'traj_visualizer.py'),
        )
        print(f'[Launch] traj visualizer     : {viz_script}')
        # decimate 100 → ~2Hz 全量路径重发(采点距约0.2-0.4m):
        # 5Hz 会加剧 Foxy rviz2 的 malloc 崩溃, 再稀轨迹折线感明显
        ld.add_action(ExecuteProcess(
            cmd=['python3', viz_script,
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', 'decimate:=100'],
            output='screen',
        ))

    # ── 5. [可选] 地图美化中继 (/map → /map_view, 未知区改白) ─
    if USE_BACKEND and USE_MAP_RECOLOR:
        recolor_script = resolve_script(
            str(Path.cwd() / 'scripts' / 'map_recolor.py'),
            str(Path.cwd() / 'map_recolor.py'),
            os.path.join(pkg_share, 'lib', 'sonar_slam_frontend', 'map_recolor.py'),
        )
        print(f'[Launch] map recolor         : {recolor_script} (/map -> /map_view)')
        # 需要在停播后继续重播时 use_sim_time 应为 false: 其墙钟定时重播地图给 respawn 后的 rviz,
        # sim 时钟在 bag 停播后会冻结致定时器失效
        ld.add_action(ExecuteProcess(
            cmd=['python3', recolor_script,
                 '--ros-args', '-p', 'use_sim_time:=false'],
            output='screen',
        ))

    # ── 6. [可选] RViz (论文视频白底配置) ─────────────────────
    # 用 ExecuteProcess+respawn: Foxy rviz2 有随机 malloc 崩溃 bug,
    # 录制中崩了 2s 内自动复活(地图 transient_local 秒回, 轨迹下次路径发布时恢复)
    if LAUNCH_RVIZ:
        rviz_cfg = resolve_config('paperView.rviz')
        print(f'[Launch] rviz2               : {rviz_cfg} (respawn=on)')
        ld.add_action(ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ))

    # ── 7. [可选] TUM 轨迹录制 ────────────────────────────────
    if RECORD_TRAJ and USE_BACKEND:
        recorder_script = resolve_script(
            str(Path.cwd() / 'scripts' / 'traj_recorder.py'),
            os.path.join(pkg_share, 'lib', 'sonar_slam_frontend', 'traj_recorder.py'),
        )
        print(f'[Launch] traj recorder       : {recorder_script}')
        ld.add_action(ExecuteProcess(
            cmd=['python3', recorder_script,
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ))

    return ld
