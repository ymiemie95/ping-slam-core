#!/usr/bin/env python3
"""Record an online planar map -> base_link trajectory in eight-column TUM format.

This is the same file format as rosGtSaver, not the same coordinate frame or
trajectory. Compose the latest map -> odom and odom -> base_link transforms
at each odometry timestamp; this does not retrospectively optimize old poses.
Use the backend dump_trajectory service for the final optimized trajectory.

The default output is pingslamTraj_.txt next to this script. Override it with:
  python3 scripts/traj_recorder.py --ros-args -p output_file:=/tmp/live_traj.txt
Save occurs on shutdown. This script is also started by full_slam_run.py.
"""

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_msgs.msg import TFMessage


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def compose_2d(a, b):
    """2D 位姿复合：world_T_a ∘ a_T_b -> world_T_b。
    a, b 均为 (x, y, yaw)。"""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    wx = ax + c * bx - s * by
    wy = ay + s * bx + c * by
    return (wx, wy, ayaw + byaw)


class SlamToolboxTrajRecorder(Node):
    """直接订阅 /tf 与 /tf_static，自行维护 map->odom 和 odom->base_link，
    相乘得 map->base_link。绕开 tf2 buffer 在 sim_time 下 Time() 查询的坑。"""

    def __init__(self):
        super().__init__('sonar_slam_backend_traj_recorder')

        default_out = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'pingslamTraj_.txt')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_file', default_out)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.output_file = self.get_parameter('output_file').value

        # 最新的两段变换，(x, y, yaw)；None 表示尚未收到
        self.map_T_odom = None
        self.odom_T_base = None

        # 输出轨迹：以 odom->base_link 的时间戳为采样点（最高频、连续）
        # (timestamp, x, y, qz, qw)
        self.poses = []
        self.last_stamp_ns = None

        # /tf 普通话题；/tf_static 需要 TRANSIENT_LOCAL
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        static_qos = QoSProfile(depth=100)
        static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(TFMessage, '/tf_static', self.on_tf, static_qos)

        self.get_logger().info(
            f'Recording {self.map_frame}->{self.base_frame} via /tf. '
            f'Ctrl+C to save to {self.output_file}')

    def on_tf(self, msg: TFMessage):
        for tr in msg.transforms:
            parent = tr.header.frame_id.lstrip('/')
            child = tr.child_frame_id.lstrip('/')
            t = tr.transform.translation
            q = tr.transform.rotation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            pose = (t.x, t.y, yaw)

            if parent == self.map_frame and child == self.odom_frame:
                self.map_T_odom = pose
            elif parent == self.odom_frame and child == self.base_frame:
                self.odom_T_base = pose
                # 每当 odom->base 更新就尝试输出一帧 map->base
                self.try_record(tr.header.stamp)

    def try_record(self, stamp):
        if self.map_T_odom is None or self.odom_T_base is None:
            return
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns == self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        mx, my, myaw = compose_2d(self.map_T_odom, self.odom_T_base)
        qz = math.sin(myaw / 2.0)
        qw = math.cos(myaw / 2.0)
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self.poses.append((timestamp, mx, my, qz, qw))
        # if len(self.poses) == 1 or len(self.poses) % 200 == 0:
        #     self.get_logger().info(
        #         f'captured {len(self.poses)} poses '
        #         f'(latest t={timestamp:.3f} x={mx:.2f} y={my:.2f})')

    def save(self):
        if not self.poses:
            self.get_logger().warn('No TF samples captured; nothing to save.')
            return
        with open(self.output_file, 'w') as f:
            for ts, x, y, qz, qw in self.poses:
                f.write(f'{ts:.6f} {x:.6f} {y:.6f} 0.000000 '
                        f'0.000000 0.000000 {qz:.6f} {qw:.6f}\n')
        self.get_logger().info(
            f'Saved {len(self.poses)} poses to {self.output_file}')


def main():
    rclpy.init()
    node = SlamToolboxTrajRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
