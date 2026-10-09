#!/usr/bin/env python3
"""Visualization-only OccupancyGrid relay: /map -> /map_view.

By default, cells >= occ_th (65) become occupied (100); all other cells,
including unknown cells, become zero. With binarize=false, only unknown cells
are replaced by unknown_to (default 0). This changes display values and must
not be treated as an unchanged probabilistic map or as evidence of free space.
Paper map exports and this display need not be pixel-identical.

The publisher uses transient-local durability and repeats the last map every
2 seconds. Use wall time if repetition should continue after bag playback:
  python3 scripts/map_recolor.py --ros-args -p use_sim_time:=false
Parameters: in_topic, out_topic, unknown_to, binarize, occ_th.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from nav_msgs.msg import OccupancyGrid


class MapRecolor(Node):
    def __init__(self):
        super().__init__('map_recolor')
        self.declare_parameter('in_topic', '/map')
        self.declare_parameter('out_topic', '/map_view')
        self.declare_parameter('unknown_to', 0)
        self.declare_parameter('binarize', True)
        self.declare_parameter('occ_th', 65)

        in_topic  = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.unknown_to = int(self.get_parameter('unknown_to').value)
        self.binarize   = bool(self.get_parameter('binarize').value)
        self.occ_th     = int(self.get_parameter('occ_th').value)

        # 发布 transient_local: RViz 后开/重开也能立刻拿到最新地图
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 订阅 volatile+reliable: 与 sonar_slam_backend 的 /map 发布端兼容
        sub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)

        self.pub = self.create_publisher(OccupancyGrid, out_topic, pub_qos)
        self.sub = self.create_subscription(OccupancyGrid, in_topic,
                                            self.on_map, sub_qos)
        # 墙钟定时重播最后一张图: rviz 崩溃重启(respawn)后即使上游不再发新图
        # 也能在 2s 内恢复地图显示。注意本节点须以 use_sim_time:=false 运行,
        # 否则 bag 停播后 sim 时钟冻结, 定时器失效。
        self.last_msg = None
        self.timer = self.create_timer(2.0, self.rebroadcast)
        self.get_logger().info(
            f'[map_recolor] {in_topic} -> {out_topic} '
            f'(binarize={self.binarize}, occ_th={self.occ_th}, '
            f'unknown_to={self.unknown_to})')

    def on_map(self, msg):
        a = np.asarray(msg.data, dtype=np.int8)
        if self.binarize:
            out = np.where(a >= self.occ_th, 100, 0).astype(np.int8)
        else:
            out = a.copy()
            out[a < 0] = np.int8(self.unknown_to)
        msg.data = out.tolist()
        self.last_msg = msg
        self.pub.publish(msg)

    def rebroadcast(self):
        if self.last_msg is not None:
            self.pub.publish(self.last_msg)


def main():
    rclpy.init()
    node = MapRecolor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
