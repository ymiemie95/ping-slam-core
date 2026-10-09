#!/usr/bin/env python3
"""
traj_visualizer.py —— 在 RViz 里画两条轨迹

发布:
  /slam_traj  (nav_msgs/Path, map 系)  —— 在线 map→base_link 采样，不回溯更新旧点
  /odom_traj  (nav_msgs/Path, odom 系) —— odom→base_link 采样；在 RViz 中通过 TF 显示

用法:
  python3 traj_visualizer.py --ros-args -p use_sim_time:=true
RViz: 添加两个 Path 显示, topics 选 /slam_traj 与 /odom_traj, 颜色区分。

消息驱动(无 /clock 依赖): 订阅 /publisherPoseEkf(~200Hz), 每 N 条采样一次
TF, 用消息自带的 header 时间戳 —— 与管线其他节点同一机制。
参考 Foxy 环境的 ros2 bag play 没有 --clock 选项，因此这里使用消息触发。
相邻点距 < min_step (默认 0.01m) 时不记点, 防止原地悬停时刷点。
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf2_ros


class TrajVisualizer(Node):
    def __init__(self):
        super().__init__('traj_visualizer')
        self.declare_parameter('decimate', 40)     # 200Hz/40 = ~5Hz 采样
        self.declare_parameter('min_step', 0.01)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('trigger_topic', '/publisherPoseEkf')

        self.map_frame  = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_step   = self.get_parameter('min_step').value
        self.decimate   = int(self.get_parameter('decimate').value)

        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.pub_slam = self.create_publisher(Path, '/slam_traj', 1)
        self.pub_odom = self.create_publisher(Path, '/odom_traj', 1)
        self.path_slam = Path(); self.path_slam.header.frame_id = self.map_frame
        self.path_odom = Path(); self.path_odom.header.frame_id = self.odom_frame

        # 消息驱动: 无 /clock 依赖 (Foxy 的 bag play 不能发 /clock)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.count = 0
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('trigger_topic').value,
            self.on_pose, qos)
        self.get_logger().info(
            f'[traj_visualizer] {self.map_frame}->{self.base_frame} => /slam_traj, '
            f'{self.odom_frame}->{self.base_frame} => /odom_traj '
            f'(message-driven, decimate={self.decimate})')

    def on_pose(self, msg):
        self.count += 1
        if self.count % self.decimate != 0:
            return
        self.tick(msg.header.stamp)

    def append(self, path, parent, stamp):
        try:
            tf = self.buf.lookup_transform(parent, self.base_frame,
                                           rclpy.time.Time())   # 最新
        except Exception:
            return
        ps = PoseStamped()
        ps.header.frame_id = parent
        ps.header.stamp = stamp
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        if path.poses:
            q = path.poses[-1].pose.position
            if (abs(ps.pose.position.x - q.x) < self.min_step and
                    abs(ps.pose.position.y - q.y) < self.min_step):
                return
        path.poses.append(ps)

    def tick(self, stamp):
        self.append(self.path_slam, self.map_frame,  stamp)
        self.append(self.path_odom, self.odom_frame, stamp)
        self.path_slam.header.stamp = stamp
        self.path_odom.header.stamp = stamp
        self.pub_slam.publish(self.path_slam)
        self.pub_odom.publish(self.path_odom)


def main():
    rclpy.init()
    node = TrajVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
