#!/usr/bin/env python3
"""Dump one latched /map message to <name>.pgm/.yaml in CWD, bypassing
nav2 map_io / ImageMagick (which intermittently SIGSEGVs). Same trinary
mapping and row order as map_saver: free<0.25->254, occ>0.65->0, else 205."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid

name = sys.argv[1] if len(sys.argv) > 1 else 'map_dump'


class Dumper(Node):
    def __init__(self):
        super().__init__('map_topic_dumper')
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.done = False
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, qos)

    def cb(self, m):
        if self.done:
            return
        self.done = True
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = m.data
        rows = []
        for y in range(h - 1, -1, -1):   # PGM row 0 = top; grid row 0 = bottom
            base = y * w
            row = bytearray(w)
            for x in range(w):
                v = data[base + x]
                row[x] = 205 if v < 0 else (254 if v < 25 else (0 if v > 65 else 205))
            rows.append(bytes(row))
        with open(f'{name}.pgm', 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(b''.join(rows))
        with open(f'{name}.yaml', 'w') as f:
            f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {res}\n"
                    f"origin: [{ox}, {oy}, 0]\nnegate: 0\n"
                    f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
        print(f'[map_topic_dump] wrote {name}.pgm ({w}x{h})', flush=True)
        raise SystemExit(0)


rclpy.init()
node = Dumper()
try:
    rclpy.spin(node)
except SystemExit:
    pass
