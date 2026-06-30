#!/usr/bin/env python3

import struct

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

SCAN_PATH = '/home/sam/Downloads/scan.ply'

# ── tunable parameters ────────────────────────────────────────────────────────
DISTANCE_THRESHOLD = 0.05   # metres
MIN_POINTS         = 100    # ignore planes smaller than this
MAX_PLANES         = 5
NUM_ITERATIONS     = 1000
# ─────────────────────────────────────────────────────────────────────────────

# R, G, B  (0-255) per plane
PLANE_COLORS = [
    (230,  50,  50),   # red
    ( 50, 200,  50),   # green
    ( 50, 100, 255),   # blue
    (255, 220,  20),   # yellow
    (255,  50, 255),   # magenta
]
OUTLIER_COLOR = (140, 140, 140)  # grey


def _pack_rgb(r, g, b):
    """Pack three 0-255 ints into the float32 RGB format RViz2 expects."""
    return struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]


class RansacNode(Node):

    def __init__(self):

        super().__init__('ransac_node')

        self.pub = self.create_publisher(PointCloud2, '/ransac/result', 10)

        self._points = self._run_ransac()   # list of [x, y, z, rgb_float]

        # publish at 1 Hz so RViz2 always has fresh data
        self.create_timer(1.0, self._publish)
        self.get_logger().info('Publishing on /ransac/result — add PointCloud2 in RViz2')

    # ------------------------------------------------------------------
    def _run_ransac(self):

        self.get_logger().info(f'Loading {SCAN_PATH} ...')
        pcd = o3d.io.read_point_cloud(SCAN_PATH)
        total = len(pcd.points)
        self.get_logger().info(f'Loaded {total} points\n')

        remaining = pcd
        result = []

        for i in range(MAX_PLANES):

            if len(remaining.points) < MIN_POINTS:
                break

            model, inliers = remaining.segment_plane(
                distance_threshold=DISTANCE_THRESHOLD,
                ransac_n=3,
                num_iterations=NUM_ITERATIONS,
            )

            if len(inliers) < MIN_POINTS:
                self.get_logger().info(f'Plane {i + 1}: only {len(inliers)} inliers — stopping.')
                break

            a, b, c, d = model
            pct = 100.0 * len(inliers) / total
            self.get_logger().info(
                f'Plane {i + 1}:  {len(inliers):>6} pts ({pct:.1f}%)   '
                f'{a:+.3f}x {b:+.3f}y {c:+.3f}z {d:+.3f} = 0'
            )

            rgb = _pack_rgb(*PLANE_COLORS[i % len(PLANE_COLORS)])
            pts = np.asarray(remaining.select_by_index(inliers).points)
            for p in pts:
                result.append([float(p[0]), float(p[1]), float(p[2]), rgb])

            remaining = remaining.select_by_index(inliers, invert=True)

        # outliers in grey
        grey = _pack_rgb(*OUTLIER_COLOR)
        for p in np.asarray(remaining.points):
            result.append([float(p[0]), float(p[1]), float(p[2]), grey])

        self.get_logger().info(f'\nNon-planar points: {len(remaining.points)}\n')
        return result

    # ------------------------------------------------------------------
    def _publish(self):

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.pub.publish(point_cloud2.create_cloud(header, fields, self._points))


# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = RansacNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
