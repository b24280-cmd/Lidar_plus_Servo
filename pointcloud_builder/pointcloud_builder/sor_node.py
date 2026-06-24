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
NB_NEIGHBORS = 20    # how many nearest neighbours to examine per point
STD_RATIO    = 1.0   # points beyond mean + STD_RATIO * std_dev are removed
# ─────────────────────────────────────────────────────────────────────────────


def _pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]


XYZRGB_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


class SORNode(Node):

    def __init__(self):

        super().__init__('sor_node')

        # /sor/clean  — kept points (white)
        # /sor/noise  — removed outliers (red) so you can see what was cut
        self.clean_pub = self.create_publisher(PointCloud2, '/sor/clean', 10)
        self.noise_pub = self.create_publisher(PointCloud2, '/sor/noise', 10)

        self._clean_pts, self._noise_pts = self._run_sor()

        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            'Publishing on /sor/clean and /sor/noise — enable both in RViz2'
        )

    # ------------------------------------------------------------------
    def _run_sor(self):

        self.get_logger().info(f'Loading {SCAN_PATH} ...')
        pcd = o3d.io.read_point_cloud(SCAN_PATH)
        total = len(pcd.points)
        self.get_logger().info(f'Loaded {total} points')
        self.get_logger().info(
            f'Running SOR  (nb_neighbors={NB_NEIGHBORS}  std_ratio={STD_RATIO}) ...'
        )

        clean, noise_idx = pcd.remove_statistical_outlier(
            nb_neighbors=NB_NEIGHBORS,
            std_ratio=STD_RATIO,
        )
        noise = pcd.select_by_index(noise_idx, invert=True)

        n_clean = len(clean.points)
        n_noise = len(noise.points)
        removed_pct = 100.0 * n_noise / total

        self.get_logger().info(
            f'\nResults:\n'
            f'  Original : {total:>7} points\n'
            f'  Clean    : {n_clean:>7} points  (kept)\n'
            f'  Noise    : {n_noise:>7} points  ({removed_pct:.2f}% removed)\n'
        )

        white = _pack_rgb(220, 220, 220)
        red   = _pack_rgb(230,  50,  50)

        def to_xyzrgb(o3d_cloud, color_float):
            pts = np.asarray(o3d_cloud.points)
            return [[float(p[0]), float(p[1]), float(p[2]), color_float] for p in pts]

        return to_xyzrgb(clean, white), to_xyzrgb(noise, red)

    # ------------------------------------------------------------------
    def _publish(self):

        stamp = self.get_clock().now().to_msg()

        def make_header():
            h = Header()
            h.stamp    = stamp
            h.frame_id = 'map'
            return h

        self.clean_pub.publish(
            point_cloud2.create_cloud(make_header(), XYZRGB_FIELDS, self._clean_pts)
        )
        self.noise_pub.publish(
            point_cloud2.create_cloud(make_header(), XYZRGB_FIELDS, self._noise_pts)
        )


# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = SORNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
