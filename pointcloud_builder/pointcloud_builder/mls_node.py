#!/usr/bin/env python3
#
# Moving Least Squares smoothing — plane-projection variant (0th-order MLS).
#
# For each point:
#   1. Find K nearest neighbours
#   2. Fit a local tangent plane through those neighbours (PCA → smallest eigenvector = normal)
#   3. Project the point onto that plane
#
# Effect: noise perpendicular to surfaces is removed; walls/floors become smoother.
# The whole maths batch is vectorised with numpy so it runs in a few seconds.

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
K_NEIGHBORS = 20   # neighbours used to estimate local tangent plane
               # higher → smoother but slower and more over-smoothed
# ─────────────────────────────────────────────────────────────────────────────


def _pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]


XYZRGB_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


class MLSNode(Node):

    def __init__(self):

        super().__init__('mls_node')

        self.pub = self.create_publisher(PointCloud2, '/mls/smoothed', 10)

        self._pts = self._run_mls()

        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            'Publishing smoothed cloud on /mls/smoothed — enable it in RViz2'
        )

    # ------------------------------------------------------------------
    def _run_mls(self):

        self.get_logger().info(f'Loading {SCAN_PATH} ...')
        pcd = o3d.io.read_point_cloud(SCAN_PATH)
        pts = np.asarray(pcd.points)
        n   = len(pts)
        self.get_logger().info(f'Loaded {n} points')
        self.get_logger().info(f'Running MLS  (k={K_NEIGHBORS}) ...')

        kdtree = o3d.geometry.KDTreeFlann(pcd)

        # ---- collect all neighbour index lists (Python loop, but fast) ----
        self.get_logger().info('  Building neighbour index table ...')
        idx_table = []
        for i in range(n):
            _, idx, _ = kdtree.search_knn_vector_3d(pts[i], K_NEIGHBORS)
            idx_table.append(list(idx))

        # ---- vectorised maths ----
        self.get_logger().info('  Fitting local planes (vectorised) ...')

        # neighbours array: (n, k, 3)
        neighbors = pts[np.array(idx_table)]          # (n, k, 3)

        # local centroids: (n, 3)
        centroids = neighbors.mean(axis=1)

        # mean-centred neighbours: (n, k, 3)
        centered = neighbors - centroids[:, np.newaxis, :]

        # 3×3 covariance per point: C[i] = centered[i].T @ centered[i]
        cov = np.einsum('nki,nkj->nij', centered, centered) / K_NEIGHBORS  # (n, 3, 3)

        # batch SVD — last right-singular vector = local surface normal
        _, _, Vt = np.linalg.svd(cov)   # Vt: (n, 3, 3)
        normals = Vt[:, -1, :]           # (n, 3)

        # project each point onto its local plane
        diff       = pts - centroids                                     # (n, 3)
        projection = np.einsum('ni,ni->n', diff, normals)               # (n,) signed dist
        smoothed   = pts - projection[:, np.newaxis] * normals          # (n, 3)

        # ---- stats ----
        displacements = np.linalg.norm(smoothed - pts, axis=1)
        self.get_logger().info(
            f'\nMLS complete:\n'
            f'  Mean point displacement : {displacements.mean()*1000:.2f} mm\n'
            f'  Max  point displacement : {displacements.max()*1000:.2f} mm\n'
        )

        white = _pack_rgb(220, 220, 220)
        return [[float(p[0]), float(p[1]), float(p[2]), white] for p in smoothed]

    # ------------------------------------------------------------------
    def _publish(self):

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        self.pub.publish(
            point_cloud2.create_cloud(header, XYZRGB_FIELDS, self._pts)
        )


# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = MLSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
