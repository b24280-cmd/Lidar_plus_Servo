#!/usr/bin/env python3
#
# ICP alignment node — handles large rotations (including ~90°).
#
# Strategy:
#   1. Try all six ±90° axis-aligned rotation seeds (centroid-aligned) in
#      parallel with a fast coarse ICP pass.
#   2. Pick the seed with the best coarse fitness.
#   3. Refine with multi-scale ICP (coarse → fine voxels).
#
# Publishers:
#   /icp/scan_a         first scan        (blue)
#   /icp/scan_b_raw     second scan raw   (red)
#   /icp/scan_b_aligned second scan after ICP  (green)
#   /icp/merged         both merged, coloured by source
#
# Parameters (no rebuild needed):
#   scan_a      path to first PLY   (reference)
#   scan_b      path to second PLY  (aligned onto A)
#   voxel_fine  finest ICP voxel in metres (default 0.05)

import os
import struct
import math

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

SCAN_DIR = '/home/sam/Desktop/Scans_s3_rplidar'


# ── colour helpers ────────────────────────────────────────────────────────────

def _pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I',
        (int(r) << 16) | (int(g) << 8) | int(b)))[0]

XYZRGB_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]

def _to_xyzrgb(pts, r, g, b):
    c = _pack_rgb(r, g, b)
    return [[float(p[0]), float(p[1]), float(p[2]), c] for p in pts]

def _make_cloud(node, pts_list):
    h = Header()
    h.stamp    = node.get_clock().now().to_msg()
    h.frame_id = 'map'
    return point_cloud2.create_cloud(h, XYZRGB_FIELDS, pts_list)


# ── rotation helpers ───────────────────────────────────────────────────────────

def _Rx(deg):
    r = math.radians(deg)
    return np.array([[1, 0,          0         ],
                     [0, math.cos(r),-math.sin(r)],
                     [0, math.sin(r), math.cos(r)]])

def _Ry(deg):
    r = math.radians(deg)
    return np.array([[ math.cos(r), 0, math.sin(r)],
                     [0,            1, 0           ],
                     [-math.sin(r), 0, math.cos(r)]])

def _Rz(deg):
    r = math.radians(deg)
    return np.array([[math.cos(r),-math.sin(r), 0],
                     [math.sin(r), math.cos(r), 0],
                     [0,           0,            1]])

def _make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


# ── ICP helpers ───────────────────────────────────────────────────────────────

def _with_normals(pcd, radius):
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
    )
    return pcd

def _icp_step(src, tgt, voxel, T_init):
    sd = _with_normals(src.voxel_down_sample(voxel), voxel * 2)
    td = _with_normals(tgt.voxel_down_sample(voxel), voxel * 2)
    return o3d.pipelines.registration.registration_icp(
        sd, td, voxel * 1.5, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )

def _multiscale_icp(src, tgt, T_init, voxels):
    T = T_init.copy()
    last = None
    for v in voxels:
        last = _icp_step(src, tgt, v, T)
        T = last.transformation
    return T, last.fitness, last.inlier_rmse


# ── main registration ─────────────────────────────────────────────────────────

_SEED_ROTATIONS = [
    ('Ry(-90°)', _Ry(-90)), ('Ry(+90°)', _Ry(90)),
    ('Rx(-90°)', _Rx(-90)), ('Rx(+90°)', _Rx(90)),
    ('Rz(-90°)', _Rz(-90)), ('Rz(+90°)', _Rz(90)),
    ('identity', np.eye(3)),
]

def align(src, tgt, voxel_fine, logger):
    """
    Find T such that T(src) best overlaps tgt.
    Returns (T_final, fitness, rmse).
    """
    ca = np.asarray(tgt.points).mean(axis=0)
    cb = np.asarray(src.points).mean(axis=0)

    # voxel schedule: 4× → 2× → 1× fine
    coarse_v  = voxel_fine * 4
    medium_v  = voxel_fine * 2

    logger.info('Testing rotation seeds (coarse pass) ...')
    best_fit, best_T = -1.0, None

    for label, R in _SEED_ROTATIONS:
        t_seed = ca - R @ cb
        T_seed = _make_T(R, t_seed)
        r = _icp_step(src, tgt, coarse_v, T_seed)
        logger.info(f'  {label:12s}  coarse fit={r.fitness:.4f}')
        if r.fitness > best_fit:
            best_fit = r.fitness
            best_T   = r.transformation

    logger.info(f'Best coarse seed: fitness={best_fit:.4f} — refining ...')

    T_final, fitness, rmse = _multiscale_icp(
        src, tgt, best_T,
        voxels=[coarse_v, medium_v, voxel_fine],
    )

    return T_final, fitness, rmse


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class ICPNode(Node):

    def __init__(self):
        super().__init__('icp_node')

        self.declare_parameter('scan_a',     os.path.join(SCAN_DIR, 'scan_lab_bp.ply'))
        self.declare_parameter('scan_b',     os.path.join(SCAN_DIR, 'scan_tilted.ply'))
        self.declare_parameter('voxel_fine', 0.05)

        path_a     = self.get_parameter('scan_a').value
        path_b     = self.get_parameter('scan_b').value
        voxel_fine = float(self.get_parameter('voxel_fine').value)

        self.pub_a      = self.create_publisher(PointCloud2, '/icp/scan_a',         10)
        self.pub_b_raw  = self.create_publisher(PointCloud2, '/icp/scan_b_raw',     10)
        self.pub_b_aln  = self.create_publisher(PointCloud2, '/icp/scan_b_aligned', 10)
        self.pub_merged = self.create_publisher(PointCloud2, '/icp/merged',         10)

        self._clouds = self._run(path_a, path_b, voxel_fine)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            'Publishing on  /icp/scan_a  /icp/scan_b_raw  '
            '/icp/scan_b_aligned  /icp/merged'
        )

    # ------------------------------------------------------------------
    def _run(self, path_a, path_b, voxel_fine):
        log = self.get_logger()

        log.info(f'Loading {path_a} ...')
        pcd_a = o3d.io.read_point_cloud(path_a)
        log.info(f'Loading {path_b} ...')
        pcd_b = o3d.io.read_point_cloud(path_b)
        log.info(f'A: {len(pcd_a.points)} pts   B: {len(pcd_b.points)} pts')

        T, fitness, rmse = align(pcd_b, pcd_a, voxel_fine, log)

        # ---- decode rotation ----
        yaw   = math.degrees(math.atan2(T[1,0], T[0,0]))
        pitch = math.degrees(math.atan2(-T[2,0], math.sqrt(T[2,1]**2+T[2,2]**2)))
        roll  = math.degrees(math.atan2(T[2,1], T[2,2]))
        log.info(
            f'Final ICP:  fitness={fitness:.4f}  rmse={rmse:.4f}\n'
            f'  Roll={roll:.1f}°  Pitch={pitch:.1f}°  Yaw={yaw:.1f}°\n'
            f'  Translation: x={T[0,3]:.3f} y={T[1,3]:.3f} z={T[2,3]:.3f} m'
        )

        # ---- apply transform to full-res scan B ----
        pcd_b_aln = o3d.geometry.PointCloud(pcd_b)
        pcd_b_aln.transform(T)

        pts_a   = np.asarray(pcd_a.points)
        pts_b   = np.asarray(pcd_b.points)
        pts_aln = np.asarray(pcd_b_aln.points)

        c_a    = _to_xyzrgb(pts_a,    100, 149, 237)   # blue
        c_braw = _to_xyzrgb(pts_b,    230,  80,  80)   # red
        c_baln = _to_xyzrgb(pts_aln,   80, 200, 120)   # green
        c_mrg  = c_a + c_baln                           # blue + green

        return c_a, c_braw, c_baln, c_mrg

    # ------------------------------------------------------------------
    def _publish(self):
        ca, cb, cal, cm = self._clouds
        self.pub_a.publish(_make_cloud(self, ca))
        self.pub_b_raw.publish(_make_cloud(self, cb))
        self.pub_b_aln.publish(_make_cloud(self, cal))
        self.pub_merged.publish(_make_cloud(self, cm))


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ICPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
