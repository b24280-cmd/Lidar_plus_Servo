#!/usr/bin/env python3
#
# localize_node — scan-matching localization between two drone positions.
#
# Concept
# -------
# Both PLY scans are saved in the drone's own local "map" frame, so their
# XYZ coordinates are relative to where the drone was when the scan was taken.
#
# ICP finds transform T such that:
#
#     T · P_a  ≈  P_b
#
# T maps scan_a's frame → scan_b's frame, so:
#   T[:3, 3]   = position of scan_a's drone in scan_b's coordinate frame
#   T[:3, :3]  = orientation of scan_a's drone relative to scan_b's frame
#
# In RViz (fixed frame = "map" = scan_b's local frame) you will see:
#   • scan_b cloud (green)                    — where you are now
#   • scan_a cloud aligned by ICP (blue)      — the earlier environment overlaid
#   • a coordinate frame "drone_scan1"        — exactly where drone was in scan 1
#   • a pose arrow on /localize/drone_pose    — same, as a PoseStamped
#
# Parameters (set at launch, no rebuild needed)
#   scan_a        path to first PLY   (reference / earlier scan)
#   scan_b        path to second PLY  (query / current scan)
#   voxel_fine    finest ICP voxel in metres (default 0.05)
#
# Usage
# -----
#   ros2 run pointcloud_builder localize_node \
#     --ros-args -p scan_a:=/path/scan1.ply -p scan_b:=/path/scan2.ply

import math
import os
import struct

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster

SCAN_DIR = '/home/sam/Downloads'


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
    return np.array([[1, 0,           0          ],
                     [0, math.cos(r), -math.sin(r)],
                     [0, math.sin(r),  math.cos(r)]])

def _Ry(deg):
    r = math.radians(deg)
    return np.array([[ math.cos(r), 0, math.sin(r)],
                     [0,            1, 0           ],
                     [-math.sin(r), 0, math.cos(r)]])

def _Rz(deg):
    r = math.radians(deg)
    return np.array([[math.cos(r), -math.sin(r), 0],
                     [math.sin(r),  math.cos(r), 0],
                     [0,            0,            1]])

def _make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T

def _rot_to_quat(R):
    """3×3 rotation matrix → (qx, qy, qz, qw)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


# ── ICP (reused from icp_node) ────────────────────────────────────────────────

_SEED_ROTATIONS = [
    ('Ry(-90°)', _Ry(-90)), ('Ry(+90°)', _Ry(90)),
    ('Rx(-90°)', _Rx(-90)), ('Rx(+90°)', _Rx(90)),
    ('Rz(-90°)', _Rz(-90)), ('Rz(+90°)', _Rz(90)),
    ('identity', np.eye(3)),
]


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


def align(src, tgt, voxel_fine, logger):
    """Find T such that T(src) best overlaps tgt."""
    ca = np.asarray(tgt.points).mean(axis=0)
    cb = np.asarray(src.points).mean(axis=0)

    coarse_v = voxel_fine * 4
    medium_v = voxel_fine * 2

    logger.info('Testing rotation seeds ...')
    best_fit, best_T = -1.0, None

    for label, R in _SEED_ROTATIONS:
        t_seed = ca - R @ cb
        T_seed = _make_T(R, t_seed)
        r = _icp_step(src, tgt, coarse_v, T_seed)
        logger.info(f'  {label:12s}  coarse fitness={r.fitness:.4f}')
        if r.fitness > best_fit:
            best_fit = r.fitness
            best_T   = r.transformation

    logger.info(f'Best seed fitness={best_fit:.4f} — refining ...')
    T_final, fitness, rmse = _multiscale_icp(
        src, tgt, best_T,
        voxels=[coarse_v, medium_v, voxel_fine],
    )
    return T_final, fitness, rmse


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class LocalizeNode(Node):

    def __init__(self):
        super().__init__('localize_node')

        self.declare_parameter('scan_a',     os.path.join(SCAN_DIR, 'scan1.ply'))
        self.declare_parameter('scan_b',     os.path.join(SCAN_DIR, 'scan2.ply'))
        self.declare_parameter('voxel_fine', 0.05)

        path_a     = self.get_parameter('scan_a').value
        path_b     = self.get_parameter('scan_b').value
        voxel_fine = float(self.get_parameter('voxel_fine').value)

        # Publishers
        self.pub_b      = self.create_publisher(PointCloud2, '/localize/scan_b',        10)
        self.pub_a_aln  = self.create_publisher(PointCloud2, '/localize/scan_a_aligned', 10)
        self.pub_merged = self.create_publisher(PointCloud2, '/localize/merged',          10)
        self.pub_pose   = self.create_publisher(PoseStamped, '/localize/drone_pose',      10)
        self.pub_marker = self.create_publisher(Marker,      '/localize/drone_marker',    10)

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Run ICP
        self._clouds, self._T = self._run(path_a, path_b, voxel_fine)

        # Broadcast drone_scan1 TF once (static — pose doesn't change after ICP)
        self._broadcast_tf(self._T)

        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            'Publishing on:\n'
            '  /localize/scan_b          (green  — current scan)\n'
            '  /localize/scan_a_aligned  (blue   — earlier scan aligned)\n'
            '  /localize/merged          (both)\n'
            '  /localize/drone_pose      (PoseStamped — add Pose display in RViz)\n'
            '  /localize/drone_marker    (Marker axes — add Marker display in RViz)\n'
            'TF frame: map → drone_scan1'
        )

    # ------------------------------------------------------------------
    def _run(self, path_a, path_b, voxel_fine):
        log = self.get_logger()

        log.info(f'Loading scan_a (reference): {path_a}')
        pcd_a = o3d.io.read_point_cloud(path_a)
        log.info(f'Loading scan_b (query):     {path_b}')
        pcd_b = o3d.io.read_point_cloud(path_b)
        log.info(f'scan_a: {len(pcd_a.points)} pts   scan_b: {len(pcd_b.points)} pts')

        # Find T such that T · P_a ≈ P_b
        T, fitness, rmse = align(pcd_a, pcd_b, voxel_fine, log)

        # Decode T
        tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
        yaw   = math.degrees(math.atan2(T[1, 0], T[0, 0]))
        pitch = math.degrees(math.atan2(-T[2, 0], math.sqrt(T[2, 1]**2 + T[2, 2]**2)))
        roll  = math.degrees(math.atan2(T[2, 1], T[2, 2]))

        log.info(
            f'\n'
            f'ICP result:  fitness={fitness:.4f}  rmse={rmse:.4f}\n'
            f'\n'
            f'Drone position (scan 1) in scan 2 frame:\n'
            f'  x      = {tx:+.3f} m\n'
            f'  y      = {ty:+.3f} m\n'
            f'  z      = {tz:+.3f} m\n'
            f'\n'
            f'Drone orientation (scan 1) in scan 2 frame:\n'
            f'  roll   = {roll:+.2f}°\n'
            f'  pitch  = {pitch:+.2f}°\n'
            f'  yaw    = {yaw:+.2f}°\n'
            f'\n'
            f'Interpretation: during scan 1, the drone was {math.sqrt(tx**2+ty**2+tz**2):.3f} m\n'
            f'from where it was during scan 2, at yaw offset {yaw:+.2f}°.\n'
        )

        # Apply T to scan_a points
        pcd_a_aln = o3d.geometry.PointCloud(pcd_a)
        pcd_a_aln.transform(T)

        pts_b   = np.asarray(pcd_b.points)
        pts_aln = np.asarray(pcd_a_aln.points)

        c_b      = _to_xyzrgb(pts_b,    80, 200, 120)   # green  — current position
        c_a_aln  = _to_xyzrgb(pts_aln, 100, 149, 237)   # blue   — earlier scan aligned
        c_merged = c_b + c_a_aln

        return (c_b, c_a_aln, c_merged), T

    # ------------------------------------------------------------------
    def _broadcast_tf(self, T):
        """Broadcast map → drone_scan1 as a static TF."""
        qx, qy, qz, qw = _rot_to_quat(T[:3, :3])

        tf_msg = TransformStamped()
        tf_msg.header.stamp    = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id  = 'drone_scan1'
        tf_msg.transform.translation.x = float(T[0, 3])
        tf_msg.transform.translation.y = float(T[1, 3])
        tf_msg.transform.translation.z = float(T[2, 3])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------
    def _publish(self):
        c_b, c_a_aln, c_merged = self._clouds
        T = self._T

        self.pub_b.publish(_make_cloud(self, c_b))
        self.pub_a_aln.publish(_make_cloud(self, c_a_aln))
        self.pub_merged.publish(_make_cloud(self, c_merged))

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _rot_to_quat(T[:3, :3])

        # PoseStamped — shows as an arrow in RViz
        pose = PoseStamped()
        pose.header.stamp    = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(T[0, 3])
        pose.pose.position.y = float(T[1, 3])
        pose.pose.position.z = float(T[2, 3])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub_pose.publish(pose)

        # Marker (ARROW) — large enough to see orientation clearly
        marker = Marker()
        marker.header.stamp    = now
        marker.header.frame_id = 'map'
        marker.ns     = 'localize'
        marker.id     = 0
        marker.type   = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose   = pose.pose
        marker.scale.x = 0.5   # shaft length
        marker.scale.y = 0.05  # shaft diameter
        marker.scale.z = 0.05  # head diameter
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_marker.publish(marker)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LocalizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
