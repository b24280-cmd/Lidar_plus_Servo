#!/usr/bin/env python3

import open3d as o3d

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

SAVE_PATH = '/home/sam/Downloads/scan.ply'


class SaveCloud(Node):

    def __init__(self):

        super().__init__('save_cloud')

        self.scan_done = False
        self.saved     = False

        # Wait for the sweep to finish before grabbing the cloud
        self.create_subscription(Bool,         '/scan_complete', self.scan_complete_cb, 10)
        self.create_subscription(PointCloud2,  '/full_cloud',    self.cloud_callback,   10)

        self.get_logger().info('Waiting for scan to complete...')

    def scan_complete_cb(self, msg):
        self.scan_done = True
        self.get_logger().info('Scan complete — will save next cloud.')

    def cloud_callback(self, msg):

        if not self.scan_done or self.saved:
            return

        points = []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            points.append([float(p[0]), float(p[1]), float(p[2])])

        self.get_logger().info(f'Saving {len(points)} points to {SAVE_PATH} ...')

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(SAVE_PATH, pcd)

        self.get_logger().info(f'Saved → {SAVE_PATH}')

        self.saved = True
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)
    node = SaveCloud()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
