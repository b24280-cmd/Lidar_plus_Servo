#!/usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Header
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformListener


class PointCloudBuilder(Node):

    def __init__(self):

        super().__init__('pointcloud_builder')

        # Accumulates points during the scan; locked into display_cloud on completion
        self.current_sweep  = []
        self.display_cloud  = []   # frozen after scan_complete
        self.scan_done      = False

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan',          self.scan_callback,     10)
        self.create_subscription(Bool,      '/scan_complete', self.scan_complete_cb,  10)

        self.cloud_pub = self.create_publisher(PointCloud2, '/full_cloud', 10)
        self.timer     = self.create_timer(0.5, self.publish_cloud)

        self.get_logger().info('PointCloud Builder started — accumulating scan...')

    # ------------------------------------------------------------------
    def scan_complete_cb(self, msg):
        """Called once when the servo finishes the full CENTER→MAX→CENTER→MIN→CENTER sweep."""
        self.display_cloud = self.current_sweep
        self.current_sweep = []
        self.scan_done     = True
        self.get_logger().info(
            f'Scan complete — {len(self.display_cloud)} points locked for display.'
        )

    # ------------------------------------------------------------------
    def scan_callback(self, scan):

        if self.scan_done:
            return   # sweep finished; ignore further scans

        try:
            tf = self.tf_buffer.lookup_transform('map', 'laser', rclpy.time.Time())
        except Exception:
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz,  2*qx*qy - 2*qz*qw,  2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,  1 - 2*qx*qx - 2*qz*qz,  2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,  2*qy*qz + 2*qx*qw,  1 - 2*qx*qx - 2*qy*qy],
        ])

        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max:
                p_map = R @ np.array([r * math.cos(angle), r * math.sin(angle), 0.0])
                p_map[0] += tx
                p_map[1] += ty
                p_map[2] += tz
                self.current_sweep.append([float(p_map[0]), float(p_map[1]), float(p_map[2])])
            angle += scan.angle_increment

    # ------------------------------------------------------------------
    def publish_cloud(self):

        # While scanning: show what's built so far.
        # After completion: show the locked cloud.
        pts = self.display_cloud if self.scan_done else self.current_sweep

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))


# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = PointCloudBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
