#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


CENTER = 127

# Sweep state machine
SWEEP_TO_MAX     = 0
RETURN_FROM_MAX  = 1
SWEEP_TO_MIN     = 2
RETURN_FROM_MIN  = 3
DONE             = 4


class ServoManager(Node):

    def __init__(self):

        super().__init__('servo_manager')

        # ---- parameters (change at launch, no rebuild needed) ----
        self.declare_parameter('center_angle', 127)  # PWM centre point for your servo
        self.declare_parameter('scan_offset', 28)   # degrees each side of center
        self.declare_parameter('sweep_speed', 10.0) # degrees per second
        self.declare_parameter('step_size', 1.0)    # degrees moved per timer tick (can be < 1)
        # map → base_link static translation (metres) and rotation (degrees)
        self.declare_parameter('base_x',     0.0)
        self.declare_parameter('base_y',     0.0)
        self.declare_parameter('base_z',     1.0)
        self.declare_parameter('base_roll',  0.0)
        self.declare_parameter('base_pitch', 0.0)
        self.declare_parameter('base_yaw',   0.0)
        # base_link → servo_link static translation (metres) and rotation (degrees)
        self.declare_parameter('servo_x',     0.0)
        self.declare_parameter('servo_y',     0.0)
        self.declare_parameter('servo_z',     -0.05)
        self.declare_parameter('servo_roll',  180.0)
        self.declare_parameter('servo_pitch', 0.0)
        self.declare_parameter('servo_yaw',   0.0)

        self.center      = self.get_parameter('center_angle').value
        scan_offset      = self.get_parameter('scan_offset').value
        sweep_speed      = self.get_parameter('sweep_speed').value
        self.step_size   = self.get_parameter('step_size').value
        self.base_x      = self.get_parameter('base_x').value
        self.base_y      = self.get_parameter('base_y').value
        self.base_z      = self.get_parameter('base_z').value
        self.base_roll   = math.radians(self.get_parameter('base_roll').value)
        self.base_pitch  = math.radians(self.get_parameter('base_pitch').value)
        self.base_yaw    = math.radians(self.get_parameter('base_yaw').value)
        self.servo_x     = self.get_parameter('servo_x').value
        self.servo_y     = self.get_parameter('servo_y').value
        self.servo_z     = self.get_parameter('servo_z').value
        self.servo_roll  = math.radians(self.get_parameter('servo_roll').value)
        self.servo_pitch = math.radians(self.get_parameter('servo_pitch').value)
        self.servo_yaw   = math.radians(self.get_parameter('servo_yaw').value)

        self.min_angle = self.center - scan_offset
        self.max_angle = self.center + scan_offset

        # ---- publishers / subscribers ----
        self.pub              = self.create_publisher(String, '/servo_command',  10)
        self.scan_complete_pub = self.create_publisher(Bool,  '/scan_complete',  10)

        self.create_subscription(String, '/servo_serial', self.servo_feedback_callback, 10)

        # ---- TF ----
        self.tf_broadcaster    = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tfs()

        # ---- sweep state machine ----
        self.angle     = float(self.center)
        self.direction = 1           # start by going toward MAX
        self.state     = SWEEP_TO_MAX

        # timer fires at sweep_speed / step_size Hz so speed stays correct
        self.timer = self.create_timer(self.step_size / sweep_speed, self.update)

        self.get_logger().info(
            f'ServoManager: center={self.center}  offset=±{scan_offset} deg  '
            f'speed={sweep_speed} deg/s  step={self.step_size} deg'
        )

    # ------------------------------------------------------------------
    def publish_static_tfs(self):

        now = self.get_clock().now().to_msg()

        t1 = TransformStamped()
        t1.header.stamp    = now
        t1.header.frame_id = 'map'
        t1.child_frame_id  = 'base_link'
        t1.transform.translation.x = self.base_x
        t1.transform.translation.y = self.base_y
        t1.transform.translation.z = self.base_z
        cr, sr = math.cos(self.base_roll  / 2), math.sin(self.base_roll  / 2)
        cp, sp = math.cos(self.base_pitch / 2), math.sin(self.base_pitch / 2)
        cy, sy = math.cos(self.base_yaw   / 2), math.sin(self.base_yaw   / 2)
        t1.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t1.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t1.transform.rotation.z = cr * cp * sy - sr * sp * cy
        t1.transform.rotation.w = cr * cp * cy + sr * sp * sy

        t2 = TransformStamped()
        t2.header.stamp    = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id  = 'servo_link'
        t2.transform.translation.x = self.servo_x
        t2.transform.translation.y = self.servo_y
        t2.transform.translation.z = self.servo_z
        # RPY → quaternion (ZYX convention: yaw applied first, then pitch, then roll)
        cr, sr = math.cos(self.servo_roll  / 2), math.sin(self.servo_roll  / 2)
        cp, sp = math.cos(self.servo_pitch / 2), math.sin(self.servo_pitch / 2)
        cy, sy = math.cos(self.servo_yaw   / 2), math.sin(self.servo_yaw   / 2)
        t2.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t2.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t2.transform.rotation.z = cr * cp * sy - sr * sp * cy
        t2.transform.rotation.w = cr * cp * cy + sr * sp * sy

        self.static_broadcaster.sendTransform([t1, t2])

    # ------------------------------------------------------------------
    def update(self):

        if self.state == DONE:
            return

        self.angle += self.direction * self.step_size

        # --- state transitions ---
        if self.state == SWEEP_TO_MAX and self.angle >= self.max_angle:
            self.angle     = self.max_angle
            self.direction = -1
            self.state     = RETURN_FROM_MAX

        elif self.state == RETURN_FROM_MAX and self.angle <= self.center:
            self.angle = self.center
            # direction stays -1, continue straight on to MIN
            self.state = SWEEP_TO_MIN

        elif self.state == SWEEP_TO_MIN and self.angle <= self.min_angle:
            self.angle     = self.min_angle
            self.direction = 1
            self.state     = RETURN_FROM_MIN

        elif self.state == RETURN_FROM_MIN and self.angle >= self.center:
            self.angle = self.center
            self.state = DONE
            self._send_command()          # park at centre
            self.timer.cancel()
            self.scan_complete_pub.publish(Bool(data=True))
            self.get_logger().info('Full scan complete — cloud locked.')
            return

        self._send_command()

    def _send_command(self):
        msg = String()
        msg.data = f'ANGLE:{int(round(self.angle))}'
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def servo_feedback_callback(self, msg):

        data = msg.data.strip()

        if not data.startswith('ANGLE_SET:'):
            return

        try:
            actual_angle = int(data.split(':')[1])
        except (IndexError, ValueError):
            return

        actual_angle = max(self.min_angle, min(self.max_angle, actual_angle))

        roll = math.radians(actual_angle - self.center)

        tf_msg = TransformStamped()
        tf_msg.header.stamp    = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'servo_link'
        tf_msg.child_frame_id  = 'laser'
        tf_msg.transform.translation.z = 0.05
        tf_msg.transform.rotation.x    = math.sin(roll / 2.0)
        tf_msg.transform.rotation.w    = math.cos(roll / 2.0)

        self.tf_broadcaster.sendTransform(tf_msg)


# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = ServoManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
