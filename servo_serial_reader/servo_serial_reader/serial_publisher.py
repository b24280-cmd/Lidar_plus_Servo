import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')

        # Publish Arduino output to ROS
        self.publisher_ = self.create_publisher(
            String,
            '/servo_serial',
            10
        )

        # Receive ROS commands and send to Arduino
        self.command_subscriber = self.create_subscription(
            String,
            '/servo_command',
            self.command_callback,
            10
        )

        self.ser = serial.Serial(
            '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            115200,
            timeout=0.1
        )

        self.timer = self.create_timer(
            0.01,
            self.read_serial
        )

        self.get_logger().info("Listening to Arduino...")

    def command_callback(self, msg):

        command = msg.data + '\n'

        self.ser.write(command.encode())

        self.get_logger().info(
            f"Sent to Arduino: {msg.data}"
        )

    def read_serial(self):

        while self.ser.in_waiting:

            line = self.ser.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if line:

                msg = String()
                msg.data = line

                self.publisher_.publish(msg)

                self.get_logger().info(
                    f"Published: {line}"
                )


def main(args=None):

    rclpy.init(args=args)

    node = SerialPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
