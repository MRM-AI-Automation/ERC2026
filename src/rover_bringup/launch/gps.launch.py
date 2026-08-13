#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8

from serial import Serial
from pyubx2 import UBXReader


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('topic_name', '/fix')

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        topic_name = self.get_parameter('topic_name').value

        self.get_logger().info(
            f'Opening GNSS receiver on {serial_port} @ {baud_rate}'
        )

        try:
            self.ser = Serial(serial_port, baud_rate, timeout=1)
        except Exception as e:
            self.get_logger().fatal(
                f'Failed to open serial port {serial_port}: {e}'
            )
            sys.exit(1)

        self.fix_pub = self.create_publisher(
            NavSatFix,
            topic_name,
            10
        )

        self.status_pub = self.create_publisher(
            UInt8,
            '/gps_status',
            10
        )

        self.ubr = UBXReader(self.ser)

        self.msg_count = 0
        self.last_fix_logged = self.get_clock().now()

        self.timer = self.create_timer(
            0.05,
            self.read_gps
        )

        self.get_logger().info('GPS node started')

    def read_gps(self):
        try:
            _, msg = self.ubr.read()

            if msg is None:
                return

            if getattr(msg, "identity", "") != "NAV-PVT":
                return

            self.msg_count += 1

            fix = NavSatFix()

            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = "gps_link"

            fix.latitude = msg.lat / 1e7
            fix.longitude = msg.lon / 1e7
            fix.altitude = msg.hMSL / 1000.0

            fix.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            )

            self.fix_pub.publish(fix)

            status = UInt8()

            if msg.fixType < 3:
                status.data = 0      # No 3D fix
            else:
                status.data = 1      # Valid 3D fix

            self.status_pub.publish(status)

            if self.msg_count % 20 == 0:
                self.get_logger().info(
                    f'GPS Fix | '
                    f'Lat={fix.latitude:.7f}, '
                    f'Lon={fix.longitude:.7f}, '
                    f'Alt={fix.altitude:.2f} m, '
                    f'FixType={msg.fixType}, '
                    f'Satellites={msg.numSV}'
                )

        except Exception as e:
            self.get_logger().warn(
                f'GPS read error: {e}'
            )


def main():
    rclpy.init()

    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down GPS node')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
