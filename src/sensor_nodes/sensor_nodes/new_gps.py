#No RTCM, just GPS, publishes /fix and /gps_details

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import Serial, SerialException
from pyubx2 import UBXReader
import threading
import sys

from custom_msgs.msg import GpsDetails


class UbxParserNode(Node):
    def __init__(self):
        super().__init__('gps')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 38400)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        try:
            self.stream = Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to GPS on {port}")
        except SerialException as e:
            self.get_logger().error(f"GPS serial error: {e}")
            sys.exit(1)

        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.details_pub = self.create_publisher(GpsDetails, '/gps_details', 10)

        self.lat = None
        self.lon = None
        self.alt = None
        self.fix_type = 0
        self.sats = 0
        self.hacc = 0.0
        self.vacc = 0.0

        self.reader_thread = threading.Thread(
            target=self.read_loop, daemon=True)
        self.reader_thread.start()

        self.timer = self.create_timer(0.25, self.publish_data)

    def read_loop(self):
        ubr = UBXReader(self.stream)
        while rclpy.ok():
            try:
                _, msg = ubr.read()
                if msg and msg.identity == "NAV-PVT":
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7
                    self.alt = msg.hMSL / 1000.0
                    self.fix_type = msg.fixType
                    self.sats = msg.numSV
                    self.hacc = msg.hAcc / 1000.0
                    self.vacc = msg.vAcc / 1000.0

                    self.get_logger().info(
                        f"FixType={self.fix_type} "
                        f"Lat={self.lat:.7f} "
                        f"Lon={self.lon:.7f}",
                        throttle_duration_sec=2.0
                    )
            except Exception:
                pass

    def publish_data(self):
        if self.lat is None:
            return

        if self.fix_type >= 3:
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = "gps_link"
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude = self.lat
            fix.longitude = self.lon
            fix.altitude = self.alt
            fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.fix_pub.publish(fix)

        details = GpsDetails()
        details.latitude = self.lat
        details.longitude = self.lon
        details.altitude = self.alt
        details.fix_type = self.fix_type
        details.satellites = self.sats
        details.horizontal_accuracy = self.hacc
        details.vertical_accuracy = self.vacc
        self.details_pub.publish(details)


def main(args=None):
    rclpy.init(args=args)
    node = UbxParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
