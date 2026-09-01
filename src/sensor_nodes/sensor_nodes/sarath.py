#!/usr/bin/env python3

import serial
import pynmea2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class F9PGPSNode(Node):

    def __init__(self):
        super().__init__('f9p_gps_node')

        self.port = '/dev/ttyACM0'
        self.baud = 115200

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.1
            )

            self.get_logger().info(
                f'F9P connected: {self.port} @ {self.baud}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to open GPS: {e}'
            )
            raise

        # Publish on /gps
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps',
            10
        )

        self.timer = self.create_timer(
            0.01,
            self.read_gps
        )

    def read_gps(self):

        try:

            while self.ser.in_waiting:

                line = self.ser.readline()

                # Ignore UBX / RTCM binary data
                if not line.startswith(b'$'):
                    continue

                try:

                    sentence = line.decode(
                        'ascii',
                        errors='ignore'
                    ).strip()

                    # Only GGA
                    if not sentence.startswith('$GNGGA'):
                        continue

                    msg = pynmea2.parse(sentence)

                    gps = NavSatFix()

                    # Header
                    gps.header.stamp = (
                        self.get_clock().now().to_msg()
                    )

                    gps.header.frame_id = 'gps'

                    # Position
                    gps.latitude = msg.latitude
                    gps.longitude = msg.longitude
                    gps.altitude = float(msg.altitude)

                    # Fix
                    fix_quality = int(msg.gps_qual)

                    if fix_quality == 0:
                        gps.status.status = (
                            NavSatStatus.STATUS_NO_FIX
                        )
                    else:
                        gps.status.status = (
                            NavSatStatus.STATUS_FIX
                        )

                    gps.status.service = (
                        NavSatStatus.SERVICE_GPS
                    )

                    # HDOP
                    if msg.horizontal_dil:

                        hdop = float(
                            msg.horizontal_dil
                        )

                        gps.position_covariance[0] = hdop ** 2
                        gps.position_covariance[4] = hdop ** 2
                        gps.position_covariance[8] = (
                            (2.0 * hdop) ** 2
                        )

                        gps.position_covariance_type = (
                            NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                        )

                    # Publish
                    self.gps_pub.publish(gps)

                    self.get_logger().info(
                        f'LAT {gps.latitude:.8f} | '
                        f'LON {gps.longitude:.8f} | '
                        f'ALT {gps.altitude:.3f} m | '
                        f'SAT {msg.num_sats} | '
                        f'FIX {fix_quality} | '
                        f'HDOP {msg.horizontal_dil}'
                    )

                except Exception as e:
                    self.get_logger().warn(
                        f'NMEA error: {e}'
                    )

        except Exception as e:
            self.get_logger().error(
                f'Serial error: {e}'
            )

    def destroy_node(self):

        if hasattr(self, 'ser'):
            self.ser.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = F9PGPSNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
