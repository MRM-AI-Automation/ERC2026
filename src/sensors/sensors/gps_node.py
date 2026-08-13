#!/usr/bin/env python3

import serial
import pynmea2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.get_logger().info(
            f'Starting GPS node on {port} @ {baudrate} baud'
        )

        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1.0
            )

            self.get_logger().info('GPS serial connection opened')

        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open GPS serial port: {e}'
            )
            raise

        self.publisher = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )

        self.timer = self.create_timer(
            0.01,
            self.read_gps
        )

    def read_gps(self):

        try:
            line = self.serial.readline().decode(
                'ascii',
                errors='ignore'
            ).strip()

            if not line:
                return

            if not line.startswith('$'):
                return

            try:
                msg = pynmea2.parse(line)

            except pynmea2.ParseError:
                return

            # GGA contains position + fix information
            if isinstance(msg, pynmea2.types.talker.GGA):

                if msg.latitude is None or msg.longitude is None:
                    return

                fix = NavSatFix()

                fix.header.stamp = self.get_clock().now().to_msg()
                fix.header.frame_id = 'gps_link'

                # Fix status
                if msg.gps_qual is not None and int(msg.gps_qual) > 0:
                    fix.status.status = (
                        NavSatStatus.STATUS_FIX
                    )
                else:
                    fix.status.status = (
                        NavSatStatus.STATUS_NO_FIX
                    )

                fix.status.service = (
                    NavSatStatus.SERVICE_GPS
                )

                # Position
                fix.latitude = float(msg.latitude)
                fix.longitude = float(msg.longitude)

                # Altitude
                if msg.altitude:
                    fix.altitude = float(msg.altitude)
                else:
                    fix.altitude = 0.0

                self.publisher.publish(fix)

                self.get_logger().info(
                    f'GPS: '
                    f'lat={fix.latitude:.8f}, '
                    f'lon={fix.longitude:.8f}, '
                    f'alt={fix.altitude:.2f} m'
                )

        except serial.SerialException as e:
            self.get_logger().error(
                f'Serial error: {e}'
            )

        except Exception as e:
            self.get_logger().error(
                f'GPS processing error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = GPSNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
