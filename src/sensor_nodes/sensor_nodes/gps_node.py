#!/usr/bin/env python3

import serial
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        self.port = '/dev/ttyACM0'
        self.baudrate = 115200

        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps',
            10
        )

        self.last_data_time = time.monotonic()
        self.last_fix_time = time.monotonic()

        self.signal_lost = False
        self.no_fix = False

        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )

            self.get_logger().info(
                f'GPS serial opened: {self.port} @ {self.baudrate}'
            )

        except serial.SerialException as e:
            self.get_logger().error(
                f'Failed to open GPS serial port: {e}'
            )
            raise

        self.read_timer = self.create_timer(
            0.01,
            self.read_gps
        )

        self.status_timer = self.create_timer(
            1.0,
            self.check_gps_status
        )

    def read_gps(self):

        try:

            while self.serial.in_waiting:

                line = self.serial.readline().decode(
                    'ascii',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                self.last_data_time = time.monotonic()

                if (
                    line.startswith('$GNGGA')
                    or line.startswith('$GPGGA')
                ):
                    self.parse_gga(line)

        except serial.SerialException as e:

            self.get_logger().error(
                f'GPS serial error: {e}'
            )

        except Exception as e:

            self.get_logger().error(
                f'GPS read error: {e}'
            )

    def parse_gga(self, line):

        try:

            data = line.split('*')[0]
            fields = data.split(',')

            if len(fields) < 10:
                return

            lat_raw = fields[2]
            lat_dir = fields[3]

            lon_raw = fields[4]
            lon_dir = fields[5]

            fix_type = int(fields[6]) if fields[6] else 0

            satellites = (
                int(fields[7])
                if fields[7]
                else 0
            )

            altitude = (
                float(fields[9])
                if fields[9]
                else 0.0
            )

            fix_name = self.get_fix_name(fix_type)

            # No GPS position
            if fix_type == 0 or not lat_raw or not lon_raw:

                if not self.no_fix:

                    self.get_logger().warn(
                        f'NO GPS FIX | '
                        f'Fix: {fix_name} ({fix_type}) | '
                        f'Satellites: {satellites}'
                    )

                    self.no_fix = True

                return

            # GPS fix recovered
            if self.no_fix:

                self.get_logger().info(
                    'GPS FIX RECOVERED'
                )

                self.no_fix = False

            latitude = self.nmea_to_decimal(
                lat_raw,
                lat_dir
            )

            longitude = self.nmea_to_decimal(
                lon_raw,
                lon_dir
            )

            msg = NavSatFix()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.header.frame_id = 'gps_link'

            msg.status.service = (
                NavSatStatus.SERVICE_GPS
            )

            msg.status.status = (
                NavSatStatus.STATUS_FIX
            )

            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude

            msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )

            self.gps_pub.publish(msg)

            self.last_fix_time = time.monotonic()

            self.get_logger().info(
                f'GPS | '
                f'Lat: {latitude:.8f} | '
                f'Lon: {longitude:.8f} | '
                f'Alt: {altitude:.3f} m | '
                f'Fix: {fix_name} ({fix_type}) | '
                f'Sats: {satellites}'
            )

        except (ValueError, IndexError) as e:

            self.get_logger().warn(
                f'Invalid GGA sentence: {e}'
            )

    def check_gps_status(self):

        elapsed = (
            time.monotonic() -
            self.last_data_time
        )

        # No serial GPS data for 2 seconds
        if elapsed > 2.0:

            if not self.signal_lost:

                self.get_logger().error(
                    'GPS SIGNAL LOST - '
                    'No data received from receiver'
                )

                self.signal_lost = True

        else:

            if self.signal_lost:

                self.get_logger().info(
                    'GPS SIGNAL RESTORED'
                )

                self.signal_lost = False

    @staticmethod
    def nmea_to_decimal(value, direction):

        if direction in ['N', 'S']:

            degrees = int(value[:2])
            minutes = float(value[2:])

        else:

            degrees = int(value[:3])
            minutes = float(value[3:])

        decimal = degrees + minutes / 60.0

        if direction in ['S', 'W']:
            decimal *= -1.0

        return decimal

    @staticmethod
    def get_fix_name(fix_type):

        fix_names = {
            0: 'NO FIX',
            1: 'GNSS FIX',
            2: 'DGPS',
            3: 'PPS',
            4: 'RTK FIXED',
            5: 'RTK FLOAT',
            6: 'DR',
            7: 'MANUAL',
            8: 'SIMULATION'
        }

        return fix_names.get(
            fix_type,
            'UNKNOWN'
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
