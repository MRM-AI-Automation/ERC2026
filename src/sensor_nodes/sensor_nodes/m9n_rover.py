#!/usr/bin/env python3

import socket
import serial
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus


class M9NRoverNode(Node):

    def __init__(self):
        super().__init__('m9n_rover_tcp')

        # =========================================================
        # CONFIGURATION
        # =========================================================

        # M9N connected to the Jetson
        self.serial_port = '/dev/ttyACM0'
        self.baudrate = 38400

        # F9P BASE NODE RUNNING ON LAPTOP
        # Laptop IP: 10.0.0.125
        self.tcp_host = '10.0.0.111'
        self.tcp_port = 5000

        # =========================================================
        # SERIAL CONNECTION TO M9N
        # =========================================================

        self.gnss = serial.Serial(
            self.serial_port,
            self.baudrate,
            timeout=0.02
        )

        # =========================================================
        # TCP / RTCM
        # =========================================================

        self.sock = None
        self.last_connect_attempt = 0.0

        self.rtcm_bytes_received = 0
        self.last_rtcm_bytes = 0

        # =========================================================
        # NMEA BUFFER
        # =========================================================

        self.nmea_buffer = bytearray()

        # =========================================================
        # NAVIGATION DATA
        # =========================================================

        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.fix_quality = 0
        self.num_satellites = 0
        self.hdop = None

        self.last_log = time.time()

        # =========================================================
        # ROS PUBLISHER
        # =========================================================

        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/GPS',
            10
        )

        # Run at 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.process
        )

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'M9N ROVER starting'
        )
        self.get_logger().info(
            f'M9N Serial: {self.serial_port}'
        )
        self.get_logger().info(
            'Jetson IP: 10.0.0.69'
        )
        self.get_logger().info(
            f'F9P Base TCP: {self.tcp_host}:{self.tcp_port}'
        )
        self.get_logger().info(
            '========================================'
        )

    # =============================================================
    # TCP CONNECTION
    # =============================================================

    def connect_tcp(self):

        # Don't retry continuously.
        if time.time() - self.last_connect_attempt < 2.0:
            return

        self.last_connect_attempt = time.time()

        sock = None

        try:

            self.get_logger().info(
                f'Connecting to F9P base at '
                f'{self.tcp_host}:{self.tcp_port}...'
            )

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            # Connection timeout
            sock.settimeout(2.0)

            sock.connect(
                (self.tcp_host, self.tcp_port)
            )

            # Switch to non-blocking after connecting
            sock.setblocking(False)

            self.sock = sock

            self.get_logger().info(
                'Connected to F9P RTCM server'
            )

        except Exception as e:

            self.get_logger().warning(
                f'RTCM connection failed: {e}'
            )

            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

            self.sock = None

    # =============================================================
    # DISCONNECT TCP
    # =============================================================

    def disconnect_tcp(self):

        if self.sock is not None:

            try:
                self.sock.close()
            except Exception:
                pass

        self.sock = None

    # =============================================================
    # RECEIVE RTCM FROM BASE
    # SEND RTCM DIRECTLY TO M9N
    # =============================================================

    def receive_rtcm(self):

        if self.sock is None:
            return

        try:

            while True:

                try:
                    data = self.sock.recv(4096)

                except BlockingIOError:
                    # No more data currently available
                    break

                if not data:

                    self.get_logger().warning(
                        'RTCM TCP connection closed by base'
                    )

                    self.disconnect_tcp()
                    return

                # -------------------------------------------------
                # IMPORTANT:
                # RTCM is binary data.
                # Send it unchanged directly to the M9N.
                # -------------------------------------------------

                self.gnss.write(data)

                self.rtcm_bytes_received += len(data)

        except (
            ConnectionResetError,
            BrokenPipeError,
            OSError
        ) as e:

            self.get_logger().warning(
                f'RTCM connection error: {e}'
            )

            self.disconnect_tcp()

    # =============================================================
    # READ GNSS / NMEA FROM M9N
    # =============================================================

    def read_gnss(self):

        if self.gnss.in_waiting <= 0:
            return

        data = self.gnss.read(
            self.gnss.in_waiting
        )

        if not data:
            return

        self.nmea_buffer.extend(data)

        # Process complete NMEA lines
        while b'\n' in self.nmea_buffer:

            line, _, self.nmea_buffer = (
                self.nmea_buffer.partition(b'\n')
            )

            try:

                sentence = line.decode(
                    'ascii',
                    errors='ignore'
                ).strip()

                self.parse_nmea(sentence)

            except Exception:
                pass

    # =============================================================
    # PARSE NMEA GGA
    # =============================================================

    def parse_nmea(self, sentence):

        if not sentence.startswith('$'):
            return

        # Accept:
        # GPGGA
        # GNGGA
        # GLGGA
        # etc.
        if sentence[3:6] != 'GGA':
            return

        # Remove checksum
        sentence = sentence.split('*')[0]

        fields = sentence.split(',')

        if len(fields) < 10:
            return

        try:

            # NMEA fix quality
            self.fix_quality = int(
                fields[6] or 0
            )

            self.num_satellites = int(
                fields[7] or 0
            )

            self.hdop = (
                float(fields[8])
                if fields[8]
                else None
            )

            # No valid position
            if self.fix_quality == 0:
                return

            lat = self.nmea_to_decimal(
                fields[2],
                fields[3]
            )

            lon = self.nmea_to_decimal(
                fields[4],
                fields[5]
            )

            altitude = (
                float(fields[9])
                if fields[9]
                else 0.0
            )

            if lat is None or lon is None:
                return

            self.latitude = lat
            self.longitude = lon
            self.altitude = altitude

            # Publish ROS GPS message
            self.publish_gps()

        except (
            ValueError,
            IndexError
        ):
            pass

    # =============================================================
    # NMEA COORDINATE CONVERSION
    # =============================================================

    def nmea_to_decimal(
        self,
        value,
        direction
    ):

        if not value:
            return None

        raw = float(value)

        degrees = int(raw / 100)

        minutes = (
            raw - degrees * 100
        )

        decimal = (
            degrees + minutes / 60.0
        )

        if direction in ('S', 'W'):
            decimal = -decimal

        return decimal

    # =============================================================
    # PUBLISH GPS
    # =============================================================

    def publish_gps(self):

        if (
            self.latitude is None
            or self.longitude is None
        ):
            return

        msg = NavSatFix()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'gps_link'

        if self.fix_quality > 0:

            msg.status.status = (
                NavSatStatus.STATUS_FIX
            )

        else:

            msg.status.status = (
                NavSatStatus.STATUS_NO_FIX
            )

        msg.status.service = (
            NavSatStatus.SERVICE_GPS
        )

        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude

        # HDOP is not a true covariance measurement.
        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )

        self.gps_pub.publish(msg)

    # =============================================================
    # STATUS LOGGING
    # =============================================================

    def log_status(self):

        now = time.time()

        if now - self.last_log < 1.0:
            return

        elapsed = now - self.last_log
        self.last_log = now

        bytes_since = (
            self.rtcm_bytes_received
            - self.last_rtcm_bytes
        )

        self.last_rtcm_bytes = (
            self.rtcm_bytes_received
        )

        rtcm_rate = bytes_since / elapsed

        # Standard NMEA GGA quality indicators
        quality_names = {
            0: 'NO FIX',
            1: 'GPS',
            2: 'DGPS',
            6: 'ESTIMATED'
        }

        quality = quality_names.get(
            self.fix_quality,
            f'UNKNOWN({self.fix_quality})'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'[TCP] '
            f'{"CONNECTED" if self.sock else "DISCONNECTED"}'
        )

        self.get_logger().info(
            f'[RTCM] '
            f'received={rtcm_rate:.0f} bytes/s'
        )

        if self.latitude is None:

            self.get_logger().info(
                '[M9N] Waiting for NMEA GGA...'
            )

        else:

            self.get_logger().info(
                f'[M9N] '
                f'fix={quality} | '
                f'sats={self.num_satellites} | '
                f'HDOP={self.hdop}'
            )

            self.get_logger().info(
                f'[POSITION] '
                f'lat={self.latitude:.7f} | '
                f'lon={self.longitude:.7f} | '
                f'alt={self.altitude:.2f} m'
            )

        self.get_logger().info(
            '========================================'
        )

    # =============================================================
    # MAIN LOOP
    # =============================================================

    def process(self):

        # Connect/reconnect to the F9P base
        if self.sock is None:

            self.connect_tcp()

        else:

            self.receive_rtcm()

        # Read NMEA from M9N
        self.read_gnss()

        # Log once per second
        self.log_status()

    # =============================================================
    # CLEANUP
    # =============================================================

    def destroy_node(self):

        try:
            self.disconnect_tcp()
            self.gnss.close()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = M9NRoverNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
