#!/usr/bin/env python3

import time
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

        # ------------------------------------------------------------
        # OPEN SERIAL PORT
        # ------------------------------------------------------------

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
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

        # Allow serial interface to stabilize
        time.sleep(0.5)

        # ------------------------------------------------------------
        # FORCE COLD START
        # ------------------------------------------------------------

        self.force_cold_start()

        # Receiver needs time to restart
        time.sleep(2.0)

        # Remove anything left in serial buffer before reset
        self.ser.reset_input_buffer()

        self.get_logger().info(
            'F9P restarted. Waiting for fresh GNSS acquisition...'
        )

        # ------------------------------------------------------------
        # ROS PUBLISHER
        # ------------------------------------------------------------

        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps',
            10
        )

        # ------------------------------------------------------------
        # GPS TIMER
        # ------------------------------------------------------------

        self.timer = self.create_timer(
            0.01,
            self.read_gps
        )

    # ================================================================
    # F9P COLD START
    # ================================================================

    def force_cold_start(self):

        self.get_logger().warn(
            'FORCING F9P COLD START'
        )

        # UBX-CFG-RST
        #
        # Sync       B5 62
        # Class      06
        # ID         04
        # Length     04 00
        #
        # Payload:
        #
        # navBbrMask = FF FF
        # resetMode  = 01
        # reserved   = 00
        #
        # FF FF = clear BBR / navigation data
        # 01    = controlled software reset
        #
        # Full UBX message:
        #
        # B5 62 06 04 04 00 FF FF 01 00 0D 5D

        cold_start_cmd = bytes([
            0xB5,
            0x62,
            0x06,
            0x04,
            0x04,
            0x00,
            0xFF,
            0xFF,
            0x01,
            0x00,
            0x0D,
            0x5D
        ])

        try:

            # Clear anything currently waiting
            self.ser.reset_input_buffer()

            # Send cold-start command
            self.ser.write(cold_start_cmd)
            self.ser.flush()

            self.get_logger().info(
                'Cold-start UBX command sent'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to send cold-start command: {e}'
            )

    # ================================================================
    # READ GPS
    # ================================================================

    def read_gps(self):

        try:

            while self.ser.in_waiting:

                line = self.ser.readline()

                # ----------------------------------------------------
                # IGNORE UBX / RTCM / BINARY DATA
                # ----------------------------------------------------

                if not line.startswith(b'$'):
                    continue

                try:

                    sentence = line.decode(
                        'ascii',
                        errors='ignore'
                    ).strip()

                    # ------------------------------------------------
                    # ONLY PROCESS GGA
                    # ------------------------------------------------

                    if not (
                        sentence.startswith('$GNGGA')
                        or
                        sentence.startswith('$GPGGA')
                    ):
                        continue

                    msg = pynmea2.parse(sentence)

                    # ------------------------------------------------
                    # CREATE NAVSATFIX
                    # ------------------------------------------------

                    gps = NavSatFix()

                    # Header
                    gps.header.stamp = (
                        self.get_clock().now().to_msg()
                    )

                    gps.header.frame_id = 'gps'

                    # ------------------------------------------------
                    # POSITION
                    # ------------------------------------------------

                    gps.latitude = float(msg.latitude)
                    gps.longitude = float(msg.longitude)
                    gps.altitude = float(msg.altitude)

                    # ------------------------------------------------
                    # FIX QUALITY
                    # ------------------------------------------------

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

                    # ------------------------------------------------
                    # FIX NAME
                    # ------------------------------------------------

                    fix_names = {
                        0: 'NO FIX',
                        1: 'GPS FIX',
                        2: 'DGPS/DGNSS FIX',
                        3: 'PPS FIX',
                        4: 'RTK FIXED',
                        5: 'RTK FLOAT',
                        6: 'DR',
                        7: 'MANUAL',
                        8: 'SIMULATION'
                    }

                    fix_name = fix_names.get(
                        fix_quality,
                        'UNKNOWN'
                    )

                    # ------------------------------------------------
                    # SATELLITES
                    # ------------------------------------------------

                    try:
                        satellites = int(msg.num_sats)
                    except (ValueError, TypeError):
                        satellites = 0

                    # ------------------------------------------------
                    # HDOP
                    # ------------------------------------------------

                    hdop = None

                    try:

                        if msg.horizontal_dil:
                            hdop = float(
                                msg.horizontal_dil
                            )

                    except (ValueError, TypeError):
                        hdop = None

                    # ------------------------------------------------
                    # POSITION COVARIANCE
                    # ------------------------------------------------

                    if (
                        hdop is not None
                        and
                        0.0 < hdop < 99.0
                    ):

                        gps.position_covariance[0] = (
                            hdop ** 2
                        )

                        gps.position_covariance[4] = (
                            hdop ** 2
                        )

                        gps.position_covariance[8] = (
                            (2.0 * hdop) ** 2
                        )

                        gps.position_covariance_type = (
                            NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                        )

                    else:

                        gps.position_covariance_type = (
                            NavSatFix.COVARIANCE_TYPE_UNKNOWN
                        )

                    # ------------------------------------------------
                    # PUBLISH
                    # ------------------------------------------------

                    self.gps_pub.publish(gps)

                    # ------------------------------------------------
                    # LOG
                    # ------------------------------------------------

                    self.get_logger().info(
                        f'LAT {gps.latitude:.8f} | '
                        f'LON {gps.longitude:.8f} | '
                        f'ALT {gps.altitude:.3f} m | '
                        f'SAT {satellites} | '
                        f'FIX {fix_quality} ({fix_name}) | '
                        f'HDOP {msg.horizontal_dil}'
                    )

                # ----------------------------------------------------
                # NMEA PARSE ERROR
                # ----------------------------------------------------

                except pynmea2.ParseError as e:

                    self.get_logger().warn(
                        f'NMEA parse error: {e}'
                    )

                # ----------------------------------------------------
                # OTHER NMEA ERROR
                # ----------------------------------------------------

                except Exception as e:

                    self.get_logger().warn(
                        f'NMEA error: {e}'
                    )

        # ------------------------------------------------------------
        # SERIAL ERROR
        # ------------------------------------------------------------

        except serial.SerialException as e:

            self.get_logger().error(
                f'Serial error: {e}'
            )

        except Exception as e:

            self.get_logger().error(
                f'GPS read error: {e}'
            )

    # ================================================================
    # CLEAN SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self.get_logger().info(
            'Closing F9P serial connection'
        )

        if hasattr(self, 'ser'):

            try:

                if self.ser.is_open:
                    self.ser.close()

            except Exception:
                pass

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = F9PGPSNode()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(f'GPS node error: {e}')

    finally:

        if node is not None:

            try:
                node.destroy_node()

            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
