import time
import serial

import rclpy
from rclpy.node import Node

from aruco_msgs.msg import ImuData


class BNO055Node(Node):

    def __init__(self):
        super().__init__('ex_imu_node')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.serial_port = None
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200

        self.imu_pub_ = self.create_publisher(
            ImuData,
            '/external_imu',
            10
        )

        if not self.initialize_imu():
            self.get_logger().error(
                'Failed to initialize IMU on %s',
                self.port
            )

        self.timer_ = self.create_timer(
            0.01,
            self.publish_imu
        )

    def initialize_imu(self):

        try:
            # Close previous connection if it exists
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass

            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )

            time.sleep(1.0)

            self.get_logger().info(
                f'Opening IMU serial port: {self.port}'
            )

            # ESP32 reset sequence
            self.serial_port.dtr = False
            self.serial_port.rts = True

            time.sleep(0.1)

            self.serial_port.rts = False

            time.sleep(0.1)

            self.serial_port.dtr = True

            time.sleep(1.0)

            # Flush old data
            self.serial_port.reset_input_buffer()

            self.get_logger().info(
                'Waiting for valid IMU data...'
            )

            # Wait for a valid packet
            timeout = time.time() + 5.0

            while time.time() < timeout:

                raw = self.serial_port.readline()

                if not raw:
                    continue

                line = raw.decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                self.get_logger().debug(
                    f'IMU: {line}'
                )

                if line.startswith('2'):
                    self.get_logger().info(
                        'IMU yaw data usable'
                    )
                    return True

            self.get_logger().error(
                'Timed out waiting for IMU data'
            )

            return False

        except serial.SerialException as e:

            self.get_logger().error(
                f'Serial error: {e}'
            )

            self.serial_port = None

            return False

        except Exception as e:

            self.get_logger().error(
                f'IMU initialization error: {e}'
            )

            self.serial_port = None

            return False

    def publish_imu(self):

        if self.serial_port is None:
            return

        try:

            raw = self.serial_port.readline()

            if not raw:
                return

            line = raw.decode(
                'utf-8',
                errors='ignore'
            ).strip()

            # Ignore empty / invalid lines
            if not line:
                return

            if not line.startswith('2'):
                return

            # Expected format:
            #
            # 2<yaw>_<pitch>_<roll>...
            #
            # Example:
            # 212.34_1.25_-0.84

            data = line[2:-1]

            values = data.split('_')

            if len(values) < 3:
                self.get_logger().warning(
                    f'Invalid IMU packet: {line}'
                )
                return

            try:
                yaw = float(values[0])
                pitch = float(values[1])
                roll = float(values[2])

            except ValueError:

                self.get_logger().warning(
                    f'Could not parse IMU packet: {line}'
                )

                return

            self.yaw = yaw
            self.pitch = pitch
            self.roll = roll

            # Console output
            print(
                f'r: {self.roll:7.2f} '
                f'p: {self.pitch:7.2f} '
                f'y: {self.yaw:7.2f}',
                end='\r',
                flush=True
            )

            # ROS message
            imu_msg = ImuData()

            imu_msg.orientation.x = self.roll
            imu_msg.orientation.y = self.pitch
            imu_msg.orientation.z = self.yaw

            imu_msg.acceleration.x = 0.0
            imu_msg.acceleration.y = 0.0
            imu_msg.acceleration.z = 0.0

            self.imu_pub_.publish(imu_msg)

        except serial.SerialException as e:

            self.get_logger().error(
                f'IMU serial error: {e}'
            )

            try:
                self.serial_port.close()
            except Exception:
                pass

            self.serial_port = None

        except Exception as e:

            self.get_logger().error(
                f'IMU read error: {e}'
            )

    def destroy_node(self):

        if self.serial_port is not None:

            try:
                self.serial_port.close()
            except Exception:
                pass

            self.serial_port = None

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = BNO055Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
