#!/usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node
from msgs.msg import ImuData


class BNO055Node(Node):
    def __init__(self):
        super().__init__('ex_imu_node')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.line = ""

        self.serial_port = None

        self.imu_pub_ = self.create_publisher(
            ImuData,
            '/imu_data',
            10
        )

        self.initialize_imu()

        # 100 Hz
        self.timer_ = self.create_timer(0.01, self.publish_imu)

    def initialize_imu(self):
        """Reset and initialize the IMU."""

        # Close an existing port before reopening it
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass

        try:
            self.serial_port = serial.Serial(
                '/dev/ttyUSB0',
                115200,
                timeout=1.0
            )

            time.sleep(1)

            self.get_logger().info("Connected to serial port")

            # Reset device
            self.serial_port.dtr = False
            self.serial_port.rts = True
            time.sleep(0.1)

            self.serial_port.rts = False
            time.sleep(0.1)

            self.serial_port.dtr = True

            self.serial_port.close()
            time.sleep(0.5)

            # Reconnect after reset
            self.serial_port = serial.Serial(
                '/dev/ttyUSB0',
                115200,
                timeout=1.0
            )

            time.sleep(1)

            self.get_logger().info("Waiting for valid IMU data...")

            while rclpy.ok():
                self.line = self.serial_port.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not self.line:
                    continue

                print(self.line)

                if self.line.startswith("2"):
                    break

            self.get_logger().info("IMU data usable")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            self.serial_port = None

    def publish_imu(self):

        # Reconnect if serial port is unavailable
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("IMU disconnected. Reinitializing...")
            self.initialize_imu()
            return

        self.line = self.serial_port.readline().decode(
            'utf-8',
            errors='ignore'
        ).strip()

        # Ignore empty lines
        if not self.line:
            return

        # Invalid packet
        if not self.line.startswith("2"):
            self.get_logger().warn(
                f"Invalid IMU packet: {self.line}"
            )
            return

        try:
            # Keep your original serial packet parsing:
            # Remove first two characters and final character
            data = self.line[2:-1]

            xcv = data.split("_")

            if len(xcv) < 3:
                raise ValueError("Expected roll, pitch and yaw data")

            # Convert correctly without using list.index()
            xcv = [float(value) for value in xcv]

            self.yaw = xcv[0]
            self.pitch = xcv[1]
            self.roll = xcv[2]

            print(
                f"r: {self.roll:7.2f} "
                f"p: {self.pitch:7.2f} "
                f"y: {self.yaw:7.2f}",
                end='\r'
            )

            imu_msg = ImuData()

            # Your custom message convention:
            # X = Roll
            # Y = Pitch
            # Z = Yaw
            imu_msg.orientation.x = self.roll
            imu_msg.orientation.y = self.pitch
            imu_msg.orientation.z = self.yaw

            imu_msg.acceleration.x = 0.0
            imu_msg.acceleration.y = 0.0
            imu_msg.acceleration.z = 0.0

            self.imu_pub_.publish(imu_msg)

        except (ValueError, IndexError) as e:
            self.get_logger().warn(
                f"Failed to parse IMU data '{self.line}': {e}"
            )

    def destroy_node(self):
        """Close serial port when the node exits."""

        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass

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

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
