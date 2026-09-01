#!/usr/bin/env python3

import socket

import rclpy
from rclpy.node import Node

from msgs.msg import ImuData


class UDPIMUNode(Node):
    def __init__(self):
        super().__init__('udp_imu_node')

        # UDP configuration
        self.host = '0.0.0.0'
        self.port = 2002

        # Create publisher
        self.imu_pub = self.create_publisher(
            ImuData,
            '/imu_data',
            10
        )

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Allow restarting without socket cleanup issues
        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        # Listen on all network interfaces
        self.sock.bind((self.host, self.port))

        # Non-blocking so ROS does not freeze
        self.sock.setblocking(False)

        self.get_logger().info(
            f'Listening for UDP IMU data on {self.host}:{self.port}'
        )

        # Check UDP packets at 100 Hz
        self.timer = self.create_timer(0.01, self.receive_and_publish)

    def receive_and_publish(self):
        try:
            # Process all packets currently waiting
            while True:
                data, address = self.sock.recvfrom(4096)

                message = data.decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                try:
                    # Expected format:
                    # yaw_pitch_roll
                    values = message.split('_')

                    if len(values) != 3:
                        raise ValueError(
                            f'Expected 3 values, got {len(values)}'
                        )

                    yaw = float(values[0])
                    pitch = float(values[1])
                    roll = float(values[2])

                    # Normalize yaw just in case
                    yaw = yaw % 360.0

                    # Create IMU message
                    imu_msg = ImuData()

                    imu_msg.orientation.x = roll
                    imu_msg.orientation.y = pitch
                    imu_msg.orientation.z = yaw

                    imu_msg.acceleration.x = 0.0
                    imu_msg.acceleration.y = 0.0
                    imu_msg.acceleration.z = 0.0

                    # Publish to ROS
                    self.imu_pub.publish(imu_msg)

                    self.get_logger().info(
                        f'Published IMU | '
                        f'Roll: {roll:.2f} '
                        f'Pitch: {pitch:.2f} '
                        f'Yaw: {yaw:.2f}'
                    )

                except (ValueError, IndexError) as e:
                    self.get_logger().warn(
                        f'Invalid UDP IMU data: "{message}" ({e})'
                    )

        except BlockingIOError:
            # No packets currently available
            pass

        except Exception as e:
            self.get_logger().error(
                f'UDP receive error: {e}'
            )

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = UDPIMUNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
