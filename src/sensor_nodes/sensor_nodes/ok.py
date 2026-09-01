#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node


class UDPReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        self.host = '0.0.0.0'
        self.port = 2002

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Allow restarting the node without waiting for socket cleanup
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind to all interfaces on port 2002
        self.sock.bind((self.host, self.port))

        # Non-blocking socket so ROS doesn't freeze
        self.sock.setblocking(False)

        self.get_logger().info(
            f'Listening for UDP data on {self.host}:{self.port}'
        )

        # Check for incoming packets at 100 Hz
        self.timer = self.create_timer(0.01, self.receive_data)

    def receive_data(self):
        try:
            while True:
                data, address = self.sock.recvfrom(4096)

                try:
                    message = data.decode('utf-8').strip()
                except UnicodeDecodeError:
                    message = str(data)

                self.get_logger().info(
                    f'Received from {address[0]}:{address[1]} -> {message}'
                )

        except BlockingIOError:
            # No more packets available right now
            pass
        except Exception as e:
            self.get_logger().error(f'UDP receive error: {e}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = UDPReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
