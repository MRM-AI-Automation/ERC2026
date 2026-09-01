#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from msgs.msg import ImuData


def quaternion_to_euler(x, y, z, w):
    """Convert a quaternion to roll, pitch, and yaw in radians."""

    # Roll (rotation around X axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around Y axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (rotation around Z axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def to_clockwise_360(angle_rad):
    """
    Convert an angle from radians to degrees in the range [0, 360),
    with the numerical angle increasing clockwise.
    """
    return (-math.degrees(angle_rad)) % 360.0


class ImuRPYNode(Node):

    def __init__(self):
        super().__init__('imu_rpy_node')

        # ZED2i IMU input
        self.subscription = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10
        )

        # Custom ImuData output
        self.publisher = self.create_publisher(
            ImuData,
            '/imu_data',
            10
        )

        self.get_logger().info('IMU RPY converter started')

    def imu_callback(self, msg):
        # Extract quaternion
        q = msg.orientation

        # Convert quaternion to Roll, Pitch, Yaw
        roll, pitch, yaw = quaternion_to_euler(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # Create custom message
        output = ImuData()

        # Copy linear acceleration
        output.acceleration.x = msg.linear_acceleration.x
        output.acceleration.y = msg.linear_acceleration.y
        output.acceleration.z = msg.linear_acceleration.z

        # Convert angles to clockwise 0-360 degrees
        output.orientation.x = to_clockwise_360(roll)
        output.orientation.y = to_clockwise_360(pitch)
        output.orientation.z = to_clockwise_360(yaw)

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = ImuRPYNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
