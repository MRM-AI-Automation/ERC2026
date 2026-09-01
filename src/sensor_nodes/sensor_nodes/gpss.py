#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        # Publish GPS messages on /gps
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps',
            10
        )

        # Publish/update at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_gps)

        self.get_logger().info('GPS node started, publishing on /gps')

    def get_gps_data(self):
        """
        Replace this function with your actual GPS receiver code.

        It must return:
            latitude  -> degrees
            longitude -> degrees
            altitude  -> meters
            accuracy  -> horizontal accuracy in meters

        Return None if no valid GPS fix is available.
        """

        # EXAMPLE DATA ONLY — REPLACE THIS
        latitude = 50.0647
        longitude = 19.9450
        altitude = 200.0
        accuracy = 1.5

        return latitude, longitude, altitude, accuracy

    def publish_gps(self):

        data = self.get_gps_data()

        if data is None:
            self.get_logger().warning('No valid GPS fix')
            return

        latitude, longitude, altitude, accuracy = data

        msg = NavSatFix()

        # Timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        # GPS status
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude

        # Store accuracy as covariance.
        # Horizontal accuracy is represented in X and Y.
        variance = accuracy * accuracy

        msg.position_covariance[0] = variance   # Latitude/X variance
        msg.position_covariance[4] = variance   # Longitude/Y variance
        msg.position_covariance[8] = variance   # Altitude variance

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        # Publish
        self.gps_pub.publish(msg)

        # Log GPS position and accuracy
        self.get_logger().info(
            f'GPS | Lat: {latitude:.8f} | '
            f'Lon: {longitude:.8f} | '
            f'Alt: {altitude:.2f} m | '
            f'Accuracy: ±{accuracy:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)

    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
