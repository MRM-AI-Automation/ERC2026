from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed.launch.py'
            )
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        )
    )

    return LaunchDescription([
        zed,
        lidar,
        Node(package='rover_perception', executable='front_scan_filter', output='screen'),
        Node(package='planner', executable='relay_node', output='screen'),
        Node(package='sensor_nodes', executable='imu_node', output='screen'),
        Node(package='sensor_nodes', executable='gps_node', output='screen'),
    ])
