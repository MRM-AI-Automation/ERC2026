from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    navsat_params = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'navsat_transform.yaml'
    )

    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',

            parameters=[navsat_params],

            remappings=[
                ('gps/fix', '/fix'),
                ('imu', '/zed/zed_node/imu/data'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        )

    ])
