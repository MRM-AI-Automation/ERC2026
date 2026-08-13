from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='log',  # don't print logs to terminal
            parameters=[
                '/home/mrmnavjet/ERC2026/traversal/src/rover_bringup/config/ekf_local.yaml'
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                'fatal'  # only show fatal errors
            ]
        )
    ])
