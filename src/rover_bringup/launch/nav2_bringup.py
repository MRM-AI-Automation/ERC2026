from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'nav2_params.yaml'
    )

    rover_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_description'),
                'launch',
                'rover.launch.py'
            )
        )
    )

    local_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_bringup'),
                'launch',
                'local_ekf.launch.py'
            )
        )
    )

    global_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_bringup'),
                'launch',
                'global_ekf.launch.py'
            )
        )
    )

    navsat_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_bringup'),
                'launch',
                'navsat_transform.launch.py'
            )
        )
    )

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

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('rover_bringup'),
                'config',
                'scan_filter.yaml'
            )
        ],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_bringup'),
                'launch',
                'slam.launch.py'
            )
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        rover_description,
        local_ekf,
        global_ekf,
        navsat_transform,
        zed,
        lidar,
        scan_filter,
        slam,
        nav2
    ])
