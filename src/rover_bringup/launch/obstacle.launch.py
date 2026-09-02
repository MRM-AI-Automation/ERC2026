from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    depth_to_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='depth_to_cloud',
        output='screen',
        remappings=[
            (
                'depth/image',
                '/zed/zed_node/depth/depth_registered'
            ),
            (
                'depth/camera_info',
                '/zed/zed_node/depth/camera_info'
            ),
            (
                'cloud',
                '/cloud'
            ),
        ],
        parameters=[{
            'decimation': 1,
        }]
    )

    obstacle_detector = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        remappings=[
            (
                'cloud',
                '/cloud'
            ),
            (
                'obstacles',
                '/obstacles'
            ),
        ],
        parameters=[{
            'Grid/3D': 'true',
            'Grid/CellSize': '0.10',
            'Grid/RangeMin': '0.2',
            'Grid/RangeMax': '5.0',
            'Grid/GroundIsObstacle': 'false',
            'Grid/MaxGroundAngle': '60',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinClusterSize': '10',
            'Grid/DepthDecimation': '2',
            'Grid/PreVoxelFiltering': 'true',
            'Grid/NormalsSegmentation': 'true',
            'frame_id': 'zed_left_camera_frame_optical',
            'wait_for_transform': 1.0,
            'qos': 0,
        }]
    )

    return LaunchDescription([
        depth_to_cloud,
        obstacle_detector,
    ])
mrmnavjet@ubun
