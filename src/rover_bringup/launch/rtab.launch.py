from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'],

            parameters=[{
                'frame_id': 'zed_camera_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',

                'rgb_frame_id': 'zed_left_camera_frame_optical',
                'depth_frame_id': 'zed_left_camera_frame_optical',

                'subscribe_rgb': True,
                'subscribe_depth': True,
                'subscribe_rgbd': False,
                'subscribe_imu': False,

                'approx_sync': True,
                'wait_for_transform': 1.2,
                'tf_delay': 0.5,
                'sync_queue_size': 20,
                'topic_queue_size': 20,
                'qos_camera_info': 1,

                'use_sim_time': False,

                'Rtabmap/DetectionRate': '15.0',
                'Rtabmap/TimeThr': '0',
                'Rtabmap/PublishOccupancyGrid': 'true',

                'RGBD/LoopClosureReextractFeatures': 'false',
                'RGBD/ProximityBySpace': 'false',
                'RGBD/ProximityByTime': '0',
                'RGBD/OptimizeFromGraphEnd': 'false',

                'Grid/Sensor': '1',
                'Grid/3D': 'true',
                'Grid/MapFrameProjection': 'true',

                'Grid/NormalK': '15',
                'Grid/MaxGroundAngle': '30',
                'Grid/GroundIsObstacle': 'false',
                'Grid/MinGroundHeight': '-0.25',
                'Grid/MaxObstacleHeight': '5.0',

                'Grid/RangeMin': '0.0',
                'Grid/RangeMax': '4.0',

                'Grid/UnknownSpaceFilled': 'false',

                'Grid/DepthDecimation': '2',
                'Grid/ObstacleFiltering': 'true',
                'Grid/NoiseFilteringRadius': '0.0',
                'Grid/NoiseFilteringMinNeighbors': '5',
                'Grid/MinClusterSize': '5',

                'Grid/CellSize': '0.10',

                'GridGlobal/FloodFillDepth': '12',
            }],

            remappings=[
                ('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
                ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
                ('odom', '/zed/zed_node/odom')
            ]
        )
    ])
