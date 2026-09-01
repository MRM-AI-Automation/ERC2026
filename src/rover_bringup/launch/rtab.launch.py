from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'],  # Enable database mode

            parameters=[{
                'frame_id': 'zed_camera_link',                     # Robot base frame
                'odom_frame_id': 'odom',                     # Odometry frame
                'map_frame_id': 'map',                       # Global map frame

                'rgb_frame_id': 'zed_left_camera_frame_optical',   # RGB optical frame
                'depth_frame_id': 'zed_left_camera_frame_optical', # Depth optical frame

                'subscribe_rgb': True,                       # Subscribe to RGB images
                'subscribe_depth': True,                     # Subscribe to depth images
                'subscribe_rgbd': False,                     # No pre-synced RGBD
                'subscribe_imu': False,                      # IMU disabled

                'approx_sync': True,                         # Approximate sync
                'wait_for_transform': 1.2,                   # TF wait timeout
                'tf_delay': 0.5,                              # TF delay compensation
                'sync_queue_size': 20,                       # Sync buffer size
                'topic_queue_size': 20,                      # ROS queue size
                'qos_camera_info': 1,                        # SensorData QoS

                'use_sim_time': False,                       # Use system time

                'Rtabmap/DetectionRate': '10.0',              # SLAM update rate
                'Rtabmap/TimeThr': '0',                      # No time limit
                'Rtabmap/PublishOccupancyGrid': 'true',      # Publish grid

                'Grid/Sensor': '1',                          # Depth sensor source

                'Grid/3D': 'true',                           # Enable 3D grid
                'Grid/MapFrameProjection': 'true',           # Project to map frame

                'Grid/NormalK': '30',                         # Normal estimation neighbors
                'Grid/MaxGroundAngle': '30',                 # Max ground slope
                'Grid/GroundIsObstacle': 'false',             # Ground drop = obstacle
                'Grid/MinGroundHeight': '-0.25',             # Min ground height
                'Grid/MaxObstacleHeight': '5.0',             # Max obstacle height

                'Grid/RangeMin': '0.0',                      # Min depth range
                'Grid/RangeMax': '4.0',                      # Max depth range

                'Grid/UnknownSpaceFilled': 'true',           # Unknown = occupied

                'Grid/DepthDecimation': '1',                 # No decimation
                'Grid/ObstacleFiltering': 'true',            # Enable filtering
                'Grid/NoiseFilteringRadius': '0.0',          # Disable radius filter
                'Grid/NoiseFilteringMinNeighbors': '5',      # Min neighbors
                'Grid/MinClusterSize': '8',                  # Min cluster size

                'Grid/CellSize': '0.10',                     # Grid resolution

                'GridGlobal/FloodFillDepth': '12',           # Fill unknown gaps
            }],

            remappings=[
                ('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
                ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered'),
                ('odom', '/zed/zed_node/odom')
            ]
        )
    ])




