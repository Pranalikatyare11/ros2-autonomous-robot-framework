#!/usr/bin/env python3
"""
Corrected ROS2 launch file for Velodyne + RTAB-Map (icp_odometry)
- Adjusted for Velodyne sparsity (libpointmatcher disabled)
- Safer ICP params for LiDAR
- Remapping expects topic '/velodyne_points' (change if needed)
- Use `guess_frame_id = base_link` for a more stable initial guess
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    robot_ns = LaunchConfiguration('robot_ns')
    use_camera = LaunchConfiguration('use_camera')

    # icp_odometry specific parameters
    icp_odom_parameters = {
        'odom_frame_id': 'odom',
        # Use base_link as the frame providing the initial guess to ICP (more stable for LiDAR)
        'guess_frame_id': 'base_link',
        # reduce radius subtracted around robot so we keep more points for matching
        'OdomF2M/ScanSubtractRadius': '0.7',
        'OdomF2M/ScanMaxSize': '10000',
    }

    # RTAB-Map parameters tuned for Velodyne-only setups
    rtabmap_parameters = {
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_scan_cloud': True,
        'use_action_for_goal': True,
        'odom_sensor_sync': True,

        # keep memory small for robot-only mapping
        'Mem/NotLinkedNodesKept': 'false',

        # Grid params (if using 3D grid)
        'Grid/RangeMin': '0.5',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'true',

        # relax RGBD strictness since we're LiDAR-only
        'RGBD/OptimizeMaxError': '10.0',
        'RGBD/LoopThr': '0.25',
        'Rtabmap/DetectionRate': '0.5',
    }

    # Shared parameters passed to both icp_odometry and rtabmap
    shared_parameters = {
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,

        # Registration strategy & ICP tuning for Velodyne
        'Reg/Strategy': '1',                   # ICP
        'Reg/Force3DoF': 'false',              # full 6DOF

        # IMPORTANT: disable PointMatcher (libpointmatcher) for sparse Velodyne scans
        'Icp/PM': 'false',

        # Voxel and correspondence tuning — keep more points (smaller voxel)
        'Icp/VoxelSize': '0.05',               # 5 cm voxel
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '1.5',# 1.5 m (use 2.0 for outdoor/long-range scenes)
        'Icp/Iterations': '30',
        'Icp/RangeMin': '0.5',
        'Icp/MaxTranslation': '2.0',
        'Icp/PointToPlaneGroundNormalsUp': '0.9',
        'Icp/DownsamplingStep': '1',

        # Optimizer
        'Optimizer/Strategy': '1',

        'Mem/NotLinkedNodesKept': 'false',
    }

    # Remap the scan topic to your Velodyne topic. Change if needed.
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        # default: velodyne_points (most drivers use this). If your driver publishes a different
        # topic, change the right-hand side accordingly (e.g. '/velodyne_points2').
        ('scan_cloud', '/velodyne_points2'),
    ]

    nodes = []

    # Arguments / launch-time declarations
    nodes.append(DeclareLaunchArgument(
        'use_sim_time', default_value='true', choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'))

    nodes.append(DeclareLaunchArgument(
        'localization', default_value='false', choices=['true', 'false'],
        description='Launch rtabmap in localization mode (a map should have been already created).'))

    nodes.append(DeclareLaunchArgument(
        'robot_ns', default_value='',
        description='Robot namespace.'))

    nodes.append(DeclareLaunchArgument(
        'use_camera', default_value='false',
        description='Use camera for global loop closure / re-localization.'))

    # icp_odometry node
    nodes.append(Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        namespace=robot_ns,
        parameters=[icp_odom_parameters, shared_parameters],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', 'warn']
    ))

    # rtabmap (SLAM) — normal mapping mode
    nodes.append(Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace=robot_ns,
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
        arguments=['-d']
    ))

    # rtabmap in localization-only mode (if requested)
    nodes.append(Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace=robot_ns,
        parameters=[rtabmap_parameters, shared_parameters,
                    {
                        'Mem/IncrementalMemory': 'true',
                        'Mem/InitWMWithAllNodes': 'true',
                        'publish_map': 'true',
                        'Grid/3D': 'true',
                        'Grid/FromDepth': 'false',
                    }],
        remappings=remappings
    ))

    # rtabmap_viz (optional GUI)
    nodes.append(Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        namespace=robot_ns,
        parameters=[rtabmap_parameters, shared_parameters,
                    {"odometry_node_name": "icp_odometry"}],
        remappings=remappings
    ))

    return LaunchDescription(nodes)
