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

    icp_odom_parameters = {
        'odom_frame_id': 'odom',
        'guess_frame_id': 'odom',
        'OdomF2M/ScanSubtractRadius': '1.5',  # match voxel size
        'OdomF2M/ScanMaxSize': '10000'
    }

    rtabmap_parameters = {
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_scan_cloud': True,
        'use_action_for_goal': True,
        'odom_sensor_sync': True,

        # Adjusted for Velodyne-only setup
        'Mem/NotLinkedNodesKept': 'false',
        'Grid/RangeMin': '0.5',  # ignore laser scan points on the robot itself
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'true',

        'RGBD/OptimizeMaxError': '10.0',     # less strict for LiDAR
        'RGBD/LoopThr': '0.25',
        'Rtabmap/DetectionRate': '0.5',      # reduce loop closure frequency
    }

    shared_parameters = {
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,

        # ICP settings
        'Reg/Strategy': '1',                  # ICP registration
        'Reg/Force3DoF': 'false',             # full 6 DOF
        'Icp/VoxelSize': '0.1',               # finer voxel size
        'Icp/PointToPlane': 'true',           # best for Velodyne
        'Icp/MaxCorrespondenceDistance': '0.5',
        'Icp/Iterations': '30',
        'Icp/RangeMin': '0.5',
        'Icp/MaxTranslation': '2.0',
        'Icp/PointToPlaneGroundNormalsUp': '0.9',

        # Optimizer
        'Optimizer/Strategy': '1',            # g2o optimizer (more robust)

        'Mem/NotLinkedNodesKept': 'false',
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('scan_cloud', '/velodyne_points2'),  # Make sure your topic name matches here!
    ]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='true', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),

        DeclareLaunchArgument(
            'robot_ns', default_value='',
            description='Robot namespace.'),

        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Use camera for global loop closure / re-localization.'),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            namespace=robot_ns,
            parameters=[icp_odom_parameters, shared_parameters],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'warn']),

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters,
                        {
                            'Mem/IncrementalMemory': 'false',
                            'Mem/InitWMWithAllNodes': 'true',
                            'publish_map': 'true',
                            'Grid/3D': 'true',
                            'Grid/FromDepth': 'false',
                            'database_path': '/home/panu/.ros/rtabmap.db'
                        }],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters,
                        {"odometry_node_name": "icp_odometry"}],
            remappings=remappings),
    ])

              
  