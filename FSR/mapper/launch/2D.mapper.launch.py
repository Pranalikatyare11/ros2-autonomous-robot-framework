#ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_param = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config', 'mapper_params_online_async.yaml'),
        description='SLAM Toolbox parameters file'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map')
        ]
    )

    
    return LaunchDescription([
        declare_slam_param,
        slam_toolbox_node,
    ])
