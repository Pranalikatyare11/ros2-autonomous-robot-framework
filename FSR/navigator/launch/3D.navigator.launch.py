import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    fsr_dir = os.path.expanduser('~/FSR_WS/src/FSR')
    config_dir = os.path.join(fsr_dir, 'config')


    dd_nav2_params = os.path.join(config_dir, '3D_params.yaml')
    software_params = os.path.join(config_dir, 'software.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),


        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[dd_nav2_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[dd_nav2_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[dd_nav2_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[dd_nav2_params, software_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[dd_nav2_params,  {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[dd_nav2_params,  {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[dd_nav2_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[dd_nav2_params, software_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='navigator',
            executable='planner_switcher',
            name='planner_switcher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'collision_monitor',
                    'smoother_server',
                    'docking_server',
                    'waypoint_follower'
                ]
            }]
        )
    ])
