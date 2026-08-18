import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    simulator_pkg = 'simulator'

    fsr_dir = os.path.expanduser('~/FSR_WS/src/FSR')
    config_dir = os.path.join(fsr_dir, 'config')

    controllers_yaml = os.path.join(config_dir, 'controllers.yaml')

    xacro_file = os.path.join(get_package_share_directory(simulator_pkg), '2D_description', 'ddrobot.xacro')
    world_file = os.path.join(get_package_share_directory(simulator_pkg), 'worlds', 'model.world')
    rviz_config_file = os.path.join(get_package_share_directory(simulator_pkg), 'rviz', '2DRviz.rviz')

    robot_description_config = xacro.process_file(xacro_file).toxml()

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': True}, controllers_yaml],
        output='screen',
    )

    spawn_entity_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'navigator_robot'],
            output='screen'
        )]
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=7.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen"
        )]
    )

    spawn_diff_drive_controller = TimerAction(
        period=9.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "-c", "/controller_manager"],
            output="screen"
        )]
    )

    teleop_node = Node(
        package='simulator',
        executable='teleop',
        name='teleop_keyboard',
        output='screen',
        remappings=[
        ],
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}]
    )


   
    rviz_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_file]
        )]
    )

    return LaunchDescription([
        gazebo,
        controller_manager_node,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller,
        teleop_node,
        rviz_node
    ])
