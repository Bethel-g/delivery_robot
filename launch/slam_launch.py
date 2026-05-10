#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg      = get_package_share_directory('delivery_robot')
    slam_pkg = get_package_share_directory('slam_toolbox')

    urdf_file   = os.path.join(pkg, 'urdf',   'robot.urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds', 'office.world')
    slam_config = os.path.join(pkg, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz',   'slam.rviz')

    use_rviz_arg     = DeclareLaunchArgument('use_rviz',     default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    x_arg = DeclareLaunchArgument('x_pose', default_value='0.5')
    y_arg = DeclareLaunchArgument('y_pose', default_value='2.0')
    z_arg = DeclareLaunchArgument('z_pose', default_value='0.1')

    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_delivery_bot',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'delivery_bot',
                    '-x', LaunchConfiguration('x_pose'),
                    '-y', LaunchConfiguration('y_pose'),
                    '-z', LaunchConfiguration('z_pose'),
                    '-Y', '0.0',
                ],
                output='screen',
            )
        ],
    )

    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_config,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ],
    )

    rviz2 = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
                output='screen',
            )
        ],
    )

    info_start = LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════╗\n'
        '║       DELIVERY ROBOT — SLAM MAPPING MODE             ║\n'
        '║  Drive with: ros2 run teleop_twist_keyboard          ║\n'
        '║                    teleop_twist_keyboard             ║\n'
        '╚══════════════════════════════════════════════════════╝'
    ))

    return LaunchDescription([
        use_rviz_arg, use_sim_time_arg,
        x_arg, y_arg, z_arg,
        info_start,
        gazebo,
        robot_state_pub,
        spawn_robot,
        slam,
        rviz2,
    ])
