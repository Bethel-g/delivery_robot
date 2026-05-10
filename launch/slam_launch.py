#!/usr/bin/env python3
"""
slam_launch.py — Phase 1: SLAM Mapping
========================================
Launches:
  1. Gazebo (office world)
  2. robot_state_publisher  (broadcasts TF from URDF)
  3. spawn_entity           (inserts robot into Gazebo)
  4. slam_toolbox           (builds the map in real time)
  5. rviz2                  (visualisation)

Usage:
  ros2 launch delivery_robot slam_launch.py
  ros2 launch delivery_robot slam_launch.py use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────────
    pkg = get_package_share_directory('delivery_robot')
    slam_pkg = get_package_share_directory('slam_toolbox')

    urdf_file   = os.path.join(pkg, 'urdf',   'robot.urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds', 'office.world')
    slam_config = os.path.join(pkg, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz',   'slam.rviz')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualisation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    x_arg = DeclareLaunchArgument('x_pose', default_value='0.5',  description='Robot spawn X')
    y_arg = DeclareLaunchArgument('y_pose', default_value='2.0',  description='Robot spawn Y')
    z_arg = DeclareLaunchArgument('z_pose', default_value='0.1',  description='Robot spawn Z')

    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. Gazebo ─────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────
    robot_description = Command(['xacro ', urdf_file])

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

    # ── 3. Spawn robot (delayed to let Gazebo start) ──────────────────────
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

    # ── 4. slam_toolbox (async online mapping) ────────────────────────────
    slam = TimerAction(
        period=5.0,     # wait for robot to spawn and TF to stabilise
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

    # ── 5. RViz2 ─────────────────────────────────────────────────────────
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

    # ── Status messages ───────────────────────────────────────────────────
    info_start = LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════╗\n'
        '║       DELIVERY ROBOT — SLAM MAPPING MODE             ║\n'
        '║  Drive the robot with teleop_twist_keyboard to map.  ║\n'
        '║  Save map: ros2 run nav2_map_server map_saver_cli    ║\n'
        '║            -f <pkg>/maps/office_map                  ║\n'
        '╚══════════════════════════════════════════════════════╝'
    ))

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        x_arg, y_arg, z_arg,
        info_start,
        gazebo,
        robot_state_pub,
        spawn_robot,
        slam,
        rviz2,
    ])
