#!/usr/bin/env python3
"""
navigation_launch.py  –  Delivery Robot Navigation Stack
==========================================================
Launches all Nav2 nodes DIRECTLY (no nav2_bringup includes) to avoid the
RDSim workspace overlay that injects topology_map_server and breaks bringup.

Stack:
  Gazebo  →  robot_state_publisher  →  (spawn_entity)
  →  map_server  +  amcl  +  lifecycle_manager_localization
  →  controller / smoother / planner / behavior / bt_navigator /
     waypoint_follower / velocity_smoother / collision_monitor
  →  lifecycle_manager_navigation
  →  rviz2
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('delivery_robot')

    urdf_file   = os.path.join(pkg, 'urdf',   'robot.urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds', 'office.world')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg, 'maps',   'office_map.yaml')
    rviz_config = os.path.join(pkg, 'rviz',   'navigation.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_arg          = DeclareLaunchArgument('map',          default_value=default_map)
    use_rviz_arg     = DeclareLaunchArgument('use_rviz',     default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_arg       = DeclareLaunchArgument('params_file',  default_value=params_file)
    x_arg = DeclareLaunchArgument('x_pose', default_value='0.5')
    y_arg = DeclareLaunchArgument('y_pose', default_value='2.0')
    z_arg = DeclareLaunchArgument('z_pose', default_value='0.1')

    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    params       = LaunchConfiguration('params_file')

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
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

    # ── Nav2 – Localization ───────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ── Nav2 – Navigation stack ───────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz2 = TimerAction(
        period=10.0,
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

    # ── Start Nav2 after Gazebo has had time to boot ──────────────────────────
    nav2_start = TimerAction(
        period=5.0,
        actions=[
            map_server,
            amcl,
            lifecycle_manager_localization,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            collision_monitor,
            lifecycle_manager_navigation,
        ],
    )

    info_start = LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════╗\n'
        '║      DELIVERY ROBOT — NAVIGATION MODE                ║\n'
        '║  Map loaded, AMCL localizing…                        ║\n'
        '║  Send mission:                                       ║\n'
        '║  ros2 run delivery_robot delivery_mission room1 room2║\n'
        '╚══════════════════════════════════════════════════════╝'
    ))

    return LaunchDescription([
        map_arg, use_rviz_arg, use_sim_time_arg,
        params_arg, x_arg, y_arg, z_arg,
        info_start,
        gazebo,
        robot_state_pub,
        spawn_robot,
        nav2_start,
        rviz2,
    ])
