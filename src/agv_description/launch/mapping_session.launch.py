#!/usr/bin/env python3
"""
mapping_session.launch.py
=========================
Autonomous & manual AMR warehouse mapping session with SLAM Toolbox.

Arguments:
  world       — Name of world in worlds/ (e.g. 'test1.world') or absolute path. Default: 'test1.world'
  run_sim     — Start Gazebo (true) or connect to existing Gazebo (false). Default: true
  run_explore — Autonomous frontier explorer (true) or manual teleop driving (false). Default: false
  headless    — Run Gazebo without GUI. Default: false
  use_rviz    — Launch RViz2. Default: false

Usage:
  ros2 launch agv_description mapping_session.launch.py world:=test1.world
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_agv = get_package_share_directory('agv_description')

    urdf_file        = os.path.join(pkg_agv, 'urdf',   'warehouse_agv.urdf')
    bridge_file      = os.path.join(pkg_agv, 'config', 'bridge.yaml')
    ekf_file         = os.path.join(pkg_agv, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(pkg_agv, 'config', 'nav2_params_explore.yaml')
    rviz_config_file = os.path.join(pkg_agv, 'config', 'agv_explore.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # World file resolution
    world_val = context.perform_substitution(LaunchConfiguration('world')).strip()
    if not world_val.endswith('.world'):
        world_val += '.world'
    if os.path.isabs(world_val):
        world_file = world_val
    else:
        world_file = os.path.join(pkg_agv, 'worlds', world_val)

    headless_val = context.perform_substitution(LaunchConfiguration('headless')).lower() in ['true', '1']

    gz_cmd = ['gz', 'sim', '-r', '-v', '2', world_file]
    if headless_val:
        gz_cmd.insert(2, '-s')

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        condition=IfCondition(LaunchConfiguration('run_sim')),
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_file, {'use_sim_time': True}],
        output='screen'
    )

    spawn_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[MAPPING] t=5s: Spawning robot into Gazebo world...'),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='robot_spawner',
                arguments=[
                    '-name', 'warehouse_agv',
                    '-topic', 'robot_description',
                    '-z', '0.05'
                ],
                condition=IfCondition(LaunchConfiguration('run_sim')),
                output='screen'
            )
        ]
    )

    rsp_node = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[MAPPING] t=3s: Starting Robot State Publisher...'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True
                }],
                output='screen'
            )
        ]
    )

    slam_node = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[MAPPING] t=8s: Starting SLAM Toolbox (online async)...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py'
                    )
                ),
                launch_arguments={
                    'slam_params_file': os.path.join(pkg_agv, 'config', 'mapper_params.yaml'),
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[MAPPING] t=10s: Starting RViz2...'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                output='screen'
            )
        ]
    )

    nav2_controller = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[MAPPING] t=15s: Starting Nav2 nodes for exploration / navigation...'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                    ],
                    'bond_timeout': 40.0,
                }],
                output='screen'
            ),
        ]
    )

    explore_node = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='[MAPPING] Starting explore_lite autonomous explorer...'),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                parameters=[
                    nav2_params_file,
                    {'use_sim_time': True}
                ],
                condition=IfCondition(LaunchConfiguration('run_explore')),
                output='screen'
            )
        ]
    )

    return [
        LogInfo(msg=f'[MAPPING] Target world: {world_file}'),
        gazebo,
        bridge_node,
        ekf_node,
        spawn_node,
        rsp_node,
        slam_node,
        rviz_node,
        nav2_controller,
        explore_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='test1.world',
            description='World file name (in worlds/) or full path'
        ),
        DeclareLaunchArgument(
            'run_sim', default_value='true',
            description='Whether to start a new Gazebo simulation instance'
        ),
        DeclareLaunchArgument(
            'run_explore', default_value='false',
            description='Whether to run explore_lite for autonomous exploration (false = manual teleop mapping)'
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without GUI'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 for live map monitoring'
        ),
        OpaqueFunction(function=launch_setup),
    ])
