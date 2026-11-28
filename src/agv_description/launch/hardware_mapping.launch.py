#!/usr/bin/env python3
"""
hardware_mapping.launch.py
==========================
Launches real-time SLAM Toolbox (online async) and RViz2 for physical AMR warehouse mapping.

Designed for physical hardware:
  - Clock source: System wall-clock (use_sim_time: false)
  - Frame topology: map -> odom (published dynamically by slam_toolbox)
  - LaserScan: /scan (from YDLIDAR X2 @ ~10 Hz)
  - Odometry: /odometry/filtered & /odom (from EKF & RMCS-2305 encoders @ 50 Hz)
  - Pre-configured RViz2 with Fixed Frame 'map', live OccupancyGrid, and laser scans.
  - Optional Hardware Frontier Exploration (run_explore:=true) using strict
    speed-capped nav2_params_hardware_explore.yaml + velocity_smoother + collision_monitor.

Usage:
  # On Laptop (subscribes to Pi over CycloneDDS Domain 7):
  ros2 launch agv_description hardware_mapping.launch.py

  # On Pi directly (headless without RViz):
  ros2 launch agv_description hardware_mapping.launch.py use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_agv = get_package_share_directory('agv_description')
    pkg_slam = get_package_share_directory('slam_toolbox')

    default_slam_params = os.path.join(pkg_agv, 'config', 'mapper_params_hardware.yaml')
    default_rviz_config = os.path.join(pkg_agv, 'config', 'agv_hardware_mapping.rviz')
    default_explore_params = os.path.join(pkg_agv, 'config', 'nav2_params_hardware_explore.yaml')

    # Fallback to source directory paths if share directory is not yet built
    ws_src_config = os.path.join(os.path.expanduser('~'), 'AMR', 'AMR-main', 'src', 'agv_description', 'config')
    if not os.path.exists(default_slam_params) and os.path.exists(os.path.join(ws_src_config, 'mapper_params_hardware.yaml')):
        default_slam_params = os.path.join(ws_src_config, 'mapper_params_hardware.yaml')
    if not os.path.exists(default_rviz_config) and os.path.exists(os.path.join(ws_src_config, 'agv_hardware_mapping.rviz')):
        default_rviz_config = os.path.join(ws_src_config, 'agv_hardware_mapping.rviz')
    if not os.path.exists(default_explore_params) and os.path.exists(os.path.join(ws_src_config, 'nav2_params_hardware_explore.yaml')):
        default_explore_params = os.path.join(ws_src_config, 'nav2_params_hardware_explore.yaml')

    # Arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true; system clock if false'
    )
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='Full path to SLAM Toolbox yaml parameters file'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz2 visualization'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='Full path to RViz configuration file'
    )
    declare_run_explore = DeclareLaunchArgument(
        'run_explore', default_value='false',
        description='Whether to launch speed-capped Nav2 + explore_lite for hardware exploration'
    )
    declare_launch_slam = DeclareLaunchArgument(
        'launch_slam', default_value='true',
        description='Whether to launch slam_toolbox (set false when attaching Nav2 + explore_lite dynamically)'
    )
    declare_explore_params = DeclareLaunchArgument(
        'explore_params_file', default_value=default_explore_params,
        description='Full path to hardware exploration Nav2/explore_lite yaml file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    run_explore = LaunchConfiguration('run_explore')
    launch_slam = LaunchConfiguration('launch_slam')
    explore_params_file = LaunchConfiguration('explore_params_file')

    # 1. SLAM Toolbox (Include official online_async_launch.py with Lifecycle Management & Autostart)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        condition=IfCondition(launch_slam),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false'
        }.items()
    )

    # 2. RViz2 Visualization
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Optional Hardware Frontier Exploration Stack (strictly gated by run_explore:=true)
    hw_nav2_explore_stack = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg='[HW_MAPPING] Starting speed-capped Nav2 stack (vx<=0.15 m/s, wz<=0.45 rad/s) + collision_monitor...'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=[explore_params_file, {'use_sim_time': False}],
                remappings=[('/cmd_vel', 'cmd_vel_nav')],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                parameters=[explore_params_file, {'use_sim_time': False}],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                parameters=[explore_params_file, {'use_sim_time': False}],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[explore_params_file, {'use_sim_time': False}],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[explore_params_file, {'use_sim_time': False}],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[explore_params_file, {'use_sim_time': False}],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                parameters=[explore_params_file, {'use_sim_time': False}],
                condition=IfCondition(run_explore),
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'velocity_smoother',
                        'collision_monitor',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                    ],
                    'bond_timeout': 15.0,
                }],
                condition=IfCondition(run_explore),
                output='screen',
            ),
        ]
    )

    hw_explore_node = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[HW_MAPPING] Starting explore_lite frontier explorer (use_sim_time:=false)...'),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                parameters=[explore_params_file, {'use_sim_time': False}],
                condition=IfCondition(run_explore),
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_slam_params,
        declare_use_rviz,
        declare_rviz_config,
        declare_run_explore,
        declare_launch_slam,
        declare_explore_params,
        LogInfo(msg="================================================================"),
        LogInfo(msg="  Starting AMR Physical Hardware SLAM Mapping Session           "),
        LogInfo(msg="  Fixed Frame: map (slam_toolbox dynamic scan matching)         "),
        LogInfo(msg="================================================================"),
        slam_toolbox_launch,
        rviz2_node,
        hw_nav2_explore_stack,
        hw_explore_node,
    ])
