#!/usr/bin/env python3
"""
explore_launch.py
=================
Launches the full autonomous exploration session:
  1. Gazebo + bridge + robot_state_publisher + EKF  (from gazebo.launch.py)
  2. SLAM Toolbox in online-async mode              (from slam_launch.py)
  3. Nav2 full stack in mapping/exploration mode    (nav2_bringup)
  4. explore_lite frontier explorer                 (m-explore-ros2)
  5. RViz2 pre-configured for exploration

Usage:
  ros2 launch agv_description explore_launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────
    pkg_agv  = get_package_share_directory('agv_description')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # ── Config files ──────────────────────────────────────────────────
    nav2_params_file  = os.path.join(pkg_agv, 'config', 'nav2_params_explore.yaml')
    slam_params_file  = os.path.join(pkg_agv, 'config', 'mapper_params.yaml')
    rviz_config_file  = os.path.join(pkg_agv, 'config', 'agv_explore.rviz')

    # ── Arguments ─────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for monitoring exploration'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo simulation headlessly (without GUI)'
    )

    # ── 1. Gazebo + Bridge + RSP + EKF ────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items()
    )

    # ── 2. SLAM Toolbox ───────────────────────────────────────────────
    # Delay slightly to let Gazebo start publishing topics first
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_agv, 'launch', 'slam_launch.py')
                )
            )
        ]
    )

    # ── 3. Nav2 Bringup (exploration mode — no static map) ────────────
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file':  nav2_params_file,
                    # No map — SLAM publishes /map live
                    'map': '',
                    'use_lifecycle_mgr': 'true',
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # ── 4. RViz2 ──────────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        headless_arg,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])

