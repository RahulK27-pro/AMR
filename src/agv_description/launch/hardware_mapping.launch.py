#!/usr/bin/env python3
"""
hardware_mapping.launch.py
==========================
Launches real-time SLAM Toolbox (online async) and RViz2 for physical AMR warehouse mapping.

Designed for physical hardware:
  - Clock source: System wall-clock (use_sim_time: false)
  - Frame topology: map -> odom (published dynamically by slam_toolbox)
  - LaserScan: /scan (from YDLIDAR X2 @ 24 Hz)
  - Odometry: /odometry/filtered & /odom (from EKF & RMCS-2305 encoders @ 50 Hz)
  - Pre-configured RViz2 with Fixed Frame 'map', live OccupancyGrid, and laser scans.

Usage:
  # On Laptop (subscribes to Pi over CycloneDDS Domain 7):
  ros2 launch agv_description hardware_mapping.launch.py

  # On Pi directly (headless without RViz):
  ros2 launch agv_description hardware_mapping.launch.py use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_agv = get_package_share_directory('agv_description')

    default_slam_params = os.path.join(pkg_agv, 'config', 'mapper_params_hardware.yaml')
    default_rviz_config = os.path.join(pkg_agv, 'config', 'agv_hardware_mapping.rviz')

    # Fallback to source directory paths if share directory is not yet built
    ws_src_config = os.path.join(os.path.expanduser('~'), 'AMR', 'AMR-main', 'src', 'agv_description', 'config')
    if not os.path.exists(default_slam_params) and os.path.exists(os.path.join(ws_src_config, 'mapper_params_hardware.yaml')):
        default_slam_params = os.path.join(ws_src_config, 'mapper_params_hardware.yaml')
    if not os.path.exists(default_rviz_config) and os.path.exists(os.path.join(ws_src_config, 'agv_hardware_mapping.rviz')):
        default_rviz_config = os.path.join(ws_src_config, 'agv_hardware_mapping.rviz')

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

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # 1. SLAM Toolbox (Async Online Mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 2. RViz2 Visualization (Fixed Frame: map)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_slam_params,
        declare_use_rviz,
        declare_rviz_config,
        LogInfo(msg="================================================================"),
        LogInfo(msg="  Starting AMR Physical Hardware SLAM Mapping Session           "),
        LogInfo(msg="  Fixed Frame: map -> odom (slam_toolbox dynamic scan matching) "),
        LogInfo(msg="================================================================"),
        slam_toolbox_node,
        rviz2_node
    ])
