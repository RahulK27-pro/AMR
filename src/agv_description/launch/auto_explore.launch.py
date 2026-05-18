#!/usr/bin/env python3
"""
auto_explore.launch.py
======================
Triggers the autonomous exploration node (explore_lite).
Run this AFTER you have started the environment with:
  ros2 launch agv_description explore_launch.py

Usage:
  ros2 launch agv_description auto_explore.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_agv = get_package_share_directory('agv_description')
    nav2_params_file = os.path.join(pkg_agv, 'config', 'nav2_params_explore.yaml')

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        explore_node
    ])
