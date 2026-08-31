#!/usr/bin/env python3
"""
navigation_launch.py
====================
Launches the full navigation session using a pre-saved SLAM map:
  1. Gazebo + bridge + robot_state_publisher + EKF  (from gazebo.launch.py)
  2. nav2_map_server — loads the saved .pgm / .yaml map
  3. Nav2 full stack with AMCL for localisation
  4. RViz2 configured with Nav2 goal panel, costmaps, and AMCL particles

Usage:
  ros2 launch agv_description navigation_launch.py \
      map:=/home/usernamerahul/agv_ws/maps/warehouse_map.yaml

  If no map argument is provided it defaults to the path above.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────
    pkg_agv  = get_package_share_directory('agv_description')
    # pkg_nav2 removed — no longer using bringup_launch.py (caused dual /cmd_vel conflict)

    # ── Default map path ──────────────────────────────────────────────
    default_map = os.path.join(pkg_agv, 'maps', 'warehouse_map.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.join(
            os.path.expanduser('~'), 'AMR', 'AMR-main', 'src', 'agv_description', 'maps', 'warehouse_map.yaml'
        )

    # ── Config files ──────────────────────────────────────────────────
    nav2_params_file = os.path.join(pkg_agv, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_agv, 'config', 'agv_nav.rviz')

    # ── Arguments ─────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the saved map YAML file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'
    )

    # ── 1. Gazebo + Bridge + RSP + EKF ────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv, 'launch', 'gazebo.launch.py')
        )
    )

    # ── 2. Localization only: map_server + AMCL + lifecycle_manager ────
    #
    # controller_server (DWB) is intentionally NOT started here.
    # route_runner.py's custom MPPI is the sole owner of /cmd_vel.
    # Running DWB alongside route_runner causes interleaved conflicting
    # velocity commands (30 Hz total) — the primary circling bug source.
    localization_launch = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[
                    nav2_params_file,
                    {
                        'yaml_filename': LaunchConfiguration('map'),
                        'use_sim_time': True,
                    },
                ],
                output='screen',
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[nav2_params_file, {'use_sim_time': True}],
                remappings=[('/initialpose', '/initialpose')],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                    'bond_timeout': 0.0,  # 0.0 = disable bond timeout to prevent premature shutdowns during sim startup
                }],
                output='screen',
            ),
        ]
    )

    # ── 3. RViz2 with Nav2 panels ─────────────────────────────────────
    rviz_node = TimerAction(
        period=7.0,
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
        map_arg,
        use_rviz_arg,
        gazebo_launch,
        localization_launch,   # map_server + amcl only (no DWB — MPPI owns /cmd_vel)
        rviz_node,
    ])
