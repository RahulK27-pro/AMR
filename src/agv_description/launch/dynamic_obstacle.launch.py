#!/usr/bin/env python3
"""
dynamic_obstacle.launch.py
==========================
Spawns and activates dynamic obstacles in Gazebo Harmonic for testing AMR avoidance.

Usage examples:
  1. Default Aisle Crossing Obstacle:
     ros2 launch agv_description dynamic_obstacle.launch.py

  2. Spawn at specific location with doorway blocker pattern:
     ros2 launch agv_description dynamic_obstacle.launch.py x:=3.0 y:=2.8 pattern:=doorway_blocker

  3. Spawn without autonomous controller (for manual teleop):
     ros2 launch agv_description dynamic_obstacle.launch.py run_controller:=false
     (In another terminal: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/dynamic_obstacle/cmd_vel)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_agv = get_package_share_directory('agv_description')
    obstacle_sdf = os.path.join(pkg_agv, 'models', 'dynamic_obstacle', 'model.sdf')

    # Arguments
    x_arg = DeclareLaunchArgument('x', default_value='1.8', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.5', description='Initial Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.57', description='Initial Yaw angle')
    name_arg = DeclareLaunchArgument('name', default_value='dynamic_obstacle_1', description='Model name in Gazebo')
    pattern_arg = DeclareLaunchArgument('pattern', default_value='aisle_crossing', description='Patrol pattern')
    speed_arg = DeclareLaunchArgument('speed', default_value='0.35', description='Speed in m/s')
    run_controller_arg = DeclareLaunchArgument('run_controller', default_value='true', description='Run automated controller')

    # Spawn obstacle into Gazebo Harmonic
    spawn_obstacle = Node(
        package='ros_gz_sim',
        executable='create',
        name='obstacle_spawner',
        arguments=[
            '-name', LaunchConfiguration('name'),
            '-file', obstacle_sdf,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    # Automated patrol controller (starts 2 seconds after spawn)
    patrol_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='agv_navigation',
                executable='dynamic_obstacle_manager',
                name='dynamic_obstacle_manager',
                parameters=[{
                    'pattern': LaunchConfiguration('pattern'),
                    'speed': LaunchConfiguration('speed'),
                }],
                condition=IfCondition(LaunchConfiguration('run_controller')),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        name_arg,
        pattern_arg,
        speed_arg,
        run_controller_arg,
        LogInfo(msg='[OBSTACLE] Spawning dynamic obstacle into Gazebo Harmonic...'),
        spawn_obstacle,
        patrol_controller,
    ])
