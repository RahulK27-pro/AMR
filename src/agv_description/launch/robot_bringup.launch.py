#!/usr/bin/env python3
"""
robot_bringup.launch.py
=======================
Physical Robot Hardware Bringup for Autonomous Mobile Robot (AMR).
Designed for deployment on Raspberry Pi or onboard robot computers.

Starts:
  1. robot_state_publisher — URDF transforms with real wall-clock time (use_sim_time=false)
  2. tf2_ros static_transform_publisher — aliases 'lidar_link' <-> 'laser' for hardware lidar drivers
  3. robot_localization (ekf_node) — fuses real wheel odometry (/odom) + real IMU (/imu/data)
  4. Optional hardware LiDAR driver launcher hook

Usage:
  # Standard bringup on real robot (Raspberry Pi):
  ros2 launch agv_description robot_bringup.launch.py

  # Bringup with custom serial port:
  ros2 launch agv_description robot_bringup.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_agv = get_package_share_directory('agv_description')
    urdf_file = os.path.join(pkg_agv, 'urdf', 'warehouse_agv.urdf')
    ekf_file = os.path.join(pkg_agv, 'config', 'ekf.yaml')

    use_sim_time_val = LaunchConfiguration('use_sim_time').perform(context).strip().lower() in ('true', '1')
    run_ekf_val = LaunchConfiguration('run_ekf').perform(context).strip().lower() in ('true', '1')
    lidar_frame_val = LaunchConfiguration('lidar_frame').perform(context).strip()

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    print("=" * 65)
    print("  [AMR Physical Robot Hardware Bringup]")
    print(f"  URDF Description:  warehouse_agv.urdf")
    print(f"  use_sim_time:      {use_sim_time_val}")
    print(f"  EKF Fusion:        {run_ekf_val}")
    print(f"  LiDAR Frame Alias: lidar_link <-> {lidar_frame_val}")
    print("=" * 65)

    # 1. Robot State Publisher (Real Hardware Mode)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time_val}],
        output='screen'
    )

    # 2. Static Transform Publisher for Physical LiDAR
    # Connects 'lidar_link' (URDF) to the driver frame (default: 'laser') so
    # physical LiDAR data seamlessly attaches to the TF tree.
    lidar_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_alias',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'lidar_link', '--child-frame-id', lidar_frame_val],
        output='screen'
    )

    # 3. EKF Odometry Filter
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_file, {'use_sim_time': use_sim_time_val}],
        condition=IfCondition(LaunchConfiguration('run_ekf')),
        output='screen'
    )

    return [
        rsp_node,
        lidar_alias_tf,
        ekf_node,
    ]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Simulation time flag (always false on physical robot)'
    )

    run_ekf_arg = DeclareLaunchArgument(
        'run_ekf',
        default_value='true',
        description='Launch robot_localization EKF node for real wheel + IMU fusion'
    )

    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Frame ID published by physical LiDAR driver (aliased to lidar_link)'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor base or hardware LiDAR'
    )

    return LaunchDescription([
        use_sim_time_arg,
        run_ekf_arg,
        lidar_frame_arg,
        serial_port_arg,
        LogInfo(msg='[BRINGUP] Launching AMR physical robot core transforms and localization...'),
        OpaqueFunction(function=launch_setup),
    ])
