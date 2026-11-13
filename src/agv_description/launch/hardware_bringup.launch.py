#!/usr/bin/env python3
"""
AMR Master Hardware Bringup Launch File
=======================================
Brings up the complete physical robot hardware on Raspberry Pi 5:
  1. AMR Hardware Drive Node   : Subscribes to /cmd_vel, drives motors, computes /odom
  2. AMR 6-Axis IMU Node       : Reads GY-91 MPU-6500 on I2C bus 1 (0x68), publishes /imu/data
  3. Robot State Publisher     : Broadcasts chassis & wheel TF frames (r=0.070m, L=0.240m)
  4. Robot Localization (EKF)  : Fuses /odom + /imu/data -> /odometry/filtered & broadcasts odom->base_link TF
  5. YDLIDAR X2 Driver         : Connects to /dev/ttyUSB0 (115200 baud) and publishes /scan

Verification Command:
  ros2 topic hz /scan /imu/data /odom /odometry/filtered
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_description')

    urdf_path = os.path.join(pkg_dir, 'urdf', 'warehouse_agv.urdf')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf_hardware.yaml')
    lidar_config_path = os.path.join(pkg_dir, 'config', 'ydlidar_x2.yaml')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # Launch Configurations & Arguments
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_ekf = LaunchConfiguration('enable_ekf')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_motors = LaunchConfiguration('enable_motors')

    declare_lidar = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Whether to launch YDLIDAR X2 driver node'
    )
    declare_ekf = DeclareLaunchArgument(
        'enable_ekf', default_value='true',
        description='Whether to launch robot_localization EKF filter'
    )
    declare_imu = DeclareLaunchArgument(
        'enable_imu', default_value='true',
        description='Whether to launch MPU-6500 IMU node'
    )
    declare_motors = DeclareLaunchArgument(
        'enable_motors', default_value='true',
        description='Whether to launch differential drive motor & odometry node'
    )

    # 1. Robot State Publisher (Static & Kinematic TFs from URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    # 2. AMR Hardware Drive & Odometry Node
    motor_drive_node = Node(
        package='agv_description',
        executable='amr_hardware_drive_node.py',
        name='amr_hardware_drive_node',
        output='screen',
        condition=IfCondition(enable_motors),
        parameters=[{
            'wheel_radius': 0.070,
            'wheel_separation': 0.240,
            'ticks_per_rev': 4713,
            'max_wheel_speed': 0.90,
            'min_pwm': 15.0,
            'max_pwm': 100.0,
            'odom_rate': 50.0,
            'cmd_vel_timeout': 1.5,
            'publish_tf': False,  # EKF publishes odom -> base_link
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': False
        }]
    )

    # 3. AMR 6-Axis IMU Node (MPU-6500 / GY-91)
    imu_node = Node(
        package='agv_description',
        executable='amr_imu_node.py',
        name='amr_imu_node',
        output='screen',
        condition=IfCondition(enable_imu),
        parameters=[{
            'i2c_bus': 1,
            'i2c_address': 0x68,
            'frame_id': 'imu_link',
            'rate': 50.0,
            'calibrate_gyro': True,
            'use_sim_time': False
        }]
    )

    # 4. Robot Localization EKF (Fuses /odom + /imu/data -> /odometry/filtered)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(enable_ekf),
        parameters=[ekf_config_path, {'use_sim_time': False}]
    )

    # 5. YDLIDAR X2 Driver Node (publishes /scan on /dev/ttyUSB0 @ 115200 baud)
    ydlidar_node = Node(
        package='agv_description',
        executable='amr_lidar_node.py',
        name='amr_lidar_node',
        output='screen',
        condition=IfCondition(enable_lidar),
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'frame_id': 'laser_frame',
            'range_min': 0.10,
            'range_max': 8.0,
            'num_bins': 360,
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        declare_lidar,
        declare_ekf,
        declare_imu,
        declare_motors,
        LogInfo(msg="=== Launching AMR Physical Hardware Stack (Pi 5) ==="),
        robot_state_publisher_node,
        motor_drive_node,
        imu_node,
        ekf_node,
        ydlidar_node
    ])

