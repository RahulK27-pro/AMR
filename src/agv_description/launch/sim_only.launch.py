"""
sim_only.launch.py
==================
Starts ONLY: Gazebo + Bridge + RSP + EKF + SLAM
NO Nav2, NO explore_lite, NO RViz

Run this in Terminal 1:
  source ~/agv_ws/install/setup.bash
  ros2 launch agv_description sim_only.launch.py

Verify everything is OK before launching explore_only.launch.py:
  ros2 topic echo /scan --once            # should print LiDAR data
  ros2 topic echo /odom --once            # should print odometry
  ros2 run tf2_ros tf2_echo map base_link # should print a transform
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg     = get_package_share_directory('agv_description')
    urdf    = os.path.join(pkg, 'urdf', 'warehouse_agv.urdf')
    world   = os.path.join(pkg, 'worlds', 'warehouse.world')
    bridge  = os.path.join(pkg, 'config', 'bridge.yaml')
    ekf     = os.path.join(pkg, 'config', 'ekf.yaml')
    slam_p  = os.path.join(pkg, 'config', 'mapper_params.yaml')

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    # ── 1. Gazebo ─────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world],
        output='screen'
    )

    # ── 2. Bridge (Gazebo ↔ ROS 2) ────────────────────────────────────
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge}'],
        output='screen'
    )

    # ── 3. Robot State Publisher ───────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # ── 4. Spawn robot in Gazebo ───────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'warehouse_agv', '-topic', 'robot_description', '-z', '0.05'],
        output='screen'
    )

    # ── 5. EKF (fuses /odom + /imu → /odometry/filtered) ─────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf, {'use_sim_time': True}]
    )

    # ── 6. SLAM Toolbox (publishes /map and map→odom TF) ─────────────
    # Delayed 5s to let Gazebo finish spawning the robot first
    slam_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[SIM] Starting SLAM Toolbox...'),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_p, {'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[SIM] Starting Gazebo + Bridge + RSP + EKF...'),
        gazebo,
        bridge_node,
        rsp,
        ekf_node,
        spawn,
        slam_node,
    ])
