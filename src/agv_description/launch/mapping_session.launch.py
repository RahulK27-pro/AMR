#!/usr/bin/env python3
"""
mapping_session.launch.py
=========================
One-shot autonomous warehouse mapping session.

Startup timeline (all delays are conservative — do not reduce them):
  t= 0s  Gazebo physics + ros_gz_bridge + EKF node + robot spawn
  t= 3s  Robot State Publisher  (needs /clock flowing first)
  t= 8s  SLAM Toolbox           (lifecycle-aware via online_async_launch.py)
  t=10s  RViz2                  (agv_explore.rviz — map + scan + costmaps)
  t=15s  Nav2 nodes             (controller, planner, behavior, bt_navigator)
  t=15s  Nav2 lifecycle_manager (autostart=true, bond_timeout=40s)
  t=35s  explore_lite           (frontier exploration — after Nav2 is active)

After exploration is complete:
  Open a new terminal and run:
    bash ~/AMR/AMR-main/src/agv_description/scripts/save_map.sh

Usage:
  source ~/AMR/AMR-main/install/setup.bash
  ros2 launch agv_description mapping_session.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_agv  = get_package_share_directory('agv_description')

    # ── Config file paths ─────────────────────────────────────────────────────
    urdf_file        = os.path.join(pkg_agv, 'urdf',   'warehouse_agv.urdf')
    world_file       = os.path.join(pkg_agv, 'worlds', 'warehouse.world')
    bridge_file      = os.path.join(pkg_agv, 'config', 'bridge.yaml')
    ekf_file         = os.path.join(pkg_agv, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(pkg_agv, 'config', 'nav2_params_explore.yaml')
    rviz_config_file = os.path.join(pkg_agv, 'config', 'agv_explore.rviz')

    # Read URDF once — shared by RSP and spawn
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── Launch arguments ──────────────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI (useful for headless servers)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for live map monitoring'
    )

    # =========================================================================
    # t=0s — GAZEBO HARMONIC
    # =========================================================================
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_file],
        output='screen'
    )

    # =========================================================================
    # t=0s — BRIDGE (Gazebo ↔ ROS 2 topic translation)
    # =========================================================================
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # =========================================================================
    # t=0s — EKF (sensor fusion: /odom + /imu/data → /odometry/filtered)
    # =========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_file, {'use_sim_time': True}],
        output='screen'
    )

    # =========================================================================
    # t=5s — SPAWN robot into Gazebo
    #   Delayed to t=5s to ensure:
    #     1. Gazebo world has finished loading (takes 3-5s)
    #     2. RSP has started at t=3s and published /robot_description
    #   The create node waits for /robot_description before spawning,
    #   but it has a limited timeout — if Gazebo isn't ready it exits silently.
    # =========================================================================
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
                output='screen'
            )
        ]
    )

    # =========================================================================
    # t=3s — ROBOT STATE PUBLISHER
    #   Delayed to guarantee /clock is already flowing from Gazebo.
    #   RSP uses use_sim_time=true — if it starts before /clock arrives it will
    #   stamp all transforms at simulation time 0, corrupting the TF tree.
    # =========================================================================
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

    # =========================================================================
    # t=12s — SLAM Toolbox (lifecycle-aware)
    #   Delayed to t=12s: Gazebo(0s) + RSP(3s) + Spawn(5s) + sensor warmup(7s).
    #   SLAM needs a fully spawned robot with /scan already publishing.
    #   Uses online_async_launch.py which boots the Lifecycle Manager:
    #     Unconfigured → Configured → Active
    #   Only when Active does SLAM subscribe to /scan and publish /map + TF.
    #   DO NOT launch async_slam_toolbox_node directly — it stays dormant.
    # =========================================================================
    slam_node = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[MAPPING] t=12s: Starting SLAM Toolbox (lifecycle-aware)...'),
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

    # =========================================================================
    # t=13s — RViz2 (live mapping monitor)
    # =========================================================================
    rviz_node = TimerAction(
        period=13.0,
        actions=[
            LogInfo(msg='[MAPPING] t=13s: Starting RViz2...'),
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

    # =========================================================================
    # t=20s — NAV2 NODES (for autonomous exploration)
    #   Only the nodes needed for movement are started.
    #   map_server and amcl are NOT started — SLAM provides /map live.
    #   Params loaded from nav2_params_explore.yaml which has:
    #     - global_costmap: 100x100m fixed grid, origin (-50,-50)→(50,50)
    #     - local_costmap:  4m rolling window, obstacle layer from /scan
    #   This prevents the "costmap waits for /map" deadlock.
    # =========================================================================
    nav2_controller = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[MAPPING] t=20s: Starting Nav2 nodes for exploration...'),
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
            # Lifecycle manager — activates all Nav2 nodes above in sequence.
            # bond_timeout=40s: generous window for slow simulation startup.
            # autostart=true: transitions nodes to Active automatically.
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

    # =========================================================================
    # t=45s — EXPLORE LITE (frontier-based autonomous exploration)
    #   Started 25 seconds after Nav2 to guarantee:
    #     1. Nav2 lifecycle manager has activated all nodes
    #     2. /global_costmap/costmap is being published
    #     3. SLAM has processed enough scans to have a valid map frame
    #   explore_lite will then autonomously drive the robot to every
    #   unexplored region until the entire warehouse has been mapped.
    # =========================================================================
    explore_node = TimerAction(
        period=45.0,
        actions=[
            LogInfo(msg='[MAPPING] t=45s: Starting explore_lite frontier explorer...'),
            LogInfo(msg='[MAPPING] Watch for: [explore] Sending goal — robot is moving!'),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                parameters=[
                    nav2_params_file,
                    {'use_sim_time': True}
                ],
                output='screen'
            )
        ]
    )

    # =========================================================================
    # RETURN — ordered launch sequence
    # =========================================================================
    return LaunchDescription([
        headless_arg,
        use_rviz_arg,

        # t=0s
        LogInfo(msg='[MAPPING] t=0s:  Starting Gazebo + Bridge + EKF + Spawn...'),
        gazebo,
        bridge_node,
        ekf_node,
        spawn_node,

        # t=3s
        rsp_node,

        # t=8s
        slam_node,

        # t=10s
        rviz_node,

        # t=15s
        nav2_controller,

        # t=35s
        explore_node,

        LogInfo(msg='[MAPPING] All components scheduled. Full startup takes ~40 seconds.'),
        LogInfo(msg='[MAPPING] When exploration is complete, open a new terminal and run:'),
        LogInfo(msg='[MAPPING]   bash ~/AMR/AMR-main/src/agv_description/scripts/save_map.sh'),
    ])
