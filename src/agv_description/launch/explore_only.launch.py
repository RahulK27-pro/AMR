"""
explore_only.launch.py
=======================
Run AFTER sim_only.launch.py is fully up.

Starts: Nav2 (minimal) + explore_lite
NO Gazebo, NO SLAM (already running), NO RViz

This is a minimal, diagnostic-friendly launch that isolates Nav2 +
explore_lite from the simulation startup. It makes it easy to:
  - See exactly which Nav2 node fails
  - Restart Nav2 without restarting Gazebo/SLAM
  - Monitor output cleanly

Run in Terminal 2:
  source ~/agv_ws/install/setup.bash
  ros2 launch agv_description explore_only.launch.py

Check these things first:
  ros2 topic echo /scan --once            # LiDAR must be publishing
  ros2 run tf2_ros tf2_echo map base_link # map frame must exist

Watch for these success messages:
  [lifecycle_manager] All nodes are active and healthy
  [explore] Received costmap
  [explore] Sending goal: x=..., y=...   <-- robot is moving!
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    # ── Nav2 params — inline for transparency ─────────────────────────
    # Uses a fixed 40x40m global costmap with NO static_layer.
    # This means Nav2 starts instantly without waiting for /map from SLAM.
    # SLAM runs in the background and provides obstacle data via /scan.

    nav2_common = {'use_sim_time': True}

    controller_params = {
        'use_sim_time': True,
        'controller_frequency': 10.0,
        'min_x_velocity_threshold': 0.001,
        'min_y_velocity_threshold': 0.5,
        'min_theta_velocity_threshold': 0.001,
        'failure_tolerance': 0.3,
        'progress_checker_plugin': 'progress_checker',
        'goal_checker_plugins': ['general_goal_checker'],
        'controller_plugins': ['FollowPath'],

        'progress_checker': {
            'plugin': 'nav2_controller::SimpleProgressChecker',
            'required_movement_radius': 0.5,
            'movement_time_allowance': 15.0,
        },
        'general_goal_checker': {
            'plugin': 'nav2_controller::SimpleGoalChecker',
            'stateful': True,
            'xy_goal_tolerance': 0.35,
            'yaw_goal_tolerance': 0.35,
        },
        'FollowPath': {
            'plugin': 'dwb_core::DWBLocalPlanner',
            'min_vel_x': 0.0,
            'max_vel_x': 0.3,
            'max_vel_theta': 1.0,
            'min_speed_xy': 0.0,
            'max_speed_xy': 0.3,
            'min_speed_theta': 0.0,
            'acc_lim_x': 2.0,
            'acc_lim_theta': 3.2,
            'decel_lim_x': -2.0,
            'decel_lim_theta': -3.2,
            'vx_samples': 15,
            'vtheta_samples': 15,
            'sim_time': 1.5,
            'linear_granularity': 0.05,
            'angular_granularity': 0.025,
            'transform_tolerance': 1.0,
            'xy_goal_tolerance': 0.35,
            'trans_stopped_velocity': 0.25,
            'short_circuit_trajectory_evaluation': True,
            'stateful': True,
            'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle',
                        'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
            'BaseObstacle.scale': 0.02,
            'PathAlign.scale': 32.0,
            'PathDist.scale': 32.0,
            'GoalAlign.scale': 24.0,
            'GoalDist.scale': 24.0,
            'RotateToGoal.scale': 32.0,
            'RotateToGoal.slowing_factor': 5.0,
            'RotateToGoal.lookahead_time': -1.0,
        },
    }

    local_costmap_params = {
        'use_sim_time': True,
        'update_frequency': 5.0,
        'publish_frequency': 2.0,
        'global_frame': 'odom',
        'robot_base_frame': 'base_link',
        'rolling_window': True,
        'width': 5,
        'height': 5,
        'resolution': 0.05,
        'robot_radius': 0.25,
        'plugins': ['obstacle_layer', 'inflation_layer'],
        'obstacle_layer': {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'scan',
            'scan': {
                'topic': '/scan',
                'max_obstacle_height': 2.0,
                'clearing': True,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': 8.0,
                'raytrace_min_range': 0.0,
                'obstacle_max_range': 7.5,
                'obstacle_min_range': 0.0,
            },
        },
        'inflation_layer': {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'cost_scaling_factor': 3.0,
            'inflation_radius': 0.45,
        },
        'always_send_full_costmap': True,
    }

    global_costmap_params = {
        'use_sim_time': True,
        'update_frequency': 1.0,
        'publish_frequency': 1.0,
        'global_frame': 'map',
        'robot_base_frame': 'base_link',
        # KEY FIX: No static_layer. Fixed 40x40m grid.
        # Starts immediately without waiting for /map from SLAM.
        'rolling_window': False,
        'width': 40,
        'height': 40,
        'resolution': 0.05,
        'robot_radius': 0.25,
        'track_unknown_space': True,
        'plugins': ['obstacle_layer', 'inflation_layer'],
        'obstacle_layer': {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'scan',
            'scan': {
                'topic': '/scan',
                'max_obstacle_height': 2.0,
                'clearing': True,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': 12.0,
                'raytrace_min_range': 0.0,
                'obstacle_max_range': 11.5,
                'obstacle_min_range': 0.0,
            },
        },
        'inflation_layer': {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'cost_scaling_factor': 3.0,
            'inflation_radius': 0.45,
        },
        'always_send_full_costmap': True,
    }

    planner_params = {
        'use_sim_time': True,
        'planner_plugins': ['GridBased'],
        'GridBased': {
            'plugin': 'nav2_navfn_planner::NavfnPlanner',
            'tolerance': 0.5,
            'use_astar': False,
            'allow_unknown': True,
        },
    }

    bt_navigator_params = {
        'use_sim_time': True,
        'global_frame': 'map',
        'robot_base_frame': 'base_link',
        'odom_topic': '/odometry/filtered',
        'bt_loop_duration': 10,
        'default_server_timeout': 20,
        # NOTE: plugin_lib_names is intentionally OMITTED.
        # In ROS 2 Jazzy, BT node plugins are auto-registered.
        # Listing them manually causes "ID already registered" fatal crash.
        'navigate_to_pose': {
            'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator',
        },
        'navigate_through_poses': {
            'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator',
        },
    }

    behavior_params = {
        'use_sim_time': True,
        'local_costmap_topic': 'local_costmap/costmap_raw',
        'global_costmap_topic': 'global_costmap/costmap_raw',
        'local_footprint_topic': 'local_costmap/published_footprint',
        'global_footprint_topic': 'global_costmap/published_footprint',
        'cycle_frequency': 10.0,
        'behavior_plugins': ['spin', 'backup', 'wait'],
        'spin':   {'plugin': 'nav2_behaviors::Spin'},
        'backup': {'plugin': 'nav2_behaviors::BackUp'},
        'wait':   {'plugin': 'nav2_behaviors::Wait'},
        'local_frame': 'odom',
        'global_frame': 'map',
        'robot_base_frame': 'base_link',
        'transform_tolerance': 0.1,
        'simulate_ahead_time': 2.0,
        'max_rotational_vel': 1.0,
        'min_rotational_vel': 0.4,
        'rotational_acc_lim': 3.2,
    }

    waypoint_params = {
        'use_sim_time': True,
        'loop_rate': 20,
        'stop_on_failure': False,
        'waypoint_task_executor_plugin': 'wait_at_waypoint',
        'wait_at_waypoint': {
            'plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
            'enabled': True,
            'waypoint_pause_duration': 200,
        },
    }

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    # ── Nav2 Nodes ────────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_params, local_costmap_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params, global_costmap_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[waypoint_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 40.0,
            'node_names': lifecycle_nodes,
        }],
    )

    # ── explore_lite — starts 20s after Nav2 to ensure it is active ──
    explore_node = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[EXPLORE] Starting frontier exploration...'),
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_base_frame': 'base_link',
                    'costmap_topic': '/global_costmap/costmap',
                    'costmap_updates_topic': '/global_costmap/costmap_updates',
                    'visualize': True,
                    'planner_frequency': 0.5,
                    'progress_timeout': 30.0,
                    'potential_scale': 3.0,
                    'orientation_scale': 0.0,
                    'gain_scale': 1.0,
                    'transform_tolerance': 0.5,
                    'min_frontier_size': 0.5,
                }],
            ),
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[EXPLORE_ONLY] Starting Nav2 nodes...'),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        explore_node,
    ])
