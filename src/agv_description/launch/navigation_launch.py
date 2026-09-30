#!/usr/bin/env python3
"""
navigation_launch.py
====================
Launches the full navigation session with configurable map and matching Gazebo world:
  1. Gazebo + bridge + robot_state_publisher + EKF (from gazebo.launch.py)
  2. nav2_map_server — loads the selected map YAML
  3. Nav2 AMCL localization
  4. RViz2 configured with Nav2 goal panel and AMCL particles
  5. Automatic synchronization of graph roadmap for route_runner and bt_manager

Usage Examples:
  # Launch with warehouse_01 map & test1.world (default):
  ros2 launch agv_description navigation_launch.py map:=warehouse_01

  # Launch with warehouse_map & warehouse.world:
  ros2 launch agv_description navigation_launch.py map:=warehouse_map

  # Custom world override:
  ros2 launch agv_description navigation_launch.py map:=warehouse_01 world:=test1.world
"""

import os
import shutil
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_agv = get_package_share_directory('agv_description')
    ws_maps_dir = os.path.join(os.path.expanduser('~'), 'AMR', 'AMR-main', 'src', 'agv_description', 'maps')
    ws_worlds_dir = os.path.join(os.path.expanduser('~'), 'AMR', 'AMR-main', 'src', 'agv_description', 'worlds')

    # Read user arguments
    map_input = LaunchConfiguration('map').perform(context).strip()
    world_input = LaunchConfiguration('world').perform(context).strip()
    headless_val = LaunchConfiguration('headless').perform(context).strip()
    use_rviz_val = LaunchConfiguration('use_rviz').perform(context).strip().lower() in ('true', '1')

    # ── 1. Resolve Map YAML File ──────────────────────────────────────
    if os.path.isabs(map_input) and os.path.exists(map_input):
        map_yaml_file = map_input
    else:
        clean_map = map_input[:-5] if map_input.endswith('.yaml') else map_input
        candidates = [
            os.path.join(pkg_agv, 'maps', f'{clean_map}.yaml'),
            os.path.join(ws_maps_dir, f'{clean_map}.yaml'),
            os.path.join(pkg_agv, 'maps', 'warehouse_01.yaml'),
            os.path.join(ws_maps_dir, 'warehouse_01.yaml'),
            os.path.join(pkg_agv, 'maps', 'warehouse_map.yaml'),
            os.path.join(ws_maps_dir, 'warehouse_map.yaml'),
        ]
        map_yaml_file = None
        for c in candidates:
            if os.path.exists(c):
                map_yaml_file = c
                break

        if map_yaml_file is None:
            map_yaml_file = candidates[0]

    map_stem = os.path.splitext(os.path.basename(map_yaml_file))[0]

    # ── 2. Resolve Gazebo World ───────────────────────────────────────
    if world_input and world_input != 'auto':
        w_clean = world_input if (world_input.endswith('.world') or world_input.endswith('.sdf')) else f'{world_input}.world'
        if os.path.isabs(w_clean):
            world_file = w_clean
        else:
            w_candidates = [
                os.path.join(pkg_agv, 'worlds', w_clean),
                os.path.join(ws_worlds_dir, w_clean),
            ]
            world_file = w_candidates[0]
            for wc in w_candidates:
                if os.path.exists(wc):
                    world_file = wc
                    break
    else:
        # Automatic Pairing:
        # warehouse_01 -> test1.world
        # warehouse_map -> warehouse.world
        if map_stem == 'warehouse_01':
            target_world_name = 'test1.world'
        elif map_stem == 'warehouse_map':
            target_world_name = 'warehouse.world'
        else:
            # Check if matching world exists
            matching = os.path.join(pkg_agv, 'worlds', f'{map_stem}.world')
            target_world_name = f'{map_stem}.world' if os.path.exists(matching) else 'warehouse.world'

        w_candidates = [
            os.path.join(pkg_agv, 'worlds', target_world_name),
            os.path.join(ws_worlds_dir, target_world_name),
        ]
        world_file = w_candidates[0]
        for wc in w_candidates:
            if os.path.exists(wc):
                world_file = wc
                break

    # ── 3. Synchronize Active Graph Roadmap for route_runner/bt_manager ─
    src_graph = os.path.join(ws_maps_dir, f'{map_stem}_graph.json')
    if not os.path.exists(src_graph):
        src_graph = os.path.join(pkg_agv, 'maps', f'{map_stem}_graph.json')

    if os.path.exists(src_graph):
        for target_dir in [ws_maps_dir, os.path.join(pkg_agv, 'maps')]:
            dest_graph = os.path.join(target_dir, 'warehouse_graph.json')
            try:
                if os.path.exists(target_dir):
                    shutil.copy2(src_graph, dest_graph)
            except Exception as e:
                print(f"[navigation_launch] Notice: Could not sync graph to {dest_graph}: {e}")

    # ── Log Configuration ─────────────────────────────────────────────
    print("=" * 65)
    print("  [AMR Navigation Launch]")
    print(f"  Active Map:    {map_stem} ({map_yaml_file})")
    print(f"  Gazebo World:  {os.path.basename(world_file)} ({world_file})")
    print(f"  RViz2 Enabled: {use_rviz_val}")
    print("=" * 65)

    # ── Config files ──────────────────────────────────────────────────
    nav2_params_file = os.path.join(pkg_agv, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_agv, 'config', 'agv_nav.rviz')

    # ── 4. Gazebo + Bridge + RSP + EKF ────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'headless': headless_val,
        }.items(),
    )

    # ── 5. Localization only: map_server + AMCL + lifecycle_manager ────
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
                        'yaml_filename': map_yaml_file,
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
                    'bond_timeout': 0.0,
                }],
                output='screen',
            ),
        ]
    )

    # ── 6. RViz2 with Nav2 panels ─────────────────────────────────────
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

    return [
        gazebo_launch,
        localization_launch,
        rviz_node,
    ]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='warehouse_01',
        description='Map name (e.g. "warehouse_01", "warehouse_map") or full path to map YAML file'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='auto',
        description='Gazebo world file name (e.g. "test1.world", "warehouse.world") or "auto" to match map'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo simulation headlessly'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )

    return LaunchDescription([
        map_arg,
        world_arg,
        headless_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
