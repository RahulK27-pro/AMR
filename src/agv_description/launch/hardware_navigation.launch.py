#!/usr/bin/env python3
"""
hardware_navigation.launch.py
=============================
Master Navigation Launch for AMR Physical Platform:
  1. nav2_map_server: Serves static occupancy grid YAML
  2. nav2_amcl: Adaptive Monte Carlo Localization on physical LiDAR (/scan)
  3. nav2_lifecycle_manager: Auto-transitions map_server & AMCL to ACTIVE
  4. route_runner: Dijkstra Roadmap Planning + 10 Hz MPPI Local Trajectory Control
  5. graph_visualizer: Publishes /agv_graph_markers (golden nodes & edges) for RViz
  6. bt_manager: Optional PyTrees Behavior Tree mission orchestrator
  7. rviz2: Visualizes Map, Particle Cloud, Dense Path, LiDAR Scan & 2D Nav Goal

All nodes execute strictly with use_sim_time:=false.
Completely decoupled from simulation files to prevent corruption.
"""

import os
import sys
import shutil
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_agv = get_package_share_directory('agv_description')
    
    # Locate workspace directories
    ws_dir = os.environ.get('AMR_WS', '')
    if not ws_dir or not os.path.exists(ws_dir):
        candidates_ws = [
            os.path.join(os.path.expanduser('~'), 'AMR', 'AMR-main'),
            os.path.join(os.path.expanduser('~'), 'AMR'),
        ]
        for c in candidates_ws:
            if os.path.exists(os.path.join(c, 'src', 'agv_description')):
                ws_dir = c
                break
        if not ws_dir:
            ws_dir = candidates_ws[0]

    ws_root_maps = os.path.join(ws_dir, 'maps')
    ws_pkg_maps = os.path.join(ws_dir, 'src', 'agv_description', 'maps')

    # Read user launch configurations
    map_input = LaunchConfiguration('map').perform(context).strip()
    use_rviz_val = LaunchConfiguration('use_rviz').perform(context).strip().lower() in ('true', '1')
    use_bt_val = LaunchConfiguration('use_bt').perform(context).strip().lower() in ('true', '1')
    init_x = float(LaunchConfiguration('initial_pose_x').perform(context).strip() or 0.0)
    init_y = float(LaunchConfiguration('initial_pose_y').perform(context).strip() or 0.0)
    init_yaw = float(LaunchConfiguration('initial_pose_yaw').perform(context).strip() or 0.0)
    max_v = float(LaunchConfiguration('max_linear_speed').perform(context).strip() or 0.20)
    max_w = float(LaunchConfiguration('max_angular_speed').perform(context).strip() or 0.75)
    cruise_v = float(LaunchConfiguration('cruise_linear_speed').perform(context).strip() or 0.16)
    lookahead_d = float(LaunchConfiguration('lookahead_dist').perform(context).strip() or 0.35)

    # ── 1. Resolve Map YAML File ──────────────────────────────────────
    if os.path.isabs(map_input) and os.path.exists(map_input):
        map_yaml_file = map_input
    else:
        clean_map = map_input[:-5] if map_input.endswith('.yaml') else map_input
        candidates = [
            os.path.join(ws_root_maps, f'{clean_map}.yaml'),
            os.path.join(ws_pkg_maps, f'{clean_map}.yaml'),
            os.path.join(pkg_agv, 'maps', f'{clean_map}.yaml'),
            os.path.join(ws_root_maps, 'prakash_room.yaml'),
            os.path.join(ws_root_maps, 'final_map.yaml'),
            os.path.join(ws_root_maps, 'my_clean_warehouse11_map.yaml'),
        ]
        map_yaml_file = None
        for c in candidates:
            if os.path.exists(c):
                map_yaml_file = c
                break

        if map_yaml_file is None:
            raise FileNotFoundError(f"Could not locate map YAML matching '{map_input}'. Checked: {candidates}")

    map_stem = os.path.splitext(os.path.basename(map_yaml_file))[0]
    map_dir = os.path.dirname(map_yaml_file)

    # ── 2. Ensure Topological Graph Exists (Auto-extract if missing) ──
    graph_candidates = [
        os.path.join(map_dir, f'{map_stem}_graph.json'),
        os.path.join(ws_root_maps, f'{map_stem}_graph.json'),
        os.path.join(ws_pkg_maps, f'{map_stem}_graph.json'),
        os.path.join(pkg_agv, 'maps', f'{map_stem}_graph.json'),
    ]
    graph_file = None
    for gc in graph_candidates:
        if os.path.exists(gc):
            graph_file = gc
            break

    if graph_file is None:
        target_graph_out = os.path.join(ws_root_maps, f'{map_stem}_graph.json')
        print(f"[HW_NAV] Topological graph for '{map_stem}' not found. Auto-generating roadmap via graph_extractor.py...")
        extractor_script = os.path.join(ws_pkg_maps, 'graph_extractor.py')
        if not os.path.exists(extractor_script):
            extractor_script = os.path.join(pkg_agv, 'maps', 'graph_extractor.py')

        try:
            cmd = [
                sys.executable, extractor_script,
                map_yaml_file,
                '--output', target_graph_out
            ]
            subprocess.run(cmd, check=True)
            graph_file = target_graph_out
            print(f"[HW_NAV] ✅ Roadmap generated successfully: {graph_file}")
        except Exception as e:
            print(f"[HW_NAV] ⚠️ Graph auto-extraction error: {e}")

    # Synchronize graph to target map directories for route_runner & graph_visualizer
    if graph_file and os.path.exists(graph_file):
        for t_dir in [ws_pkg_maps, os.path.join(pkg_agv, 'maps'), ws_root_maps]:
            if os.path.exists(t_dir):
                target_named = os.path.join(t_dir, f'{map_stem}_graph.json')
                if not os.path.exists(target_named) or os.path.getmtime(graph_file) > os.path.getmtime(target_named):
                    try:
                        shutil.copy2(graph_file, target_named)
                    except Exception:
                        pass

    # ── Log Operational Parameters ────────────────────────────────────
    print("=" * 72)
    print("  [AMR PHYSICAL HARDWARE NAVIGATION SESSION]")
    print(f"  Active Map          : {map_stem} ({map_yaml_file})")
    print(f"  Roadmap Graph       : {graph_file}")
    print(f"  Max Linear Speed    : {max_v:.2f} m/s (Calibrated Moving Crawl)")
    print(f"  Max Angular Speed   : {max_w:.2f} rad/s (~{max_w * 57.2958:.0f} deg/s)")
    print(f"  Cruise Speed        : {cruise_v:.2f} m/s")
    print(f"  Lookahead Distance  : {lookahead_d:.2f} m")
    print("  Auto-Localizer      : ZERO-PRIOR CORRELATIVE SCAN MATCHER (ACTIVE)")
    print("                        Auto-detects robot position anywhere on startup!")
    print(f"  Local Controller    : 10 Hz Vectorized MPPI (route_runner)")
    print(f"  Behavior Tree       : Enabled={use_bt_val}")
    print(f"  RViz Display        : Enabled={use_rviz_val}")
    print("=" * 72)

    # Config files
    nav2_hw_params = os.path.join(pkg_agv, 'config', 'nav2_params_hardware.yaml')
    if not os.path.exists(nav2_hw_params):
        nav2_hw_params = os.path.join(ws_dir, 'src', 'agv_description', 'config', 'nav2_params_hardware.yaml')

    rviz_config_file = os.path.join(pkg_agv, 'config', 'agv_hardware_nav.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(ws_dir, 'src', 'agv_description', 'config', 'agv_hardware_nav.rviz')

    has_manual_init = (init_x != 0.0 or init_y != 0.0 or init_yaw != 0.0)
    amcl_params = [
        nav2_hw_params,
        {
            'use_sim_time': False,
            'set_initial_pose': has_manual_init,
            'always_reset_initial_pose': True,
        }
    ]
    if has_manual_init:
        amcl_params[1].update({
            'initial_pose_x': init_x,
            'initial_pose_y': init_y,
            'initial_pose_a': init_yaw,
        })

    # ── 3. Localization Nodes: Map Server + AMCL + Lifecycle Manager ──
    localization_nodes = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[
                    nav2_hw_params,
                    {
                        'yaml_filename': map_yaml_file,
                        'use_sim_time': False,
                    },
                ],
                output='screen',
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=amcl_params,
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                    'bond_timeout': 0.0,
                }],
                output='screen',
            ),
        ]
    )

    # ── 3b. Zero-Prior Global Scan-Matching Auto-Localizer ────────────
    auto_localizer_node = TimerAction(
        period=1.8,
        actions=[
            Node(
                package='agv_navigation',
                executable='auto_localizer',
                name='auto_localizer',
                parameters=[{
                    'map_name': map_stem,
                    'use_sim_time': False,
                    'auto_on_startup': True,
                }],
                output='screen',
            )
        ]
    )

    # ── 4. Topological Route Runner (Dijkstra + 10 Hz MPPI Controller) ─
    route_runner_node = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='agv_navigation',
                executable='route_runner',
                name='route_runner',
                parameters=[{
                    'map_name': map_stem,
                    'use_sim_time': False,
                    'max_linear_speed': max_v,
                    'max_angular_speed': max_w,
                    'cruise_linear_speed': cruise_v,
                    'min_linear_speed': 0.10,
                    'lookahead_dist': lookahead_d,
                    'accel_lim_v': 0.35,
                    'decel_lim_v': 0.50,
                    'accel_lim_w': 1.20,
                    'kp_heading': 1.20,
                    'safe_standoff': 0.30,
                    'critical_proximity': 0.14,
                }],
                output='screen',
            )
        ]
    )

    # ── 5. Graph Visualizer (Publishes /agv_graph_markers for RViz) ────
    graph_vis_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='agv_navigation',
                executable='graph_visualizer',
                name='graph_visualizer',
                parameters=[{
                    'map_name': map_stem,
                    'use_sim_time': False,
                }],
                output='screen',
            )
        ]
    )

    # ── 6. Behavior Tree Mission Manager (Optional) ───────────────────
    bt_manager_node = Node(
        package='agv_navigation',
        executable='bt_manager',
        name='bt_manager',
        parameters=[{
            'map_name': map_stem,
            'use_sim_time': False,
        }],
        condition=IfCondition(LaunchConfiguration('use_bt')),
        output='screen',
    )

    # ── 7. RViz2 Hardware Navigation Interface ────────────────────────
    rviz_node = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': False}],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                output='screen',
            )
        ]
    )

    # ── 8. Dedicated Real-Time Navigation Telemetry Logger ────────────
    run_logger_node = Node(
        package='agv_navigation',
        executable='run_logger',
        name='run_logger',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return [
        localization_nodes,
        auto_localizer_node,
        route_runner_node,
        graph_vis_node,
        bt_manager_node,
        rviz_node,
        run_logger_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='prakash_room',
            description='Map name (e.g. "prakash_room", "final_map") or absolute path to map YAML file'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to open RViz2'
        ),
        DeclareLaunchArgument(
            'use_bt',
            default_value='false',
            description='Whether to launch Behavior Tree orchestrator (bt_manager)'
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial AMCL estimated X position in meters'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial AMCL estimated Y position in meters'
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial AMCL estimated Yaw heading in radians'
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.20',
            description='Max physical linear speed in m/s (calibrated moving crawl)'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.75',
            description='Max physical angular turning rate in rad/s'
        ),
        DeclareLaunchArgument(
            'cruise_linear_speed',
            default_value='0.16',
            description='Target physical linear cruising speed in m/s'
        ),
        DeclareLaunchArgument(
            'lookahead_dist',
            default_value='0.35',
            description='Lookahead target distance along path arc in meters'
        ),
        OpaqueFunction(function=launch_setup),
    ])
