import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    home_dir    = os.environ['HOME']
    pkg_path    = os.path.join(home_dir, 'agv_ws', 'src', 'agv_description')
    urdf_file   = os.path.join(pkg_path, 'urdf',   'agv.urdf')
    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    world_file  = os.path.join(pkg_path, 'worlds', 'warehouse.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── 1. GAZEBO ─────────────────────────────────────────────
    # -r = run immediately so clock ticks from t=0, not paused
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_file],
        output='screen',
        name='gz'
    )

    # ── 2. BRIDGE — starts at t=0 with Gazebo ─────────────────
    # No delay. /clock must reach ROS 2 as early as possible.
    # RSP starts 3s later, so /clock has 3 full seconds to
    # establish before RSP subscribes to it.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        output='screen'
    )

    start_bridge = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                LogInfo(msg='[MASTER 1/4] Gazebo started — bridge launching immediately...'),
                bridge_node,
            ]
        )
    )

    # ── 3. RSP — t=3s after Gazebo ────────────────────────────
    # /clock has been flowing for 3s by now.
    # RSP will subscribe to /clock and use real sim-timestamps.
    # TF from RSP will have timestamps like "At time 8.xxx"
    # not "At time 0.0" which was the SLAM killer.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen'
    )

    start_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg='[MASTER 2/4] robot_state_publisher starting...'),
                        rsp_node,
                    ]
                )
            ]
        )
    )

    # ── 4. SPAWNER — t=6s after Gazebo ────────────────────────
    # RSP must be running and publishing /robot_description.
    # After spawn: DiffDrive activates, /tf and /odom start flowing.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='robot_spawner',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'warehouse_agv',
            '-x',     '-2.0',
            '-y',     '0.0',
            '-z',     '0.09',
            '-Y',     '0.0',
        ],
        output='screen'
    )

    start_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=6.0,
                    actions=[
                        LogInfo(msg='[MASTER 3/4] Spawning robot...'),
                        spawn_node,
                    ]
                )
            ]
        )
    )

    # ── 5. NAVIGATION — t=9s after Gazebo ─────────────────────
    tracer_node = Node(
        package='agv_navigation',
        executable='tracer',
        output='screen'
    )

    start_tracer = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=9.0,
                    actions=[
                        LogInfo(msg='[MASTER 4/4] Navigation node starting...'),
                        tracer_node,
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        start_bridge,   # t=0 — /clock into ROS 2 immediately
        start_rsp,      # t=3 — RSP gets real timestamps
        start_spawn,    # t=6 — robot enters world
        start_tracer,   # t=9 — nav node
    ])