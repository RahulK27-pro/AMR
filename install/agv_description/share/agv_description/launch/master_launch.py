import os
from ament_index_python.packages import get_package_share_directory
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

    pkg_path    = get_package_share_directory('agv_description')
    urdf_file   = os.path.join(pkg_path, 'urdf',   'agv.urdf')
    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    world_file  = os.path.join(pkg_path, 'worlds', 'warehouse.sdf')
    ekf_file    = os.path.join(pkg_path, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_file],
        output='screen',
        name='gz'
    )

    # 2. Bridge — 6s after Gazebo. Longer delay ensures Gazebo fully
    #    loads the world AND the gz transport layer is ready to accept
    #    connections. Connecting too early = bridge silently fails =
    #    /clock never flows = all use_sim_time nodes freeze forever.
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
                LogInfo(msg='[1/5] Gazebo started — waiting 6s for world + gz transport to be ready...'),
                TimerAction(
                    period=6.0,
                    actions=[
                        LogInfo(msg='[2/5] Bridge starting — /clock should appear in ~2s...'),
                        bridge_node,
                    ]
                )
            ]
        )
    )

    # 3. RSP — 10s after Gazebo (4s after bridge).
    #    /clock must be flowing for RSP to use real sim timestamps.
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
                    period=10.0,
                    actions=[
                        LogInfo(msg='[3/5] robot_state_publisher starting...'),
                        rsp_node,
                    ]
                )
            ]
        )
    )

    # 4. Spawner — 13s after Gazebo.
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
                    period=13.0,
                    actions=[
                        LogInfo(msg='[4/5] Spawning robot...'),
                        spawn_node,
                    ]
                )
            ]
        )
    )

    # 5. EKF — 17s after Gazebo.
    #    Robot must be spawned first so /odom and /imu/data exist.
    #    Tracer node removed — it was crashing and killing the whole launch.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            ekf_file,
            {'use_sim_time': True, 'frequency': 30.0}
        ],
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    start_ekf_nav = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=17.0,
                    actions=[
                        LogInfo(msg='[5/5] EKF starting...'),
                        ekf_node,
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        start_bridge,    # t=6s
        start_rsp,       # t=10s
        start_spawn,     # t=13s
        start_ekf_nav,   # t=17s — EKF only, tracer removed
    ])