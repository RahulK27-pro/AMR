import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    home_dir    = os.environ['HOME']
    pkg_path    = os.path.join(home_dir, 'agv_ws', 'src', 'agv_description')
    urdf_file   = os.path.join(pkg_path, 'urdf',   'agv.urdf')
    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    world_file  = os.path.join(pkg_path, 'worlds', 'warehouse.sdf')
    ekf_file    = os.path.join(pkg_path, 'config', 'ekf.yaml')   # ← new

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # A. The Blueprint
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': True}]  # <-- THE CRITICAL FIX
    )

    # B. Physics engine
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # C. Spawner
    spawn_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
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
        ]
    )

    # D. The Wideband Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        parameters=[{'use_sim_time': True}]  # <-- ENSURES BRIDGE RESPECTS SIM CLOCK
    )

    # E. EKF — fuses /odom + /imu/data → /odometry/filtered
    #    This is now the authoritative pose estimate for the robot.
    #    Nav2 and SLAM will use /odometry/filtered, not raw /odom.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            ekf_file,
            {'use_sim_time': True, 'frequency': 30.0}
        ],
        remappings=[
            # EKF publishes here — this becomes your clean odometry
            ('odometry/filtered', '/odometry/filtered'),
        ],
        output='screen'
    )

    # F. Static TF: map → odom (remove once SLAM is added)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # G. Navigation
    tracer_node = Node(
        package='agv_navigation',
        executable='tracer',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        bridge_node,
        static_tf,
        ekf_node,        # ← added
        spawn_node,
        tracer_node,
    ])