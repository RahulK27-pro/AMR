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
    ekf_file    = os.path.join(pkg_path, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # A. Gazebo — starts first, owns the clock
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # B. Bridge — starts immediately after Gazebo.
    #    Must be running BEFORE robot_state_publisher so
    #    the /clock topic exists when RSP subscribes to it.
    #    Without clock, RSP stamps all TF at time 0 and
    #    SLAM cannot match scans to transforms.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        output='screen'
    )

    # C. Blueprint — parameters= style is mandatory.
    #    arguments= style does NOT publish /robot_description
    #    topic, which means spawner and SLAM cannot find the model.
    #    Delayed 1s to ensure bridge is publishing /clock first.
    rsp_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True,
                }],
                output='screen'
            )
        ]
    )

    # D. Spawner — delayed 3s so Gazebo + bridge + RSP are all ready
    spawn_node = TimerAction(
        period=3.0,
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

    # E. EKF — fuses /odom + /imu/data → /odometry/filtered
    #    Delayed 3s so bridge is publishing both inputs first
    ekf_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[
                    ekf_file,
                    {
                        'use_sim_time': True,
                        'frequency': 30.0,
                    }
                ],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ],
                output='screen'
            )
        ]
    )

    # F. Navigation
    tracer_node = Node(
        package='agv_navigation',
        executable='tracer',
        output='screen'
    )

    # static_tf map→odom is intentionally ABSENT.
    # slam_launch.py owns that transform exclusively.

    return LaunchDescription([
        gazebo,         # 1. world + clock
        bridge_node,    # 2. bridge clock + all topics into ROS 2
        rsp_node,       # 3. robot description (after clock exists)
        spawn_node,     # 4. inject robot into world
        ekf_node,       # 5. fused odometry
        tracer_node,    # 6. your navigation code
    ])