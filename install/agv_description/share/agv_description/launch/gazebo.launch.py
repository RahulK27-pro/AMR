from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('agv_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'warehouse_agv.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'warehouse.world')
    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    ekf_file = os.path.join(pkg_path, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_file],
        output='screen'
    )

    # 2. Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        output='screen'
    )

    # 3. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # 4. EKF Node (Robot Localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': True}]
    )

    # 5. Spawn Entity
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'warehouse_agv',
            '-topic', 'robot_description',
            '-z', '0.05'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge_node,
        rsp,
        ekf_node,
        spawn
    ])
