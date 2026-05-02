import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Dynamically find your home directory to build absolute paths
    home_dir = os.environ['HOME']
    pkg_path = os.path.join(home_dir, 'agv_ws', 'src', 'agv_description')
    
    # 2. Define exactly where our critical files live
    urdf_file = os.path.join(pkg_path, 'urdf', 'agv.urdf')
    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    rviz_file = os.path.join(pkg_path, 'config', 'agv.rviz')

    # 3. Create the RViz arguments list (Fail-safe: only load config if you saved it in Step 1)
    rviz_args = ['-d', rviz_file] if os.path.exists(rviz_file) else []

    # ==========================================
    # NODE DEFINITIONS
    # ==========================================

    # A. The Blueprint
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen'
    )

    # B. The Physics Engine
    world_file = os.path.join(pkg_path, 'worlds', 'warehouse.sdf')
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # C. The Spawner (Injects URDF into Gazebo)
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'warehouse_agv', '-z', '0.09', '-x', '-2.0'],
        output='screen'
    )

    # D. The Wideband Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        output='screen'
    )

    # E. The Brain (Your Custom Navigation Code)
    tracer_node = Node(
        package='agv_navigation',
        executable='tracer',
        output='screen'
    )

    # F. The Matrix Monitor
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    # ==========================================
    # EXECUTION SEQUENCE
    # ==========================================
    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_node,
        bridge_node,
        tracer_node,
        rviz_node
    ])