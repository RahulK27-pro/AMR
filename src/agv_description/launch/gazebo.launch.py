from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    pkg_path = get_package_share_directory('agv_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'warehouse_agv.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'warehouse.world')
    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    ekf_file = os.path.join(pkg_path, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    gz_cmd = ['gz', 'sim']
    if headless.lower() in ('true', '1'):
        gz_cmd.append('-s')
    gz_cmd.extend(['-r', '-v', '2', world_file])

    # WSL2 requires software rendering via llvmpipe since there is no /dev/dri
    # OGRE2 in hardware mode causes [WARN:COPY MODE] and window crashes under WSL2
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
            'MESA_GL_VERSION_OVERRIDE': '4.5',
        },
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': True}]
    )

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

    return [
        gazebo,
        bridge_node,
        rsp,
        ekf_node,
        spawn
    ]

def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo simulation headlessly (without GUI)'
    )

    return LaunchDescription([
        headless_arg,
        OpaqueFunction(function=launch_setup)
    ])


