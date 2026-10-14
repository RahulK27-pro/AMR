from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    world_val = LaunchConfiguration('world').perform(context).strip()
    pkg_path = get_package_share_directory('agv_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'warehouse_agv.urdf')

    # Resolve world file: supports "warehouse.world", "test1.world", "test1", or full path
    if not world_val.endswith('.world') and not world_val.endswith('.sdf'):
        world_val += '.world'
    if os.path.isabs(world_val):
        world_file = world_val
    else:
        cand = os.path.join(pkg_path, 'worlds', world_val)
        if not os.path.exists(cand):
            candidates_ws = [os.environ.get('AMR_WS')] if os.environ.get('AMR_WS') else []
            candidates_ws.extend([
                os.path.join(os.path.expanduser('~'), 'AMR', 'AMR-main'),
                os.path.join(os.path.expanduser('~'), 'AMR'),
            ])
            world_file = cand
            for ws in candidates_ws:
                src_cand = os.path.join(ws, 'src', 'agv_description', 'worlds', world_val)
                if os.path.exists(src_cand):
                    world_file = src_cand
                    break
        else:
            world_file = cand

    world_stem = os.path.splitext(os.path.basename(world_file))[0]

    bridge_file = os.path.join(pkg_path, 'config', 'bridge.yaml')
    ekf_file = os.path.join(pkg_path, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    gz_cmd = ['gz', 'sim']
    if headless.lower() in ('true', '1'):
        gz_cmd.extend(['-s', '--headless-rendering'])
    gz_cmd.extend(['-r', '-v', '2', world_file])

    # For headless rendering & environments without desktop OpenGL 3.3 GPU (Raspberry Pi, WSL2):
    # Ogre2 requires OpenGL 3.3. On Raspberry Pi (VideoCore GPU) or headless Linux,
    # force Mesa software rasterization with OpenGL 3.3 override so Ogre2 doesn't crash with SIGSEGV.
    gz_env = {
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'MESA_GL_VERSION_OVERRIDE': '3.3',
    }

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        additional_env=gz_env if gz_env else None,
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

    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'warehouse_agv',
                    '-topic', 'robot_description',
                    '-world', world_stem,
                    '-z', '0.05'
                ],
                output='screen'
            )
        ]
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
    world_arg = DeclareLaunchArgument(
        'world', default_value='warehouse.world',
        description='World file name (in worlds/) or full path to world file'
    )

    return LaunchDescription([
        headless_arg,
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])


