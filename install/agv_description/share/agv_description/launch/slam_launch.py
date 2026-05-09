import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path    = get_package_share_directory('agv_description')
    slam_params = os.path.join(pkg_path, 'config', 'mapper_params.yaml')

    slam_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[SLAM] Starting slam_toolbox...'),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params,
                    {'use_sim_time': True}
                ],
                output='screen',
                emulate_tty=True,
                # additional_env MERGES with existing environment.
                # env={} would REPLACE it, stripping LD_LIBRARY_PATH
                # and causing "cannot open shared object file" crashes.
                additional_env={
                    'RCUTILS_LOGGING_BUFFERED_STREAM': '0',
                    'RCUTILS_COLORIZED_OUTPUT': '1',
                },
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[SLAM] Waiting 5s for TF buffer to warm up...'),
        slam_node,
    ])