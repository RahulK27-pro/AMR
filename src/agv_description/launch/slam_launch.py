import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    home_dir    = os.environ['HOME']
    pkg_path    = os.path.join(home_dir, 'agv_ws', 'src', 'agv_description')
    slam_params = os.path.join(pkg_path, 'config', 'mapper_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='SLAM Toolbox starting. Drive the robot to build the map.'),
        slam_node,
    ])