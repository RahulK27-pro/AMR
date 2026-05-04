import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    home_dir  = os.environ['HOME']
    pkg_path  = os.path.join(home_dir, 'agv_ws', 'src', 'agv_description')
    slam_params = os.path.join(pkg_path, 'config', 'mapper_params.yaml')

    # Delayed 5s to ensure master stack is fully running:
    #   - Gazebo clock flowing
    #   - Robot spawned
    #   - Bridge publishing /scan and /tf
    #   - EKF publishing /odometry/filtered
    #   - RSP publishing TF with real timestamps (not time 0)
    slam_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params,
                    {'use_sim_time': True}
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        slam_node,
    ])