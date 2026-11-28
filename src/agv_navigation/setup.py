from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'agv_navigation'

model_files = [f for f in glob('models/*') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usernamerahul',
    maintainer_email='usernamerahul@todo.todo',
    description='Topological navigation with Dijkstra planner and custom MPPI controller for warehouse AGV',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'route_runner = agv_navigation.route_runner:main',
            'graph_visualizer = agv_navigation.graph_visualizer:main',
            'dynamic_obstacle_manager = agv_navigation.dynamic_obstacle_manager:main',
            'obstacle_teleop = agv_navigation.obstacle_teleop:main',
            'bt_manager = agv_navigation.bt_manager:main',
            'auto_localizer = agv_navigation.amr_auto_localizer:main',
            'run_logger = agv_navigation.run_logger:main',
            'dataset_recorder = agv_navigation.dataset_recorder:main',
            'sim_mission_agent = agv_navigation.ml_agents.sim_mission_agent:main',
            'obstacle_director_agent = agv_navigation.ml_agents.obstacle_director_agent:main',
            'dataset_trainer_agent = agv_navigation.ml_agents.dataset_trainer_agent:main',
        ],
    },
)
