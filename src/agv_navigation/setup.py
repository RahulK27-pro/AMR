from setuptools import find_packages, setup

package_name = 'agv_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
