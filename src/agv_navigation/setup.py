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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tracer = agv_navigation.path_tracer:main',
            # 'map_builder = agv_navigation.map_builder:main',  # RETIRED — superseded by graph_extractor
            'route_runner = agv_navigation.route_runner:main',
            'graph_visualizer = agv_navigation.graph_visualizer:main',
            'data_logger = agv_navigation.data_logger:main'
        ],
    },
)
