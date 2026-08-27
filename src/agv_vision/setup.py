from setuptools import find_packages, setup

package_name = 'agv_vision'

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
    description='Camera-based obstacle detection for warehouse AGV using HSV color thresholding',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = agv_vision.obstacle_detector:main'
        ],
    },
)
