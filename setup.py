from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'delivery_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ROS 2 to find the package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # URDF / Xacro models
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*')),

        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*')),

        # Navigation / SLAM config
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),

        # Saved maps (pgm + yaml)
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),

        # RViz configurations
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Delivery Robot Team',
    maintainer_email='team@delivery-robot.dev',
    description='Autonomous Indoor Delivery Robot — ROS 2 Humble Simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Primary delivery mission node
            'delivery_mission = delivery_robot.delivery_mission:main',
            # Waypoint recorder utility
            'record_waypoints = delivery_robot.record_waypoints:main',
            # System health checker
            'health_check = delivery_robot.health_check:main',
            # Automated SLAM room explorer
            'slam_explorer = delivery_robot.slam_explorer:main',
        ],
    },
)
