from setuptools import setup
import os
from glob import glob

package_name = 'museum_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'arduino_bridge = museum_robot.arduino_bridge:main',
            'scan_filter = museum_robot.scan_filter:main',
            'lidar_watchdog = museum_robot.lidar_watchdog:main',
            'apriltag_handler = museum_robot.apriltag_handler:main',
        ],
    },
)
