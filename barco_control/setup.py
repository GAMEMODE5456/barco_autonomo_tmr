from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'barco_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cienticeromalote',
    maintainer_email='lucasrosasdiego@gmail.com',
    description='Paquete ROS2 para control de barco autónomo TMR México',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'motor_node       = barco_control.motor_node:main',
            'servo_node       = barco_control.servo_node:main',
            'imu_node         = barco_control.imu_node:main',
            'gps_node         = barco_control.gps_node:main',
            'navigation_node  = barco_control.navigation_node:main',
            'conveyor_node    = barco_control.conveyor_node:main',
            'ultra_sonic_node = barco_control.ultra_sonic_node:main',
            'esp_bridge_node  = barco_control.esp_bridge_node:main',
            'camera_node      = barco_control.camera_node:main',
        ],
    },
)
