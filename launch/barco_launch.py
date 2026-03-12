from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='barco_autonomo_tmr',
            executable='gps_node',
            name='gps_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='imu_node',
            name='imu_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='navigation_node',
            name='navigation_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='motor_node',
            name='motor_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='servo_node',
            name='servo_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='conveyor_node',
            name='conveyor_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='ultra_sonic_node',
            name='ultra_sonic_node'
        ),
        Node(
            package='barco_autonomo_tmr',
            executable='esp_bridge_node',
            name='esp_bridge_node'
        )
    ])
