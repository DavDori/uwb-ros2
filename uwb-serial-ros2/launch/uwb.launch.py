import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar_turret'),
        'config',
        'lidar_turret.yaml',
        )
    
    return LaunchDescription([
        Node(
            package='lidar_turret',
            executable='turret',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_a3_launch.py',
        ),
    ])