import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('uwb_serial'),
        'config',
        'params.yaml',
        )
    print(config)
    return LaunchDescription([
        launch_ros.actions.Node(
            package='uwb_serial',
            executable='uwb',
            output='screen',
            parameters=[config],
        ),
    ])