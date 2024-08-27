from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ocean_current',
            executable='ocean_current_publisher',
            name='ocean_current_publisher',
            output='screen',
        )
    ])