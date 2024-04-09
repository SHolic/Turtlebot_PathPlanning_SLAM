import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a4g16',
            executable='my_navigation_node',
            name='my_navigation_node',
            output='screen'
        ),
    ])
