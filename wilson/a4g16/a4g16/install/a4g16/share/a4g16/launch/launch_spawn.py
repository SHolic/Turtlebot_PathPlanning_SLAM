
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a4g16',
            executable='spawn_entity.py',
            arguments=['-topic', '/gazebo/spawn_sdf_model', '-entity', 'gridworld', '-file', 'package://a4g16/worlds/gridworld.world'],
            output='screen',
        ),
    ])
