# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
    # return LaunchDescription([
        # Node(
            # package='a4g16',
            # executable='world_create',
            # name='world_create',
            # output='screen'
        # ),
        # # Add Gazebo launch configurations here if needed
    # ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Set Gazebo world file path
    world_file_path = os.path.join(
        os.getenv('GAZEBO_MODEL_PATH'), 'your_package_name', 'worlds', 'your_world.world')

    return LaunchDescription([
        # Launch Gazebo with the specified world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(os.getenv('GAZEBO_ROS_PACKAGE_PATH'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': world_file_path}.items(),
        ),

        # Launch your node
        Node(
            package='a4g16',
            executable='world_create',
            name='world_create',
            output='screen'
        ),
    ])






