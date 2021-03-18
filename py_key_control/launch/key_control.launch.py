import launch
import launch.actions
import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='py_key_control',
            executable='key_controller',
            name='key_controller',
        )
    ])