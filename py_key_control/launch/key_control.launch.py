import launch
import launch.actions
import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node'
        ),
        Node(
            package='py_key_control',
            executable='key_controller',
            name='key_control',
            prefix=["gnome-terminal ", "-- "],
            output='screen',
            parameters=[
            	{'move_forward_key': 'i'},
            	{'move_back_key': 'k'},
            	{'turn_left_key': 'j'},
            	{'turn_right_key': 'l'}
            ]
        )
    ])