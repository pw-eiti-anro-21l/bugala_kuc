import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'robot_lab2.urdf.xml'
  rviz_file_name = 'robot_lab2.rviz'

  urdf = os.path.join(
      get_package_share_directory('urdf_lab2b'),
      urdf_file_name)
  rviz = os.path.join(
      get_package_share_directory('urdf_lab2b'),
      rviz_file_name)

  with open(urdf, 'r') as infp:
    robot_desc = infp.read()

  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro',' ',urdf])
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz]),
    ])
