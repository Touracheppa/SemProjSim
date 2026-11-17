"""Topic-only control; use ros2 run control_node.py for terminal keyboard input."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='sieglinde_sim', executable='control_node.py', output='screen',
             parameters=[{'keyboard': False, 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
