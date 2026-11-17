"""Inspect the original kinematic model without running physics."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('sieglinde_sim')
    description = xacro.process_file(os.path.join(share, 'urdf/tracker/tracker.urdf.xacro'), mappings={'simulation': 'false'}).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config/sieglinde.rviz')]),
    ])
