"""Spawn Sieglinde in Gazebo Harmonic with ros2_control and ROS bridges."""
import os
import shlex
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    share = get_package_share_directory('sieglinde_sim')
    description = xacro.process_file(os.path.join(share, 'urdf/tracker/tracker.urdf.xacro')).toxml()
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    args = '-r -v 3 ' + ('' if gui else '-s --headless-rendering ') + shlex.quote(world)
    spawn = Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
        '-name', 'sieglinde', '-topic', 'robot_description', '-z', '0.12', '-allow_renaming', 'false'])
    controllers = Node(package='controller_manager', executable='spawner', output='screen', arguments=[
        'joint_state_broadcaster', 'arm_controller', 'gripper_controller', 'flipper_controller',
        '--controller-manager-timeout', '120'])
    def after_spawn(event, context):
        if event.returncode != 0:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='Robot spawn failed'))]
        return [controllers]
    resources = [os.path.dirname(share), os.path.dirname(get_package_share_directory('velodyne_description')),
                 os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resources)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config/bridge.yaml'), 'use_sim_time': True}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)), spawn,
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config/sieglinde.rviz')], parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(get_package_share_directory('sieglinde_sim'), 'worlds/sieglinde.sdf')),
        OpaqueFunction(function=setup),
    ])
