"""The base: serial bridge to the Mega and the robot's fixed frames.

    ros2 launch nontok_bringup base.launch.py                 real Mega on /dev/nontok_mega
    ros2 launch nontok_bringup base.launch.py fake:=true      simulated Mega, no hardware
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

FAKE_PORT = '/tmp/nontok_fake_mega'


def generate_launch_description():
    fake = LaunchConfiguration('fake')
    base_config = os.path.join(get_package_share_directory('nontok_base'), 'config', 'base.yaml')
    port = PythonExpression(["'", FAKE_PORT, "' if '", fake, "' == 'true' else '/dev/nontok_mega'"])
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false'),
        ExecuteProcess(cmd=['ros2', 'run', 'nontok_base', 'fake_mega', '--link', FAKE_PORT],
                       condition=IfCondition(fake), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nontok_description'), 'launch', 'description.launch.py'))),
        # Started a moment later so the fake Mega's port exists.
        TimerAction(period=1.0, actions=[
            Node(package='nontok_base', executable='base_bridge', parameters=[base_config, {'port': port}],
                 output='screen'),
        ]),
    ])
