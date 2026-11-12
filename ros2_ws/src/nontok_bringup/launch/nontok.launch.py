"""The whole robot.

    ros2 launch nontok_bringup nontok.launch.py
    ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=bolt     no hardware at all

With fake:=true the Mega is simulated, cameras and the wake word are off, and the tray is
driven by hand:
    ros2 run nontok_voice trigger                                   # "Nontok!"
    ros2 topic pub -1 /tray/sim_occupied std_msgs/msg/Bool "{data: true}"    # item placed
    ros2 topic pub -1 /tray/sim_occupied std_msgs/msg/Bool "{data: false}"   # item gone (after the dump)
In fake mode the tag detections are simulated from the simulated Mega's odometry, so
localisation, Nav2, tag alignment and the mission all run.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def share(package, *parts):
    return os.path.join(get_package_share_directory(package), *parts)


def generate_launch_description():
    fake = LaunchConfiguration('fake')
    real = PythonExpression(["'false' if '", fake, "' == 'true' else 'true'"])
    is_fake = PythonExpression(["'", fake, "' == 'true'"])
    here = share('nontok_bringup', 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false'),
        DeclareLaunchArgument('fake_item', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'base.launch.py')),
                                 launch_arguments={'fake': fake}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'localization.launch.py')),
                                 launch_arguments={'cameras': real, 'sim_tags': fake}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            share('nontok_navigation', 'launch', 'navigation.launch.py'))),
        Node(package='nontok_docking', executable='dock_server',
             parameters=[share('nontok_docking', 'config', 'docking.yaml')], output='screen'),
        Node(package='nontok_perception', executable='tray_node',
             parameters=[share('nontok_perception', 'config', 'tray.yaml'),
                         {'simulate': is_fake, 'fake_item': LaunchConfiguration('fake_item')}], output='screen'),
        Node(package='nontok_voice', executable='wake_node',
             parameters=[share('nontok_voice', 'config', 'voice.yaml')],
             condition=UnlessCondition(fake), output='screen'),
        Node(package='nontok_mission', executable='mission_node',
             parameters=[share('nontok_mission', 'config', 'mission.yaml')], output='screen'),
        Node(package='foxglove_bridge', executable='foxglove_bridge', parameters=[{'port': 8765}]),
    ])
