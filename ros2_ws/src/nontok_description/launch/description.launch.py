"""Publishes the robot's fixed frames (cameras, ultrasonics, gyro)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from nontok_description.urdf_builder import build_urdf, load_config


def generate_launch_description():
    config = load_config(os.path.join(get_package_share_directory('nontok_description'), 'config', 'robot.yaml'))
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': build_urdf(config)}]),
    ])
