"""Base only, for driving by keyboard. In a second terminal:

    ros2 service call /base/arm std_srvs/srv/SetBool "{data: true}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.join(get_package_share_directory('nontok_bringup'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'base.launch.py')),
                                 launch_arguments={'fake': LaunchConfiguration('fake')}.items()),
    ])
