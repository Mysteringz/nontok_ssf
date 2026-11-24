"""Map server and Nav2. Localisation (map -> odom) comes from nontok_localization, not AMCL."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from nontok_description.urdf_builder import footprint_string, load_config


def generate_launch_description():
    share = get_package_share_directory('nontok_navigation')
    robot = load_config(os.path.join(get_package_share_directory('nontok_description'), 'config', 'robot.yaml'))
    params = RewrittenYaml(
        source_file=os.path.join(share, 'config', 'nav2_params.yaml'),
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(share, 'config', 'nav_to_pose_no_spin.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(share, 'config', 'nav_through_poses_no_spin.xml'),
            'yaml_filename': LaunchConfiguration('map'),
            'footprint': footprint_string(robot),
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'config', 'map.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': params, 'use_sim_time': 'false', 'autostart': 'true'}.items(),
        ),
    ])
