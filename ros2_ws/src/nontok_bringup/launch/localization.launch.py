"""Tag detectors, tag localiser and the two EKFs.

    cameras:=false    no tag detectors (bench work without cameras); the localiser then
                      reports "healthy" on odometry alone.
    sim_tags:=true    with cameras:=false, simulate the detections from the simulated Mega's
                      odometry, so localisation and docking run without hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# name: detections per second. Kept low for the Jetson Nano (see detector.yaml); the localiser
# uses at most 5 fixes per second from a camera anyway.
# No rear camera is fitted (three cameras: two front, one tray). If one is added for a back_in
# station in stations.yaml, add 'rear': 6.0 here and to the cameras lists in
# localization.yaml and docking.yaml, and give one front camera 3.
CAMERAS = {'front_left': 6.0, 'front_right': 6.0}


def generate_launch_description():
    share = get_package_share_directory('nontok_localization')
    config = os.path.join(share, 'config')
    cameras = LaunchConfiguration('cameras')
    sim_tags = LaunchConfiguration('sim_tags')
    require_tags = PythonExpression(["'", cameras, "' == 'true' or '", sim_tags, "' == 'true'"])
    ekf = os.path.join(config, 'ekf.yaml')
    actions = [
        DeclareLaunchArgument('cameras', default_value='true'),
        DeclareLaunchArgument('sim_tags', default_value='false'),
        Node(package='nontok_localization', executable='fake_tags',
             parameters=[{'tags_file': os.path.join(config, 'tags.yaml'),
                          'cameras_dir': os.path.join(config, 'cameras')}],
             condition=IfCondition(sim_tags), output='screen'),
    ]
    for name, rate in CAMERAS.items():
        actions.append(Node(
            package='nontok_localization', executable='tag_detector', name='tag_detector_' + name,
            parameters=[os.path.join(config, 'detector.yaml'),
                        {'camera_name': name, 'device': '/dev/cam_' + name, 'rate_hz': rate}],
            condition=IfCondition(cameras), output='screen', respawn=True, respawn_delay=3.0))
    actions += [
        # A little below the control nodes: position fixes can wait a moment, steering cannot.
        Node(package='nontok_localization', executable='tag_localizer', prefix='nice -n 5',
             parameters=[os.path.join(config, 'localization.yaml'),
                         {'tags_file': os.path.join(config, 'tags.yaml'),
                          'cameras_dir': os.path.join(config, 'cameras'),
                          'require_tags': require_tags}],
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_local', parameters=[ekf],
             remappings=[('odometry/filtered', 'odometry/local'), ('set_pose', 'ekf_local/set_pose')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global', parameters=[ekf],
             remappings=[('odometry/filtered', 'odometry/global'), ('set_pose', 'ekf_global/set_pose')]),
    ]
    return LaunchDescription(actions)
