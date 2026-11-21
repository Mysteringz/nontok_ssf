"""Demo of the parts that need no wheels, on the real hardware: one tag camera, the tray
camera, the wake word and the dashboard. The Mega bridge, navigation and the mission are NOT
started, so nothing can arm or move the motors.

    ros2 launch nontok_bringup camera_demo.launch.py [tag_camera:=/dev/cam_front_right] [tray_camera:=/dev/cam_tray]

Watch it with tools/mac_viz.sh. Show a printed tag (docs/tags_160mm.pdf) to the tag camera to
see it outlined; say "Nontok" to see "Wake word heard"; ask what is in front of the tray camera:
    ros2 service call /tray/classify nontok_interfaces/srv/ClassifyItem
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    def share(package, *parts):
        return os.path.join(get_package_share_directory(package), *parts)

    def call(service, kind, request=''):
        return ExecuteProcess(cmd=['ros2', 'service', 'call', service, kind] + ([request] if request else []),
                              output='log')

    return LaunchDescription([
        DeclareLaunchArgument('tag_camera', default_value='/dev/cam_front_left'),
        DeclareLaunchArgument('tray_camera', default_value='/dev/cam_tray'),
        Node(package='nontok_localization', executable='tag_detector', name='tag_detector_front_left',
             parameters=[share('nontok_localization', 'config', 'detector.yaml'),
                         {'camera_name': 'front_left', 'device': LaunchConfiguration('tag_camera'), 'rate_hz': 6.0}],
             output='screen', respawn=True, respawn_delay=3.0),
        Node(package='nontok_perception', executable='tray_node',
             parameters=[share('nontok_perception', 'config', 'tray.yaml'),
                         {'device': LaunchConfiguration('tray_camera'), 'pause_detector': ''}], output='screen'),
        Node(package='nontok_voice', executable='wake_node',
             parameters=[share('nontok_voice', 'config', 'voice.yaml'), {'start_enabled': True}], output='screen'),
        Node(package='nontok_voice', executable='speech_node',
             parameters=[share('nontok_voice', 'config', 'speech.yaml')], output='screen'),
        Node(package='nontok_viz', executable='viz_node', output='screen'),
        # The mission normally switches the tray camera on; here it is on from the start.
        TimerAction(period=30.0, actions=[call('/tray/enable', 'std_srvs/srv/SetBool', '{data: true}')]),
        TimerAction(period=40.0, actions=[call('/tray/capture_reference', 'std_srvs/srv/Trigger')]),
    ])
