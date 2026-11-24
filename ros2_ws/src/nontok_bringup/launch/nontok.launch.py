"""The whole robot.

    ros2 launch nontok_bringup nontok.launch.py
    ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=bolt     no hardware at all
    ... viewer:=true                                                           also start the Foxglove bridge (port 8765)
    ... fake:=true scene:=busy                                                 simulated tables and walking people
                                                                               (nontok_navigation/config/scene_busy.yaml)

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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# The Jetson Nano has no CPU to spare. Nodes that can wait (dashboard, voice, tray) run at low
# priority so that the motor link, the filters, navigation and tag alignment always go first.
LOW_PRIORITY = 'nice -n 10'


def share(package, *parts):
    return os.path.join(get_package_share_directory(package), *parts)


def room(context):
    """The Mega, navigation and the dashboard, which must agree on the room. With scene:=<name>
    (simulation only) the scene's tables are drawn onto a copy of the map, and the simulated
    Mega gets the scene for its ultrasonics; the walking people are on no map."""
    here = os.path.dirname(os.path.abspath(__file__))
    fake = LaunchConfiguration('fake')
    map_file = share('nontok_navigation', 'config', 'map.yaml')
    scene_file = ''
    name = LaunchConfiguration('scene').perform(context)
    if name:
        import yaml
        from nontok_navigation.draw_map import load_layout, rasterise, write_map
        scene_file = share('nontok_navigation', 'config', 'scene_%s.yaml' % name)
        with open(scene_file) as handle:
            scene = yaml.safe_load(handle) or {}
        bounds, rectangles, resolution = load_layout(share('nontok_navigation', 'config', 'lab_layout.yaml'))
        rectangles += [table['rect'] for table in scene.get('tables') or []]
        write_map('/tmp/nontok_scene_map', rasterise(bounds, rectangles, resolution), bounds, resolution)
        map_file = '/tmp/nontok_scene_map.yaml'
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'base.launch.py')),
                                 launch_arguments={'fake': fake, 'scene_file': scene_file}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            share('nontok_navigation', 'launch', 'navigation.launch.py')),
            launch_arguments={'map': map_file}.items()),
        Node(package='nontok_viz', executable='viz_node', condition=IfCondition(LaunchConfiguration('dashboard')),
             prefix=LOW_PRIORITY, parameters=[{'map_file': map_file, 'scene_file': scene_file}], output='screen'),
    ]


def generate_launch_description():
    fake = LaunchConfiguration('fake')
    real = PythonExpression(["'false' if '", fake, "' == 'true' else 'true'"])
    is_fake = PythonExpression(["'", fake, "' == 'true'"])
    here = share('nontok_bringup', 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false'),
        DeclareLaunchArgument('fake_item', default_value=''),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('speech', default_value='true'),
        DeclareLaunchArgument('scene', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'localization.launch.py')),
                                 launch_arguments={'cameras': real, 'sim_tags': fake}.items()),
        Node(package='nontok_docking', executable='dock_server',
             parameters=[share('nontok_docking', 'config', 'docking.yaml')], output='screen'),
        Node(package='nontok_perception', executable='tray_node', prefix=LOW_PRIORITY,
             parameters=[share('nontok_perception', 'config', 'tray.yaml'),
                         {'simulate': is_fake, 'fake_item': LaunchConfiguration('fake_item')}], output='screen'),
        Node(package='nontok_voice', executable='wake_node',
             parameters=[share('nontok_voice', 'config', 'voice.yaml')],
             condition=UnlessCondition(fake), output='screen'),
        # Live dashboard as a web page on port 8080 (tools/mac_viz.sh shows it on a laptop).
        OpaqueFunction(function=room),
        # The robot's voice. Without a speaker it only logs that it could not play.
        Node(package='nontok_voice', executable='speech_node', condition=IfCondition(LaunchConfiguration('speech')),
             prefix=LOW_PRIORITY,
             parameters=[share('nontok_voice', 'config', 'speech.yaml')], output='screen'),
        Node(package='nontok_mission', executable='mission_node',
             parameters=[share('nontok_mission', 'config', 'mission.yaml')], output='screen'),
        # Off unless asked for: on the Jetson Nano the viewer bridge takes CPU the robot needs.
        Node(package='foxglove_bridge', executable='foxglove_bridge', parameters=[{'port': 8765}],
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
