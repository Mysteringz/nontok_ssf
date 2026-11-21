"""ROS 2 node: the live dashboard. Collects what every part of the robot is doing and serves
it as one web page (default port 8080). View it from a laptop with tools/mac_viz.sh.

    /            the page
    /state       all telemetry as JSON (the page asks for it four times a second)
    /layout      the map picture's scale, the tags and the stations (asked for once)
    /map.png     the map
    /cam/<name>  the newest picture from a camera: front_left, front_right, rear, tray

Camera pictures are only produced while someone is looking: this node publishes
viz/watching while the page is open, and the detectors and the tray node send a small
picture twice a second only then.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import Bool, Empty, String

from nontok_interfaces.msg import BaseStatus, LocalizationStatus, MissionState, TagDetectionArray, TrayState

from .telemetry import Telemetry, system_stats, thin_path, yaw_of

RANGES = ('left45', 'right45', 'front', 'left', 'right')
CAMERAS = ('front_left', 'front_right', 'rear')
TRAY_STATES = {0: 'EMPTY', 1: 'OCCUPIED', 2: 'MOVING'}
LOUD_NODES = ('mission_node', 'speech_node', 'wake_node', 'tray_node', 'dock_server', 'base_bridge')


class VizNode(Node):
    def __init__(self):
        super().__init__('viz_node')
        p = self.declare_parameter
        self.port = p('port', 8080).value
        nav = get_package_share_directory('nontok_navigation')
        loc = get_package_share_directory('nontok_localization')
        self.web = os.path.join(get_package_share_directory('nontok_viz'), 'web')
        self.telemetry = Telemetry()
        self.images = {}                       # camera name -> newest JPEG bytes
        self.last_request = 0.0
        self.layout, self.map_png = self.load_layout(
            p('map_file', os.path.join(nav, 'config', 'map.yaml')).value,
            p('tags_file', os.path.join(loc, 'config', 'tags.yaml')).value,
            p('stations_file', os.path.join(nav, 'config', 'stations.yaml')).value)
        self.previous_state = None
        self.scene_tables(p('scene_file', '').value)
        self.sim_file = p('sim_world_file', '/tmp/nontok_sim_world.json').value
        self.sim_contacts = 0
        self.boot = time.time()

        # Slow topics are always listened to, so the event list is complete when the page is
        # opened. The fast ones (odometry, ranges, tags, pictures) cost a Jetson Nano about a
        # quarter of a core to unpack in Python, so they are only listened to while someone
        # is watching: see start_feed.
        t = self.telemetry
        self.create_subscription(MissionState, 'mission/state', self.on_mission, 5)
        self.create_subscription(BaseStatus, 'base/status', lambda m: t.set('base', {
            'armed': m.armed, 'tray_deg': round(m.tray_deg, 1), 'battery_v': round(m.battery_v, 2),
            'charge_v': round(m.charge_v, 2), 'fault': m.fault}), 2)
        self.create_subscription(LocalizationStatus, 'localization/status', lambda m: t.set('localization', {
            'healthy': m.healthy, 'tags_in_view': m.tags_in_view,
            'seconds_since_fix': round(min(m.seconds_since_fix, 9999.0), 1),
            'metres_since_fix': round(min(m.metres_since_fix, 9999.0), 2)}), 2)
        self.create_subscription(Empty, 'wake', lambda m: (t.set('wake', True), t.event('Wake word heard')), 5)
        self.create_subscription(String, 'base/oled', lambda m: t.set('display', m.data), 5)
        self.create_subscription(Log, 'rosout', self.on_log, 50)
        self.feed = None           # (node, executor, thread) while someone is watching
        self.watching = self.create_publisher(Bool, 'viz/watching', 2)
        self.create_timer(1.0, self.on_second)
        self.server = ThreadingHTTPServer(('0.0.0.0', self.port), self.handler())
        threading.Thread(target=self.server.serve_forever, name='http', daemon=True).start()
        self.get_logger().info('Dashboard on port %d' % self.port)

    # ---- fixed things: the map, the tags, the stations ----
    @staticmethod
    def load_layout(map_file, tags_file, stations_file):
        with open(map_file) as handle:
            meta = yaml.safe_load(handle)
        image = cv2.imread(os.path.join(os.path.dirname(map_file), meta['image']), cv2.IMREAD_GRAYSCALE)
        png = cv2.imencode('.png', image)[1].tobytes() if image is not None else b''
        with open(tags_file) as handle:
            tags = yaml.safe_load(handle).get('tags') or {}
        with open(stations_file) as handle:
            stations = yaml.safe_load(handle).get('stations') or {}
        layout = {
            'resolution': float(meta['resolution']), 'origin': [float(v) for v in meta['origin'][:2]],
            'width': 0 if image is None else int(image.shape[1]), 'height': 0 if image is None else int(image.shape[0]),
            'tags': [{'id': int(i), 'x': float(v['x']), 'y': float(v['y']), 'yaw_deg': float(v.get('yaw_deg', 0.0))}
                     for i, v in tags.items()],
            'stations': [],
        }
        for name, entry in stations.items():
            pair = [tags.get(i) for i in entry['tag_ids']]
            if all(pair):
                layout['stations'].append({'name': name, 'x': (pair[0]['x'] + pair[1]['x']) / 2.0,
                                           'y': (pair[0]['y'] + pair[1]['y']) / 2.0})
        return layout, png

    # ---- the fast topics, only while watched ----
    def start_feed(self) -> None:
        t = self.telemetry
        node = rclpy.create_node('viz_feed')
        sensor = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Odometry, 'odometry/global', self.on_odometry, 2)
        node.create_subscription(Path, 'plan', lambda m: t.set('path', thin_path(
            [(q.pose.position.x, q.pose.position.y) for q in m.poses])), 2)
        for name in RANGES:
            node.create_subscription(Range, 'range/' + name,
                                     lambda m, n=name: t.set('range_' + n, round(float(m.range), 3)), 2)
        node.create_subscription(TrayState, 'tray/state', lambda m: t.set('tray', {
            'state': TRAY_STATES.get(m.state, '?'), 'occupied_fraction': round(m.occupied_fraction, 3),
            'stable': m.stable, 'has_reference': m.has_reference}), 2)
        for name in CAMERAS:
            node.create_subscription(TagDetectionArray, 'tags/%s/detections' % name,
                                     lambda m, n=name: t.set('tags_' + n, sorted(d.id for d in m.detections)), 2)
            node.create_subscription(CompressedImage, 'tags/%s/debug/compressed' % name,
                                     lambda m, n=name: self.on_image(n, m), sensor)
        node.create_subscription(CompressedImage, 'tray/debug/compressed', lambda m: self.on_image('tray', m), sensor)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        stop = threading.Event()

        def spin():
            while rclpy.ok() and not stop.is_set():
                executor.spin_once(timeout_sec=0.2)
            # Taken apart by the thread that used it; nothing else touches this node.
            executor.remove_node(node)
            executor.shutdown(timeout_sec=0.0)
            node.destroy_node()

        thread = threading.Thread(target=spin, name='viz_feed', daemon=True)
        thread.start()
        self.feed = (stop, thread)

    def stop_feed(self) -> None:
        stop, thread = self.feed
        self.feed = None
        stop.set()
        thread.join(timeout=2.0)

    # ---- incoming ----
    def on_mission(self, msg: MissionState) -> None:
        self.telemetry.set('mission', {'state': msg.state, 'detail': msg.detail})
        if msg.state != self.previous_state:
            self.telemetry.event('%s%s' % (msg.state, (' (%s)' % msg.detail) if msg.detail else ''))
            self.previous_state = msg.state

    def on_odometry(self, msg: Odometry) -> None:
        pose, twist = msg.pose.pose, msg.twist.twist
        self.telemetry.set('pose', [round(pose.position.x, 3), round(pose.position.y, 3),
                                    round(yaw_of(pose.orientation.z, pose.orientation.w), 3)])
        self.telemetry.set('velocity', [round(twist.linear.x, 3), round(twist.linear.y, 3), round(twist.angular.z, 3)])

    def on_image(self, name: str, msg: CompressedImage) -> None:
        self.images[name] = bytes(msg.data)
        self.telemetry.set('image_' + name, True)

    def on_log(self, msg: Log) -> None:
        if msg.name == self.get_name():
            return
        if msg.level >= 30:
            self.telemetry.log('ERROR' if msg.level >= 40 else 'WARN', msg.name, msg.msg)
        elif msg.name in LOUD_NODES:
            if msg.name == 'speech_node' and msg.msg.startswith('Saying'):
                self.telemetry.event(msg.msg)
            elif not msg.msg.startswith('{'):          # the mission's own JSON log is shown as events
                self.telemetry.log('INFO', msg.name, msg.msg)

    def scene_tables(self, scene_file: str) -> None:
        """Names and rectangles of a simulated scene's tables, for labels on the map."""
        if not scene_file:
            return
        with open(scene_file) as handle:
            scene = yaml.safe_load(handle) or {}
        self.layout['tables'] = [{'name': t.get('name', 'table'), 'rect': [float(v) for v in t['rect']]}
                                 for t in scene.get('tables') or []]

    def simulated_room(self):
        """Where the simulated people are (written by the fake Mega), or None on a real robot."""
        try:
            with open(self.sim_file) as handle:
                room = json.load(handle)
        except (OSError, ValueError):
            return None
        return room if time.time() - room.get('t', 0.0) < 2.0 else None

    def on_second(self) -> None:
        self.telemetry.set('system', system_stats())
        room = self.simulated_room()
        if room and room.get('contacts', 0) > self.sim_contacts:
            self.telemetry.event('SIMULATION: the robot touched %s' % (room.get('touching') or 'something'))
        self.sim_contacts = room.get('contacts', 0) if room else 0
        idle = time.monotonic() - self.last_request
        if idle < 3.0:
            self.watching.publish(Bool(data=True))
            if self.feed is None:
                self.start_feed()
        elif idle > 10.0 and self.feed is not None:
            self.stop_feed()

    # ---- web ----
    def handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def reply(self, body: bytes, kind: str, status: int = 200) -> None:
                self.send_response(status)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                node.last_request = time.monotonic()
                path = self.path.split('?')[0]
                try:
                    if path in ('/', '/index.html'):
                        with open(os.path.join(node.web, 'index.html'), 'rb') as handle:
                            self.reply(handle.read(), 'text/html; charset=utf-8')
                    elif path == '/state':
                        state = node.telemetry.snapshot()
                        state['sim'] = node.simulated_room()
                        state['boot'] = node.boot      # a page left open reloads when this changes
                        self.reply(json.dumps(state).encode(), 'application/json')
                    elif path == '/layout':
                        self.reply(json.dumps(node.layout).encode(), 'application/json')
                    elif path == '/map.png':
                        self.reply(node.map_png, 'image/png')
                    elif path.startswith('/cam/') and path[5:] in node.images:
                        self.reply(node.images[path[5:]], 'image/jpeg')
                    else:
                        self.reply(b'not found', 'text/plain', 404)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass   # no line per request in the robot's log

        return Handler


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
