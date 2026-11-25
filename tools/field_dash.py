"""Field dashboard for the Mac (25 Nov 2026): three camera views, the detection switch, live readings,
the mission log and hand control (W/S A/D Q/E). Started by ./handctr.sh on the Mac; runs in the robot's
container next to the robot stack and is reached through an SSH tunnel at http://localhost:8090.

    python3 -u /nontok/tools/field_dash.py            dashboard + hand control (mission_run.py must be stopped)
    python3 -u /nontok/tools/field_dash.py --view     dashboard only: no arming, no driving

Camera pictures come from the nodes that own the cameras (tag detectors, tray node): they send them while
someone watches (viz/watching). Detection on = tag outlines on the front views and the item's box, label and
station on the tray view (tray node 'detect').

Hand control safety: the robot stops when the page goes quiet for DEADMAN_S (a lost link, a closed tab);
motion toward anything the front or side ultrasonics see within GUARD_M is refused; speeds are capped; it
disarms after IDLE_DISARM_S with no keys and when it exits. While armed it sends /cmd_vel at 20 Hz (zero
when idle): the Mega disarms itself after 300 ms without a command.
"""
import argparse
import collections
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PORT = 8090
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'field_dash.html')
MISSION_LOG = '/tmp/mission.log'
CAMERAS = {'front_left': 'tags/front_left/debug/compressed', 'front_right': 'tags/front_right/debug/compressed',
           'tray': 'tray/debug/compressed'}
DETECTORS = ('tag_detector_front_left', 'tag_detector_front_right')
RANGES = ('front', 'left45', 'right45', 'left', 'right')
PICTURE_PERIOD_S = 0.25   # 4 pictures a second per camera while the page is open
DEADMAN_S = 0.5           # no message from the page for this long -> stop
IDLE_DISARM_S = 60.0      # armed with no keys for this long -> disarm
GUARD_M = 0.15            # refuse motion toward an echo closer than this
MAX_SPEED = 0.25          # m/s, forward and sideways
MAX_TURN = 1.0            # rad/s
NO_ECHO = 1.45


def unit(value) -> float:
    try:
        return max(-1.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0


def limited(value, cap: float, default: float) -> float:
    try:
        return max(0.0, min(cap, float(value)))
    except (TypeError, ValueError):
        return default


class HandLogic:
    """Pure logic: keys + ranges + time -> the velocity to send and what was refused."""

    def __init__(self):
        self.keys = (0.0, 0.0, 0.0)
        self.speed, self.turn = 0.10, 0.5
        self.last_msg = 0.0
        self.last_motion = 0.0
        self.ranges = {}          # name -> (metres or None, time)

    def on_drive(self, query: dict, now: float) -> None:
        self.keys = (unit(query.get('vx')), unit(query.get('vy')), unit(query.get('wz')))
        self.speed = limited(query.get('speed'), MAX_SPEED, self.speed)
        self.turn = limited(query.get('turn'), MAX_TURN, self.turn)
        self.last_msg = now
        if any(self.keys):
            self.last_motion = now

    def near(self, name: str, now: float) -> bool:
        r, t = self.ranges.get(name, (None, 0.0))
        return r is not None and r < GUARD_M and now - t < 1.0

    def command(self, now: float):
        """(vx, vy, wz, blocked list)."""
        if now - self.last_msg > DEADMAN_S:
            return 0.0, 0.0, 0.0, []
        kx, ky, kw = self.keys
        vx, vy, wz = kx * self.speed, ky * self.speed, kw * self.turn
        blocked = []
        if vx > 0 and self.near('front', now):
            vx = 0.0
            blocked.append('forward (front %.2f m)' % self.ranges['front'][0])
        if vy > 0 and self.near('left', now):
            vy = 0.0
            blocked.append('left (side %.2f m)' % self.ranges['left'][0])
        if vy < 0 and self.near('right', now):
            vy = 0.0
            blocked.append('right (side %.2f m)' % self.ranges['right'][0])
        return vx, vy, wz, blocked


def mission_tail(lines: int = 40):
    """The newest mission log lines, without the every-30-s hardware lines."""
    try:
        with open(MISSION_LOG, 'rb') as handle:
            handle.seek(0, os.SEEK_END)
            handle.seek(max(0, handle.tell() - 20000))
            text = handle.read().decode('utf-8', 'replace').splitlines()[1:]
    except OSError:
        return []
    return [line.rstrip() for line in text if line.strip() and 'HW ok' not in line][-lines:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--view', action='store_true', help='dashboard only: no arming, no driving')
    args = ap.parse_args()

    import math
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Range
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger
    from nontok_interfaces.msg import BaseStatus, TagDetectionArray, TrayState

    rclpy.init()
    node = Node('field_dash')
    logic = HandLogic()
    lock = threading.Lock()
    state = {'armed': False, 'base_t': -1e9, 'fault': '', 'battery_v': 0.0, 'tray_deg': 0.0, 'oled': '',
             'pose': None, 'pose_t': -1e9, 'vel': [0.0, 0.0, 0.0], 'tags': {}, 'tray': None, 'tray_t': -1e9,
             'detection': None, 'detection_t': -1e9, 'message': '', 'detect': False, 'tray_camera': False}
    images = {}                                   # camera -> (jpeg bytes, time)
    history = collections.deque(maxlen=600)       # (t, front, left45, right45) for the chart
    last_page = [-1e9]

    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    watching = node.create_publisher(Bool, 'viz/watching', 2)
    arm_client = node.create_client(SetBool, 'base/arm')
    tray_enable = node.create_client(SetBool, 'tray/enable')
    tray_reference = node.create_client(Trigger, 'tray/capture_reference')
    param_clients = {name: node.create_client(SetParameters, '/%s/set_parameters' % name)
                     for name in DETECTORS + ('tray_node',)}

    def on_range(name, msg):
        now = time.monotonic()
        with lock:
            logic.ranges[name] = (float(msg.range) if msg.range < NO_ECHO else None, now)
            if name == 'front':
                r = {k: logic.ranges.get(k, (None, 0))[0] for k in ('front', 'left45', 'right45')}
                history.append((time.time(), r['front'], r['left45'], r['right45']))

    for name in RANGES:
        node.create_subscription(Range, 'range/' + name, lambda m, n=name: on_range(n, m), 5)

    def on_status(msg):
        with lock:
            state.update(armed=bool(msg.armed), fault=msg.fault, battery_v=float(msg.battery_v),
                         tray_deg=float(msg.tray_deg), base_t=time.monotonic())

    def on_odom(msg):
        q, t = msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with lock:
            state.update(pose=[msg.pose.pose.position.x, msg.pose.pose.position.y, yaw], pose_t=time.monotonic(),
                         vel=[t.linear.x, t.linear.y, t.angular.z])

    def on_image(name, msg):
        images[name] = (bytes(msg.data), time.monotonic())

    def on_tags(name, msg):
        with lock:
            state['tags'][name] = ([d.id for d in msg.detections], time.monotonic())

    def on_tray(msg):
        with lock:
            state.update(tray={'state': {0: 'EMPTY', 1: 'OCCUPIED', 2: 'MOVING'}.get(msg.state, '?'),
                               'fraction': round(float(msg.occupied_fraction), 3), 'stable': bool(msg.stable),
                               'reference': bool(msg.has_reference)}, tray_t=time.monotonic())

    def on_detection(msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        with lock:
            state.update(detection=value, detection_t=time.monotonic())

    node.create_subscription(BaseStatus, 'base/status', on_status, 5)
    node.create_subscription(Odometry, 'odometry/local', on_odom, 5)
    node.create_subscription(TrayState, 'tray/state', on_tray, 5)
    node.create_subscription(String, 'tray/detection', on_detection, 5)
    node.create_subscription(String, 'base/oled', lambda m: state.update(oled=m.data), 5)
    for cam, topic in CAMERAS.items():
        node.create_subscription(CompressedImage, topic, lambda m, c=cam: on_image(c, m), 1)
    for cam in ('front_left', 'front_right'):
        node.create_subscription(TagDetectionArray, 'tags/%s/detections' % cam, lambda m, c=cam: on_tags(c, m), 2)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # ---- helpers ----
    def wait(future, timeout):
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def call(client, request, timeout=6.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None, 'no %s service (is the robot stack running?)' % client.srv_name
        result = wait(client.call_async(request), timeout)
        return (result, '') if result is not None else (None, '%s did not answer' % client.srv_name)

    def set_params(node_name, values: dict) -> str:
        msg = []
        for name, value in values.items():
            v = ParameterValue()
            if isinstance(value, bool):
                v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
            else:
                v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
            msg.append(Parameter(name=name, value=v))
        _, err = call(param_clients[node_name], SetParameters.Request(parameters=msg), 3.0)
        return err

    def send(vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        pub.publish(t)

    def arm(on: bool) -> str:
        if on and args.view:
            return 'view-only dashboard: start it with ./handctr.sh to drive'
        result, err = call(arm_client, SetBool.Request(data=on))
        if err:
            return err
        if on:
            with lock:
                logic.last_motion = time.monotonic()
        return '' if result.success else (result.message or 'refused')

    def set_detect(on: bool) -> str:
        errors = [e for e in (set_params(n, {'overlay': on}) for n in DETECTORS) if e]
        errors += [e for e in [set_params('tray_node', {'detect': on})] if e]
        state['detect'] = on
        return '; '.join(errors)

    def faster_pictures():
        """Ask the camera nodes for PICTURE_PERIOD_S pictures (they default to two a second)."""
        for name in DETECTORS:
            set_params(name, {'debug_period_s': PICTURE_PERIOD_S})
        set_params('tray_node', {'debug_period_s': PICTURE_PERIOD_S})
        set_detect(state['detect'])   # the detectors start with their outlines on: match the switch

    # ---- background: watching flag, driving, idle disarm ----
    def loop():
        was_moving, last_watch, tuned = False, 0.0, False
        while rclpy.ok():
            now = time.monotonic()
            if now - last_page[0] < 3.0 and now - last_watch > 1.0:
                watching.publish(Bool(data=True))
                last_watch = now
                if not tuned:
                    threading.Thread(target=faster_pictures, daemon=True).start()
                    tuned = True
            with lock:
                vx, vy, wz, blocked = logic.command(now)
                armed = state['armed']
                idle = now - logic.last_motion
            if args.view:
                vx = vy = wz = 0.0
            moving = bool(vx or vy or wz)
            if (armed and not args.view) or moving or was_moving:
                send(vx, vy, wz)
            was_moving = moving
            if armed and not args.view and idle > IDLE_DISARM_S:
                node.get_logger().info('no keys for %.0f s: disarming' % IDLE_DISARM_S)
                state['message'] = 'disarmed after %.0f s with no keys' % IDLE_DISARM_S
                arm(False)
                with lock:
                    logic.last_motion = now
            time.sleep(0.05)

    threading.Thread(target=loop, daemon=True).start()

    running = {'t': -1e9, 'value': False}

    def mission_running() -> bool:
        """Is tools/mission_run.py running? Checked at most every 3 s (a process start costs the Nano)."""
        now = time.monotonic()
        if now - running['t'] > 3.0:
            # [m]: the shell running pgrep has the pattern in its own command line, so must not match it
            running.update(t=now, value=os.system("pgrep -f '[m]ission_run.py' >/dev/null 2>&1") == 0)
        return running['value']

    def snapshot(now):
        with lock:
            vx, vy, wz, blocked = logic.command(now)
            ranges = {k: (None if v[0] is None else round(v[0], 3)) for k, v in logic.ranges.items() if now - v[1] < 1.0}
            tags = {k: v[0] for k, v in state['tags'].items() if now - v[1] < 1.5}
            pose = state['pose'] if now - state['pose_t'] < 2.0 else None
            cut = time.time() - 30.0
            chart = [h for h in history if h[0] >= cut][::2]
            return {
                'ok': True, 'view_only': args.view, 'armed': state['armed'], 'base_alive': now - state['base_t'] < 1.5,
                'fault': state['fault'], 'battery_v': round(state['battery_v'], 2), 'tray_deg': round(state['tray_deg'], 1),
                'oled': state['oled'], 'pose': None if pose is None else [round(v, 3) for v in pose],
                'vel': [round(v, 3) for v in state['vel']], 'ranges': ranges, 'tags': tags,
                'tray': state['tray'] if now - state['tray_t'] < 2.0 else None,
                'detection': state['detection'] if now - state['detection_t'] < 2.0 else None,
                'detect': state['detect'], 'tray_camera': state['tray_camera'],
                'cameras': {c: round(now - images[c][1], 1) if c in images else None for c in CAMERAS},
                'cmd': [round(vx, 3), round(vy, 3), round(wz, 3)], 'blocked': blocked, 'message': state['message'],
                'chart': [[round(h[0], 2), h[1], h[2], h[3]] for h in chart],
                'log': mission_tail(),
                'mission_running': mission_running(),
                'limits': {'speed': MAX_SPEED, 'turn': MAX_TURN, 'deadman_s': DEADMAN_S, 'guard_m': GUARD_M}}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'     # keep-alive: one connection through the SSH tunnel, not one per key

        def send_body(self, body: bytes, kind: str, status: int = 200):
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def reply(self, data: dict, status: int = 200):
            self.send_body(json.dumps(data).encode(), 'application/json', status)

        def do_GET(self):
            url = urlparse(self.path)
            query = {k: v[-1] for k, v in parse_qs(url.query).items()}
            now = time.monotonic()
            last_page[0] = now
            try:
                if url.path in ('/', '/index.html'):
                    with open(PAGE, 'rb') as handle:
                        self.send_body(handle.read(), 'text/html; charset=utf-8')
                elif url.path == '/api/state':
                    self.reply(snapshot(now))
                elif url.path.startswith('/api/cam/'):
                    item = images.get(url.path[9:])
                    if item is None:
                        self.send_body(b'', 'image/jpeg', 204)
                    else:
                        self.send_body(item[0], 'image/jpeg')
                elif url.path == '/api/detect':
                    err = set_detect(query.get('on') == '1')
                    state['message'] = err or 'detection %s' % ('on' if state['detect'] else 'off')
                    self.reply({'ok': not err, 'message': state['message']})
                elif url.path == '/api/tray_camera':
                    on = query.get('on') == '1'
                    result, err = call(tray_enable, SetBool.Request(data=on), 15.0)
                    err = err or ('' if result.success else (result.message or 'refused'))
                    if not err:
                        state['tray_camera'] = on
                    state['message'] = err or 'tray camera %s' % ('on' if on else 'off')
                    self.reply({'ok': not err, 'message': state['message']})
                elif url.path == '/api/reference':
                    result, err = call(tray_reference, Trigger.Request(), 10.0)
                    err = err or ('' if result.success else (result.message or 'refused'))
                    state['message'] = err or 'empty-tray reference taken'
                    self.reply({'ok': not err, 'message': state['message']})
                elif url.path == '/hand/drive':
                    with lock:
                        logic.on_drive(query, now)
                    self.reply({'ok': True})
                elif url.path == '/hand/arm':
                    on = query.get('on') == '1'
                    message = arm(on)
                    state['message'] = message or ('armed' if on else 'disarmed')
                    self.reply({'ok': not message, 'message': state['message']})
                else:
                    self.reply({'ok': False, 'message': 'not found'}, 404)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', PORT), Handler)
    server.daemon_threads = True
    node.get_logger().info('Field dashboard on port %d%s' % (PORT, ' (view only)' if args.view else ''))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with lock:
            logic.last_msg = 0.0
        for _ in range(10):
            send()
            time.sleep(0.05)
        if not args.view:
            arm(False)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
