"""Voice-called return run (24 Nov 2026), scripted routes in the real room, no Nav2. Start docked.
"Nontok" -> dock to pickup -> empty-tray reference, ask for the item, classify -> its station
(electrical, mechanical or tools), dump
-> back to the dock. Speaks only while standing still (the speaker shares the USB hub with the cameras).
Before every step it checks the USB devices that step needs (front cameras, Mega, tray camera, speaker, mic);
a silent one gets its node restarted, then its USB port reset (sysfs authorized 0/1), then the step is retried.
It never stops to ask for help: no item, an unknown item or a mechanical item -> it goes home.
Run in the container with base bridge, localization, dock_server, tray_node and wake_node."""
import math
import glob
import os
import signal
import struct
import subprocess
import threading
import time
import wave

import numpy as np
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger

from nontok_interfaces.action import DockToTags
from nontok_interfaces.msg import TagDetectionArray, TrayState
from nontok_interfaces.srv import ClassifyItem, SetTray

SOUNDS = '/nontok/ros2_ws/src/nontok_voice/sounds/'
SPEAKER = 'plughw:CARD=Audio,DEV=0'
BEEP = '/tmp/parking_beep.wav'
BEEP_WHILE_MOVING = False   # 24 Nov: the speaker shares the USB hub with the cameras; they dropped out while it beeped
NO_ECHO = 1.45          # the front ultrasonic reports 1.5 m for "nothing in range"
ULTRA_TRUST_M = 1.20    # the front ultrasonic is trusted nearer than this (it flickers at 1.4-1.5 m)
# 25 Nov: the front ultrasonic alone can take the near edge of a box for its back wall and stop the robot short,
# so the tray dumps in front of the box. The two 45 deg ultrasonics sit low and see the box lip only when the
# robot is right at the box. Before a dump the working ones must read close, or the wheels must stall against
# the box (confirm_at_box).
LIP_NEAR = 0.12         # both 45 deg sensors at or under this = at the box   TODO tune on the real boxes
LIP_EXTRA = 0.15        # at most this much further in on the encoders to find the lip
LIP_MEMORY_S = 10.0     # under 20 mm the sensors report no echo: their last echo, if close and this recent, counts
                        # (long enough to cover a whole push-in; any later real echo replaces it)
LIP_SENSORS = ('left45', 'right45')
T0 = time.time()


# ---- live data for the Mac: tools/live_dash.py streams LIVE_FILE (./run_dash.sh). Small: no pictures. ----
LIVE_FILE = '/tmp/nontok_live.json'
LIVE_LAYOUT = '/tmp/nontok_live_layout.json'
LIVE = {'stage': 'Starting', 'step': '', 'try': 0, 'plan': None, 'located': None, 'item': '', 'station': '',
        'thinking': [], 'started': time.time()}
STAGES = ('Listening', 'To pickup', 'Parking at pickup', 'Item & classify', 'To station', 'Parking at station',
          'Dumping', 'Going home', 'Docking')


def stage_of(step_name):
    n = step_name
    if n.startswith('wait for the wake'):
        return 'Listening'
    if n in ('leave the dock', 'approach the pickup'):
        return 'To pickup'
    if n == 'park at the pickup':
        return 'Parking at pickup'
    if n == 'get the item':
        return 'Item & classify'
    if n == 'park at the dock' or n == 'home: park at the dock':
        return 'Docking'
    if n.startswith('home:') or n.startswith('leave') or n in ('approach the dock', 'mechanical to the dock line'):
        return 'Going home'
    if n.startswith('park at'):
        return 'Parking at station'
    if n == 'dump':
        return 'Dumping'
    if n.startswith('pickup to') or n.startswith('approach'):
        return 'To station'
    return LIVE['stage']


def plan(name, points):
    """The route the robot is about to drive, in map metres, for the live view."""
    LIVE['plan'] = {'name': name, 'points': [[round(x, 3), round(y, 3)] for x, y in points], 't': time.time()}


def log(s):
    print('%6.1fs  %s' % (time.time() - T0, s), flush=True)
    LIVE['thinking'] = (LIVE['thinking'] + [[round(time.time(), 2), s]])[-14:]


def make_beep():
    rate = 22050
    frames = []
    for i in range(int(0.12 * rate)):            # 120 ms at 1.2 kHz, faded in and out
        env = min(1.0, i / 200.0, (0.12 * rate - i) / 200.0)
        frames.append(int(20000 * env * math.sin(2 * math.pi * 1200 * i / rate)))
    frames += [0] * int(0.28 * rate)             # then 280 ms quiet: ~2.5 beeps a second
    with wave.open(BEEP, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(struct.pack('<%dh' % len(frames), *frames))


def show(text):
    """State on the Mega's OLED (one 16-character line under "NONTOK SSF")."""
    try:
        oled_pub.publish(String(data=text[:16]))
    except Exception:
        pass
    st['oled'] = text[:16]


def say(name):
    path = SOUNDS + name + '.wav'
    if os.path.exists(path):
        log('SAY %s' % name)
        try:
            subprocess.run(['aplay', '-q', '-D', SPEAKER, path], timeout=15)
        except Exception as e:   # a missing speaker must never stop the run
            log('  (speaker: %s)' % e)


class Beeper:
    def __init__(self):
        self.on = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            if self.on and BEEP_WHILE_MOVING:
                subprocess.run(['aplay', '-q', '-D', SPEAKER, BEEP], timeout=5)
            else:
                time.sleep(0.05)


make_beep()
beeper = Beeper()
rclpy.init()
n = Node('home_from_pickup')
st = {'x': None, 'y': None, 'yaw': None, 'range': None, 'tags': {}, 'u': {}, 'c': {}, 'tray': None, 'wake': 0,
      'lip': {name: {'t': 0.0, 'echo': None, 'echo_t': 0.0, 'ever': False} for name in LIP_SENSORS}}
# camera: (x, y, viewing yaw rad, fx, cx) from robot.yaml and the 24 Nov calibrations
CAMS = {'front_left': (0.12, 0.080, math.radians(-1.1), 845.0, 627.8),
        'front_right': (0.12, -0.080, math.radians(-3.4), 840.0, 598.6)}
HALF_SPACING = 0.15   # every tag pair is 30 cm apart (centre to centre); the lower id is on the left


def on_odom(m):
    q = m.pose.pose.orientation
    st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y
    st['odom_t'] = time.time()
    st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_tags(cam, m):
    now = time.time()
    st['tags'][cam] = (now, [d.id for d in m.detections])
    st['u'][cam] = (now, {d.id: sum(d.corners[0::2]) / 4.0 for d in m.detections})
    st['c'][cam] = (now, {d.id: list(d.corners) for d in m.detections})


n.create_subscription(Odometry, 'odometry/local', on_odom, 20)


def on_global(m):
    q = m.pose.pose.orientation
    st['gpose'] = (m.pose.pose.position.x, m.pose.pose.position.y,
                   math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    st['gpose_t'] = time.time()


n.create_subscription(Odometry, 'odometry/global', on_global, 5)
n.create_subscription(Range, 'range/front', lambda m: (st.__setitem__('range', m.range), st.__setitem__('range_t', time.time())), 5)


def on_lip(name, m):
    """A 45 deg reading. 'echo' keeps the newest real echo: after a close one the sensor may go silent because the
    lip is now under its 20 mm minimum, and the bridge then reports 1.5 m ("clear") - that must not count as far."""
    s = st['lip'][name]
    s['t'] = time.time()
    if m.range < NO_ECHO:
        s['echo'], s['echo_t'], s['ever'] = m.range, s['t'], True


for name in LIP_SENSORS:
    n.create_subscription(Range, 'range/' + name, lambda m, k=name: on_lip(k, m), 5)
for cam in ('front_left', 'front_right'):
    n.create_subscription(TagDetectionArray, 'tags/%s/detections' % cam,
                          lambda m, c=cam: on_tags(c, m), 5)
pub = n.create_publisher(Twist, 'cmd_vel', 10)
arm = n.create_client(SetBool, 'base/arm')
dock = ActionClient(n, DockToTags, 'dock_to_tags')
n.create_subscription(TrayState, 'tray/state', lambda m: (st.__setitem__('tray', m), st.__setitem__('tray_t', time.time())), 5)
n.create_subscription(Empty, 'wake', lambda m: st.__setitem__('wake', st['wake'] + 1), 5)
voice_enable = n.create_client(SetBool, 'voice/enable')
tray_enable = n.create_client(SetBool, 'tray/enable')
tray_reference = n.create_client(Trigger, 'tray/capture_reference')
classify = n.create_client(ClassifyItem, 'tray/classify')
set_tray = n.create_client(SetTray, 'base/set_tray')
oled_pub = n.create_publisher(String, 'base/oled', 5)
LENSES = {}   # filled on first use (tag_poses)
ex = rclpy.executors.MultiThreadedExecutor()
ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()


class Abort(Exception):
    pass


def call(client, request, timeout=10.0):
    if not client.wait_for_service(timeout_sec=5.0):
        raise Abort('service %s not available' % client.srv_name)
    f = client.call_async(request)
    end = time.time() + timeout
    while not f.done() and time.time() < end:
        time.sleep(0.05)
    if not f.done():
        raise Abort('service %s did not answer' % client.srv_name)
    return f.result()


def send(vx=0.0, wz=0.0, vy=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
    pub.publish(t)


def stop(sec=0.2):
    end = time.time() + sec
    while time.time() < end:
        send()
        time.sleep(0.05)
    beeper.on = False


def set_arm(on):
    ok = False
    for _ in range(2):
        ok = call(arm, SetBool.Request(data=on), 5).success
        if ok:
            break
        time.sleep(1.0)
    log('arm(%s) -> %s' % (on, ok))
    if on and not ok:
        raise Abort('arm failed')
    return ok


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


PARK_SPEED = 0.12      # m/s: parking and short moves (was 0.08)
TRAVEL_SPEED = 0.40    # m/s: legs between stations - the firmware's wheel limit (MAX_WHEEL_MM_S 400)
TRAVEL_ACCEL = 0.30    # m/s per s: ramp up, the firmware's ACCEL_MM_S2; faster slips the omni wheels
DECEL = 0.25           # m/s per s: speed is held to sqrt(2 DECEL x room left), so it stops where asked
TURN_RATE = 1.2        # rad/s (firmware allows ~2: 400 mm/s / 0.1925 m)
TURN_DECEL = 1.0       # rad/s per s, the same idea for turns


def drive(dist, stop_front=None, limit=20.0, fast=False):
    """Straight on the encoders. fast: TRAVEL_SPEED with a ramp up, slowing for the end of the move and for
    anything the front ultrasonic sees (never faster than 0.8/s x the room left before stop_front or 0.15 m)."""
    x0, y0, yaw0 = st['x'], st['y'], st['yaw']
    sign = 1 if dist > 0 else -1
    top = TRAVEL_SPEED if fast else PARK_SPEED
    t0 = time.time()
    end = t0 + limit
    beeper.on = True
    while True:
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        r = st['range']
        if done >= abs(dist) - 0.005:
            break
        if sign > 0 and stop_front is not None and r is not None and r <= stop_front:
            break
        if sign > 0 and r is not None and r < 0.15:
            stop()
            raise Abort('front %.3f m < 0.15 m' % r)
        if time.time() > end:
            stop()
            raise Abort('drive: only %.2f of %.2f m in %.0f s' % (done, abs(dist), limit))
        v = min(top, math.sqrt(2 * DECEL * max(0.0, abs(dist) - done)))
        v = min(v, 0.06 + TRAVEL_ACCEL * (time.time() - t0))
        if sign > 0 and r is not None and r < NO_ECHO:   # slow for what the front sees: stop before it
            room = r - (stop_front if stop_front is not None else 0.15) - 0.02
            v = min(v, math.sqrt(2 * DECEL * max(0.0, room)))
        # hold the heading: 25 Nov a 2.2 m leg at travel speed ended 13 deg off and 0.4 m to the side
        send(vx=sign * max(0.04, v), wz=max(-0.3, min(0.3, -2.0 * wrap(st['yaw'] - yaw0))))
        time.sleep(0.05)
    stop()
    log('drove %+.3f m (encoders), front %.3f m' % (sign * math.hypot(st['x'] - x0, st['y'] - y0), st['range'] or -1))


def turn(deg, limit=12.0):
    turned, prev = 0.0, st['yaw']
    target = math.radians(deg)
    end = time.time() + limit
    beeper.on = True
    while True:
        turned += wrap(st['yaw'] - prev)
        prev = st['yaw']
        err = target - turned
        if abs(err) < math.radians(1.5):
            break
        if time.time() > end:
            stop()
            raise Abort('turn: %.1f of %.0f deg in %.0f s' % (math.degrees(turned), deg, limit))
        w = max(0.15, min(TURN_RATE, math.sqrt(2 * TURN_DECEL * abs(err)),
                          0.15 + 2.0 * (time.time() - (end - limit))))   # ramp up, decelerate into the target
        send(wz=w if err > 0 else -w)
        time.sleep(0.05)
    stop()
    log('turned %.1f deg (encoders)' % math.degrees(turned))


def creep_to(front, max_dist=0.40, limit=15.0):
    """Forward until the front reads `front`, at most max_dist, and only while the ultrasonic sees something."""
    x0, y0 = st['x'], st['y']
    end = time.time() + limit
    beeper.on = True
    while True:
        r = st['range']
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        if r is not None and r <= front + 0.005:
            break
        if r is None or r >= NO_ECHO:
            stop()
            raise Abort('creep: the front ultrasonic sees nothing ahead (%.2f m)' % (r or -1))
        if done >= max_dist:
            stop()
            raise Abort('creep: %.2f m and the front still reads %.3f m' % (done, r))
        if r < 0.15 or time.time() > end:
            stop()
            raise Abort('creep: front %.3f m, %.0f s' % (r, time.time() - (end - limit)))
        send(vx=0.05)
        time.sleep(0.05)
    stop()
    log('crept in: front %.3f m' % (st['range'] or -1))


def tags_seen(ids):
    now = time.time()
    return {c: v for c, (t, v) in st['tags'].items() if now - t < 1.0 and any(i in v for i in ids)}


def face_tags(ids):
    """If the map pose (odometry/global) is fresh and the tags are well off to one side, turn to face them.
    25 Nov: a strafe had turned the robot 143 deg and the +-40 deg search kept looking at the wrong wall,
    although the localization knew exactly where the robot was."""
    import yaml
    g = st.get('gpose')
    if g is None or time.time() - st.get('gpose_t', 0) > 1.0:
        return
    try:
        tags = yaml.safe_load(open(TAGS_FILE))['tags']
        cx = sum(tags[i]['x'] for i in ids) / len(ids)
        cy = sum(tags[i]['y'] for i in ids) / len(ids)
    except (OSError, KeyError, TypeError, ValueError):
        return
    off = wrap(math.atan2(cy - g[1], cx - g[0]) - g[2])
    if abs(off) > math.radians(30):
        log('tags %s are %+.0f deg off to the side by the map pose (%.2f, %.2f, %+.0f deg): turning to face them' % (
            ids, math.degrees(off), g[0], g[1], math.degrees(g[2])))
        turn(math.degrees(off), limit=20.0)


def find_tags(ids, max_deg=40, step=10):
    """Tags in view now, else face them by the map pose, else turn in steps (+10, -20, +30 ...) up to
    +-max_deg around here."""
    time.sleep(0.6)
    if tags_seen(ids):
        log('tags %s in view: %s' % (ids, tags_seen(ids)))
        return
    face_tags(ids)
    time.sleep(0.6)
    if tags_seen(ids):
        log('tags %s in view after facing them: %s' % (ids, tags_seen(ids)))
        return
    offset = 0
    for k in range(1, 2 * max_deg // step + 1):
        target = (k + 1) // 2 * step * (1 if k % 2 else -1)
        if abs(target) > max_deg:
            break
        turn(target - offset)
        offset = target
        time.sleep(0.6)
        if tags_seen(ids):
            log('tags %s found at %+d deg: %s' % (ids, offset, tags_seen(ids)))
            return
    turn(-offset)
    raise Abort('tags %s not found within +-%d deg' % (ids, max_deg))


TAG_SIZE = 0.16


def load_lens(cam):
    import yaml
    y = yaml.safe_load(open('/nontok/ros2_ws/src/nontok_localization/config/cameras/%s.yaml' % cam))
    return np.array(y['camera_matrix'], float).reshape(3, 3), np.array(y['distortion'], float)


H = TAG_SIZE / 2
TAG_CORNERS = np.array([[-H, H, 0], [H, H, 0], [H, -H, 0], [-H, -H, 0]], float)   # detector corner order


MOUNTS = {}   # camera -> 4x4 base_link <- optical, from robot.yaml (filled on first use)


def load_mount(cam):
    import yaml
    from nontok_localization.geometry import camera_mount
    c = yaml.safe_load(open('/nontok/ros2_ws/src/nontok_description/config/robot.yaml'))['cameras'][cam]
    return camera_mount(*c['xyz'], math.radians(c['yaw_deg']), math.radians(c.get('pitch_deg', 0.0)),
                        math.radians(c.get('roll_deg', 0.0)))


def tag_poses(corner_sets, lenses=None, mounts=None):
    """{cam: {id: [8 corners]}} -> {id: (x, y, wall_dx, wall_dy)} in the robot frame (x forward, y left):
    the tag centre and the unit direction into the wall, from cv2.solvePnP (IPPE_SQUARE) with the lens model,
    averaged over the cameras. Fits worse than 3 px are dropped."""
    import cv2
    if lenses is None:
        for cam in CAMS:
            if cam not in LENSES:
                LENSES[cam] = load_lens(cam)
        lenses = LENSES
    if mounts is None:
        for cam in CAMS:
            if cam not in MOUNTS:
                MOUNTS[cam] = load_mount(cam)
        mounts = MOUNTS
    acc = {}
    for cam, cs in corner_sets.items():
        if cam not in lenses or cam not in mounts:
            continue
        K, D = lenses[cam]
        T = mounts[cam]
        for i, corners in cs.items():
            img = np.array(corners, float).reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(TAG_CORNERS, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            err = np.linalg.norm(cv2.projectPoints(TAG_CORNERS, rvec, tvec, K, D)[0].reshape(4, 2) - img, axis=1).mean()
            if err > 3.0:
                continue
            centre = T @ np.r_[tvec.ravel(), 1.0]                    # tag centre in base_link
            d = T[:3, :3] @ -cv2.Rodrigues(rvec)[0][:, 2]            # normal points at the camera; minus = into the wall
            x, y, dx, dy = centre[0], centre[1], d[0], d[1]
            acc.setdefault(i, []).append((x, y, dx, dy))
    out = {}
    for i, v in acc.items():
        x, y, dx, dy = (sum(q[k] for q in v) / len(v) for k in range(4))
        nrm = math.hypot(dx, dy) or 1.0
        out[i] = (x, y, dx / nrm, dy / nrm)
    return out


def pair_geometry(poses, ids, half=0.15):
    """(skew_deg, middle_x, middle_y) of a tag pair: skew = turn that faces the wall square (+ = left),
    middle = the pair's centre. Both tags: from the line between them. One tag: from its own normal, the
    middle 15 cm along the wall from it (lower id = left tag). None if neither is seen."""
    left_id, right_id = min(ids), max(ids)
    if left_id in poses and right_id in poses:
        (lx, ly, _, _), (rx, ry, _, _) = poses[left_id], poses[right_id]
        wx, wy = lx - rx, ly - ry
        return math.degrees(math.atan2(-wx, wy)), (lx + rx) / 2, (ly + ry) / 2
    for i, sign in ((left_id, -1), (right_id, +1)):
        if i in poses:
            x, y, dx, dy = poses[i]
            ax, ay = -dy, dx                          # along the wall, toward the left tag
            return math.degrees(math.atan2(dy, dx)), x + sign * half * ax, y + sign * half * ay
    return None


def fresh_corners():
    now = time.time()
    return {cam: cs for cam, (t, cs) in st['c'].items() if now - t < 0.8}


STRAFE_GAIN = 0.6                     # fraction of the sideways error a centring strafe asks for
STRAFE_MAX_SLIP_DEG = 8.0             # heading change mid-strafe that stops it to turn back
STRAFE_MIN, STRAFE_MAX = 0.07, 0.14   # m/s: 0.04 left the omni wheels standing still sideways (24 Nov)


def strafe(dist, limit=15.0):
    """Sideways on the omni wheels: dist > 0 to the left, on the wheel odometry, holding the heading
    (an unheld 5 cm crab turned the robot ~5 deg). If it moves < 1 cm in 1.5 s the speed goes up in steps
    (below a threshold the wheels just stand still); still stuck at full speed = blocked: stop, report and
    carry on - the dock server corrects what is left."""
    x0, y0, yaw0 = st['x'], st['y'], st['yaw']
    sign = 1 if dist > 0 else -1
    end = time.time() + limit
    boost = 0.0
    last_t, last_done = time.time(), 0.0
    fixes = 0
    while True:
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        if done >= abs(dist) - 0.005:
            break
        slip = wrap(st['yaw'] - yaw0)
        if abs(slip) > math.radians(STRAFE_MAX_SLIP_DEG):
            # 25 Nov: with the wheels slipping at the boosted speed the hold lost and a 0.27 m crab turned the
            # robot 143 deg - then every tag search looked the wrong way. Stop, turn back, carry on.
            stop()
            fixes += 1
            log('strafe: heading slipped %+.0f deg - turning back (%d)' % (math.degrees(slip), fixes))
            turn(-math.degrees(slip), limit=20.0)
            if fixes >= 3:
                log('strafe: keeps turning - giving up after %.3f of %.3f m' % (done, abs(dist)))
                break
            boost = 0.0
            last_t, last_done = time.time(), math.hypot(st['x'] - x0, st['y'] - y0)
            continue
        if time.time() - last_t > 1.5:
            if done - last_done < 0.01:
                if STRAFE_MIN + boost >= STRAFE_MAX:
                    log('strafe blocked after %.3f of %.3f m at %.2f m/s - carrying on' % (done, abs(dist), STRAFE_MAX))
                    break
                boost += 0.03
                log('strafe: not moving, speed up to %.2f m/s' % min(STRAFE_MAX, STRAFE_MIN + boost))
            last_t, last_done = time.time(), done
        if time.time() > end:
            log('strafe: %.3f of %.3f m in %.0f s - carrying on' % (done, abs(dist), limit))
            break
        speed = min(STRAFE_MAX, max(STRAFE_MIN + boost, min(0.10, 0.8 * (abs(dist) - done))))
        hold = max(-0.25, min(0.25, -2.0 * wrap(st['yaw'] - yaw0)))
        send(vy=sign * speed, wz=hold)
        time.sleep(0.05)
    stop()
    log('strafed %+.3f m (encoders), heading change %+.1f deg' % (
        sign * math.hypot(st['x'] - x0, st['y'] - y0), math.degrees(wrap(st['yaw'] - yaw0))))
    if abs(wrap(st['yaw'] - yaw0)) > math.radians(3.0):
        turn(-math.degrees(wrap(st['yaw'] - yaw0)), limit=20.0)


def push_in(dist, limit=8.0):
    """Forward on the encoders only, ignoring the front sensor (it reads the box itself), slowly, until `dist`
    or the wheels stall against the box (< 5 mm in 1 s). 24 Nov: the mechanical drop landed ~20 cm short."""
    x0, y0 = st['x'], st['y']
    end = time.time() + limit
    last_t, last_done = time.time(), 0.0
    while True:
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        if done >= dist - 0.005:
            break
        if time.time() - last_t > 1.0:
            if done - last_done < 0.005:
                log('push: against something after %.3f m' % done)
                break
            last_t, last_done = time.time(), done
        if time.time() > end:
            break
        send(vx=0.05)
        time.sleep(0.05)
    stop()
    log('pushed in %.3f m (encoders), front %.3f m' % (math.hypot(st['x'] - x0, st['y'] - y0), st['range'] or -1))


def lip_seen(name):
    """True if this 45 deg sensor sees the box lip: its newest echo is close and recent (so also when it has
    since gone silent from being under 20 mm). Abort if the sensor has stopped reporting at all."""
    s, now = st['lip'][name], time.time()
    if now - s['t'] > 1.0:
        raise Abort('no reading from the %s ultrasonic' % name)
    return s['echo'] is not None and s['echo'] <= LIP_NEAR and now - s['echo_t'] <= LIP_MEMORY_S


def lip_text():
    out = []
    for name in LIP_SENSORS:
        s, now = st['lip'][name], time.time()
        if s['echo'] is None or now - s['echo_t'] > LIP_MEMORY_S:
            out.append('%s clear' % name)
        elif now - s['echo_t'] > 0.3:
            out.append('%s silent, last echo %.3f m' % (name, s['echo']))   # under 20 mm, or lost
        else:
            out.append('%s %.3f m' % (name, s['echo']))
    return ', '.join(out)


def lip_sensors():
    """The 45 deg sensors that work: one that has not given a single echo since the mission started is taken
    as broken (25 Nov: left45 read 1.5 m, i.e. no echo, all run long) and left out of the lip check."""
    working = [k for k in LIP_SENSORS if st['lip'][k]['ever']]
    for k in LIP_SENSORS:
        if k not in working:
            log('WARNING: the %s ultrasonic has given no echo since the start: left out of the lip check' % k)
    return working


def confirm_at_box(station):
    """Before a dump: make sure the robot is really at the box, not stopped short because the front ultrasonic
    read the box's near edge. At the box = every working low 45 deg sensor sees the lip, or the wheels stall
    against the box. Otherwise creep in on the encoders (front ignored - it may read the box itself), at most
    LIP_EXTRA. Neither lip nor box after that -> Abort, so the park step is retried instead of dumping short."""
    time.sleep(0.3)   # a fresh reading from both after stopping
    sensors = lip_sensors()
    if sensors and all(lip_seen(k) for k in sensors):
        log('%s: at the box (%s)' % (station, lip_text()))
        return
    log('%s: the box lip is not seen (%s, front %.3f m): creeping in' % (station, lip_text(), st['range'] or -1))
    x0, y0 = st['x'], st['y']
    end = time.time() + 10.0
    last_t, last_done = time.time(), 0.0
    stalled = lip = False
    while True:
        if sensors and all(lip_seen(k) for k in sensors):
            lip = True        # remembered: at 45 deg the lip edge is caught on and off (25 Nov: refused a good park)
            break
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        if done >= LIP_EXTRA or time.time() > end:
            break
        if time.time() - last_t > 1.0:
            if done - last_done < 0.005:
                stalled = True
                break
            last_t, last_done = time.time(), done
        send(vx=0.04)
        time.sleep(0.05)
    stop()
    time.sleep(0.3)
    done = math.hypot(st['x'] - x0, st['y'] - y0)
    if stalled:
        log('%s: at the box - the wheels stalled against it after %.3f m more (%s)' % (station, done, lip_text()))
    elif lip or (sensors and all(lip_seen(k) for k in sensors)):
        log('%s: at the box - lip seen after %.3f m more (%s)' % (station, done, lip_text()))
    else:
        raise Abort('%s: neither the box lip nor the box after %.3f m more (%s); not dumping short' % (
            station, done, lip_text()))


# ---- the robot's map pose for the live view: anchored on tags, carried on wheel odometry in between ----
# 25 Nov: odometry/global equalled raw odometry (no tag fixes reached the EKF), so the live view put the
# docked robot inside the dock wall. The mission measures tag pairs all the time; each one fixes the pose.
ANCHOR = {}
_TAGS = {}
ANCHOR_SKIP = {(10, 11)}   # electrical tags: ~0.4 m off in tags.yaml (25 Nov loops) - not used until measured
FRONT_SENSOR_X = 0.13   # front ultrasonic ahead of the robot centre (robot.yaml)


def tags_map():
    if not _TAGS:
        import yaml
        _TAGS.update(yaml.safe_load(open(TAGS_FILE))['tags'])
    return _TAGS


def set_anchor(pose, src):
    if st.get('x') is None:
        return
    ANCHOR.update(map=tuple(pose), odom=(st['x'], st['y'], st['yaw']), t=time.time(), src=src)


def map_pose():
    """(x, y, heading) in the map: the last anchor moved on by the odometry since then; None before any."""
    if not ANCHOR or st.get('x') is None:
        return None
    (mx, my, mth), (ox, oy, oth) = ANCHOR['map'], ANCHOR['odom']
    dx, dy = st['x'] - ox, st['y'] - oy
    lx, ly = math.cos(oth) * dx + math.sin(oth) * dy, -math.sin(oth) * dx + math.cos(oth) * dy
    return (mx + math.cos(mth) * lx - math.sin(mth) * ly, my + math.sin(mth) * lx + math.cos(mth) * ly,
            wrap(mth + st['yaw'] - oth))


def anchor_at_dock():
    """Parked at the dock: square to its wall, centred, the front ultrasonic gives the distance."""
    r = st['range']
    if r is not None and r < 0.35:
        set_anchor((0.35 - FRONT_SENSOR_X - r, 0.0, 0.0), 'parked at the dock')


def measure(ids, secs=0.4):
    """The pair's (skew, middle x, middle y), the median over `secs` of fresh frames (both cameras)."""
    vals, end = [], time.time() + secs
    while time.time() < end:
        g = pair_geometry(tag_poses(fresh_corners()), ids)
        if g is not None:
            vals.append(g)
        time.sleep(0.1)
    if not vals:
        return None
    g = tuple(float(np.median([v[k] for v in vals])) for k in range(3))
    try:
        tags = tags_map()
        if all(i in tags for i in ids) and tuple(sorted(ids)) not in ANCHOR_SKIP:
            set_anchor(pose_from_pair(ids, g, tags), 'tags %s' % '/'.join(str(i) for i in ids))
    except Exception:
        pass
    return g


def square_and_center(ids, tries=3, skew_tol=3.0, side_tol=0.04):
    """Square to the wall (one turn), then crab until the pair's middle is straight ahead (one slide).
    Acts only beyond 3 deg / 4 cm so it does not chase noise: 25 Nov, 2-3 cm strafes twisted the robot
    3.5-5 deg, which then cost a turn and another measurement, 3-4 times per station."""
    for _ in range(tries):
        time.sleep(0.15)
        g = measure(ids)
        if g is None:
            log('square: no tag of %s in view, searching' % (ids,))
            find_tags(ids, max_deg=40)
            continue
        skew, mx, my = g
        log('square: wall skew %+.1f deg, pair middle %+.3f m sideways, %.2f m ahead' % (skew, my, mx))
        if abs(skew) > skew_tol:
            turn(max(-35.0, min(35.0, skew)))
            continue
        if abs(my) > side_tol:
            # 25 Nov: a strafe moved ~1.6x what was asked (-8 cm -> +5 cm -> -4 cm, three tries): ask for 60 %
            strafe(max(-0.40, min(0.40, STRAFE_GAIN * my)))
            continue
        return
    log('square: best effort after %d tries' % tries)


def visual_park(ids, contact):
    """No pose solve: square and centre, in to 0.45 m, one more check, then straight in on the encoders to the
    contact distance (closer than ~0.4 m a tag leaves a camera's view, so no centring there)."""
    log('visual park on %s to front %.3f m' % (ids, contact))
    square_and_center(ids, tries=2)
    r = st['range']
    if r is not None and r > 0.47:
        drive(min(0.30, r - 0.45), stop_front=0.45)
        square_and_center(ids, tries=2)
    for _ in range(3):    # 25 Nov: one pass stopped 1-3 cm short of the contact distance
        r = st['range'] or 0.3
        if r <= contact + 0.015:
            break
        drive(min(0.30, r - contact), stop_front=contact)
    log('parked by eye: front %.3f m' % (st['range'] or -1))


USE_DOCK_SERVER = False   # 25 Nov: it reported 'station tags not seen' at every station; the visual park did the work


def dock_centred(station, ids, guard, contact):
    dock_centred_inner(station, ids, guard, contact)
    WHERE['at'] = station
    if station == 'dock':
        anchor_at_dock()


def dock_centred_inner(station, ids, guard, contact):
    r = st['range']
    if r is not None and r < 0.45:   # a retry from close in: back out to where both tags are in view
        drive(-(0.55 - r))
    square_and_center(ids)
    if not USE_DOCK_SERVER:
        visual_park(ids, contact)
        return
    try:
        dock_at(station, guard)
    except Abort as e:
        if 'not seen' not in str(e) and 'lost the station tags' not in str(e):
            raise
        visual_park(ids, contact)   # 24 Nov: boxes near the pickup hide the tags in the last ~0.2 m


def dock_at(station, guard, limit=45.0):
    dock.wait_for_server(timeout_sec=5)
    last = [0.0]

    def fb(m):
        f = m.feedback
        if time.time() - last[0] > 1.0:
            last[0] = time.time()
            log('  dock %-8s lateral %+.3f m  distance %+.3f m  yaw %+.1f deg  front %.3f m' % (
                f.phase, f.lateral_error_m, f.distance_error_m, math.degrees(f.yaw_error_rad), st['range'] or -1))

    beeper.on = True
    gf = dock.send_goal_async(DockToTags.Goal(station=station, require_charge=False), feedback_callback=fb)
    end = time.time() + 5
    while not gf.done() and time.time() < end:
        time.sleep(0.05)
    gh = gf.result() if gf.done() else None
    if not gh or not gh.accepted:
        stop()
        raise Abort('dock goal rejected')
    rf = gh.get_result_async()
    end = time.time() + limit
    while not rf.done() and time.time() < end:
        if st['range'] is not None and 0 < st['range'] < guard:
            gh.cancel_goal_async()
            stop()
            raise Abort('front %.3f m < %.2f m while docking' % (st['range'], guard))
        time.sleep(0.05)
    stop()
    if not rf.done():
        gh.cancel_goal_async()
        raise Abort('docking at %s did not finish in %.0f s' % (station, limit))
    r = rf.result().result
    log('%s: success %s  lateral %+.3f m  distance %+.3f m  yaw %+.1f deg  %r' % (
        station, r.success, r.lateral_error_m, r.distance_error_m, math.degrees(r.yaw_error_rad), r.message))
    if not r.success:
        # 24 Nov: at the dock wall the tags leave the view, the server waits in FINAL and times out
        # although its last errors are inside tolerance - that is docked
        if ('timed out' in r.message and abs(r.lateral_error_m) <= 0.025 and abs(r.distance_error_m) <= 0.025
                and abs(math.degrees(r.yaw_error_rad)) <= 3.0):
            log('%s: timed out but inside tolerance - counted as docked' % station)
            return
        raise Abort('docking at %s failed: %s' % (station, r.message))



# ================= USB health: check before every step, restart / reset instead of asking for help =========
WAKE_WAIT_S = 900.0
ITEM_WAIT_S = 600.0
DEFAULT_PORTS = {'front_left': '1-2.1', 'front_right': '1-2.3', 'tray': '1-2.2', 'speaker': '1-2.4.3', 'mega': '1-2.4.2'}
DEVICES = {'front_left': '/dev/cam_front_left', 'front_right': '/dev/cam_front_right', 'tray': '/dev/cam_tray',
           'mega': '/dev/nontok_mega'}
NODES = {'front_left': '__node:=tag_detector_front_left', 'front_right': '__node:=tag_detector_front_right',
         'tray': 'nontok_perception/tray_node', 'mic': 'nontok_voice/wake_node', 'mega': 'nontok_base/base_bridge'}
RESPAWNED = ('front_left', 'front_right')   # the localization launch respawns the tag detectors
MIC_CAMERA = 'front_left'                    # the wake word listens on the first "XWF" camera's microphone
PORTS, ARGV = dict(DEFAULT_PORTS), {}
SHARE = '/nontok/ros2_ws/install/%s/share/%s/config/%s'
DEFAULT_ARGV = {   # used when a node is not running at the start (24 Nov: the wake node had to be restarted)
    'mic': ['ros2', 'run', 'nontok_voice', 'wake_node', '--ros-args', '--params-file',
            SHARE % ('nontok_voice', 'nontok_voice', 'voice.yaml')],
    'tray': ['ros2', 'run', 'nontok_perception', 'tray_node', '--ros-args', '--params-file',
             SHARE % ('nontok_perception', 'nontok_perception', 'tray.yaml')],
    'mega': ['ros2', 'run', 'nontok_base', 'base_bridge', '--ros-args', '--params-file',
             SHARE % ('nontok_base', 'nontok_base', 'base.yaml'), '-p', 'port:=/dev/nontok_mega']}
ALL_HW = ('mega', 'cams', 'speaker', 'mic', 'tray_hw')   # checked before every step


def usb_port(dev):
    try:
        real = os.path.basename(os.path.realpath(dev))
        if real.startswith('video'):
            return os.path.basename(os.path.realpath('/sys/class/video4linux/%s/device/..' % real))
        if real.startswith('ttyUSB'):
            return os.path.basename(os.path.realpath('/sys/class/tty/%s/device/../..' % real))
    except OSError:
        pass
    return None


def speaker_port():
    for c in glob.glob('/sys/class/sound/card*'):
        try:
            if open(c + '/id').read().strip() == 'Audio':
                return os.path.basename(os.path.realpath(c + '/device/..'))
        except OSError:
            pass
    return None


def procs(pattern):
    out = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            argv = [a.decode() for a in open('/proc/%s/cmdline' % pid, 'rb').read().split(b'\0') if a]
        except OSError:
            continue
        line = ' '.join(argv)
        if argv and pattern in line and 'mission_run' not in line and 'ros2 run' not in line and argv[0] != 'bash':
            out.append((int(pid), argv))
    return out


def learn():
    """Remember each device's USB port and each node's command line while everything works."""
    for name, dev in DEVICES.items():
        PORTS[name] = usb_port(dev) or PORTS[name]
    PORTS['speaker'] = speaker_port() or PORTS['speaker']
    for name, pattern in NODES.items():
        found = procs(pattern)
        if found:
            ARGV[name] = found[0][1]
    log('USB ports %s; nodes known: %s' % (PORTS, sorted(ARGV)))


def restart_node(name):
    found = procs(NODES[name])
    for pid, _ in found:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(2.0)
    for pid, _ in procs(NODES[name]):
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    if name in RESPAWNED:
        end = time.time() + 8
        while time.time() < end:
            if procs(NODES[name]):
                log('USB: %s node respawned' % name)
                return
            time.sleep(0.5)
    argv = ARGV.get(name) or DEFAULT_ARGV.get(name)
    if not argv:
        log('USB: no command line known for %s' % name)
        return
    subprocess.Popen(argv, stdout=open('/tmp/%s_restart.log' % name, 'a'), stderr=subprocess.STDOUT,
                     start_new_session=True)
    log('USB: %s node restarted' % name)


def usb_reset(name):
    port = PORTS.get(name)
    path = '/sys/bus/usb/devices/%s/authorized' % port
    if not port or not os.path.exists(path):
        log('USB: cannot reset %s (port %s)' % (name, port))
        return
    log('USB: resetting %s at port %s' % (name, port))
    try:
        open(path, 'w').write('0')
        time.sleep(1.5)
        open(path, 'w').write('1')
    except OSError as e:
        log('USB: reset failed: %s' % e)
        return
    dev = DEVICES.get(name)
    end = time.time() + 12
    while dev and time.time() < end and not os.path.exists(dev):
        time.sleep(0.3)
    time.sleep(2.0)


def wait_for(cond, sec):
    end = time.time() + sec
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.2)
    return cond()


def cam_ok(cam):
    return time.time() - st['tags'].get(cam, (0, []))[0] < 1.5


def mega_ok():   # the front ultrasonic comes only from the Mega (odometry keeps flowing from the EKF)
    return time.time() - st.get('range_t', 0) < 1.0 and time.time() - st.get('odom_t', 0) < 1.0


def tray_ok():
    return time.time() - st.get('tray_t', 0) < 2.0


def xwf_cards():
    cards = []
    for line in open('/proc/asound/cards'):
        if 'XWF' in line and line.strip()[:1].isdigit():
            cards.append(int(line.split()[0]))
    return cards


def mic_status():
    """Hardware check of the wake word: exactly one wake node, holding an XWF camera microphone open
    (24 Nov: a stale second node answered the service while its microphone was dead)."""
    nodes = procs(NODES['mic'])
    if len(nodes) != 1:
        return False, '%d wake nodes' % len(nodes)
    pid = nodes[0][0]
    try:
        fds = [os.readlink('/proc/%d/fd/%s' % (pid, f)) for f in os.listdir('/proc/%d/fd' % pid)]
    except OSError:
        return False, 'wake node gone'
    held = [c for c in xwf_cards() if '/dev/snd/pcmC%dD0c' % c in fds]
    if not held:
        return False, 'wake node holds no XWF microphone'
    if not voice_enable.wait_for_service(timeout_sec=2.0):
        return False, 'voice/enable not offered'
    return True, 'mic card %d' % held[0]


def mic_ok():
    return mic_status()[0]


def mic_camera():
    """Which front camera's USB port carries the microphone the wake node uses."""
    ok, text = mic_status()
    if ok:
        port = os.path.basename(os.path.realpath('/sys/class/sound/card%s/device/..' % text.split()[-1]))
        for cam in ('front_left', 'front_right'):
            if PORTS.get(cam) == port:
                return cam
    return 'front_left'


def tray_hw_ok():
    return os.path.exists('/dev/cam_tray') and bool(procs(NODES['tray']))


def fix(name, ok, settle, reset=True):
    """ok() false -> restart the node -> still false -> reset the USB port and restart the node again."""
    if wait_for(ok, 3.0):
        return True
    log('USB: %s silent: restarting its node' % name)
    show('USB FIX %s' % {'front_left': 'CAM L', 'front_right': 'CAM R'}.get(name, name.upper()))
    restart_node(name)
    if wait_for(ok, settle):
        log('USB: %s back' % name)
        return True
    if reset:
        usb_reset('speaker' if name == 'speaker' else name)
        restart_node(name)
        if name in ('front_left', 'front_right') and name == mic_camera():
            restart_node('mic')   # the wake word uses this camera's microphone
        if wait_for(ok, settle + 10):
            log('USB: %s back after a USB reset' % name)
            return True
    log('USB: %s still silent' % name)
    return False


LAST_CHECK = {'t': 0.0, 'needs': set()}


def check(needs):
    """Hardware check before every step: everything in ALL_HW plus what the step needs. Skipped when the
    same devices passed less than 8 s ago (back-to-back steps; the tray is always checked: it is switched)."""
    needs = set(needs) | set(ALL_HW)
    if 'tray' not in needs and needs <= LAST_CHECK['needs'] and time.time() - LAST_CHECK['t'] < 8.0:
        return
    bad = []
    if 'mega' in needs and not fix('mega', mega_ok, 10):
        bad.append('mega')
    if 'cams' in needs:
        for cam in ('front_left', 'front_right'):
            if not os.path.exists(DEVICES[cam]):
                usb_reset(cam)
            if not fix(cam, lambda c=cam: cam_ok(c), 15):
                bad.append(cam)
    if 'speaker' in needs and 'Audio' not in open('/proc/asound/cards').read():
        usb_reset('speaker')
        if 'Audio' not in open('/proc/asound/cards').read():
            bad.append('speaker')
    if 'mic' in needs and not mic_ok():
        log('USB: microphone: %s' % mic_status()[1])
        restart_node('mic')
        if not wait_for(mic_ok, 30):
            cam = mic_camera()
            usb_reset(cam)
            restart_node(cam)
            restart_node('mic')
            if not wait_for(mic_ok, 30):
                bad.append('mic')
    if 'tray_hw' in needs and not tray_hw_ok():
        if not os.path.exists('/dev/cam_tray'):
            usb_reset('tray')
        restart_node('tray')
        if not wait_for(tray_hw_ok, 30):
            bad.append('tray')
    if 'tray' in needs:
        def tray_on_ok():
            try:
                call(tray_enable, SetBool.Request(data=True), 5)
            except Abort:
                return False
            return wait_for(tray_ok, 3.0)
        if not fix('tray', tray_on_ok, 30):
            bad.append('tray')
    if bad:
        LAST_CHECK.update(t=0.0, needs=set())
        raise Abort('USB devices still silent: %s' % bad)
    LAST_CHECK.update(t=time.time(), needs=needs)
    log('HW ok: Mega (front %.2f m), cameras %s, speaker, %s, tray camera' % (
        st['range'] or -1, sorted(c for c in ('front_left', 'front_right') if cam_ok(c)), mic_status()[1]))


ARMED = {'on': False}
WHERE_FILE = '/tmp/nontok_where'   # survives a crash and restart by the supervisor


class Where(dict):
    """Last place it parked: 'dock', 'pickup', 'electrical', 'mechanical'; 'away' once it drives off."""
    def __setitem__(self, key, value):
        super().__setitem__(key, value)
        try:
            open(WHERE_FILE, 'w').write(value or '')
        except OSError:
            pass


WHERE = Where()
try:
    dict.__setitem__(WHERE, 'at', open(WHERE_FILE).read().strip() or None)
except OSError:
    dict.__setitem__(WHERE, 'at', None)


def arm_if(on):
    if ARMED['on'] != on:
        set_arm(on)
        ARMED['on'] = on


LABELS = {'home: back off': 'FIND DOCK', 'home: find the dock': 'FIND DOCK', 'home: approach the dock': 'FIND DOCK', 'home: park at the dock': 'DOCKING', 'leave the dock': 'TO PICKUP', 'approach the pickup': 'TO PICKUP', 'park at the pickup': 'PARK PICKUP', 'leave the pickup': 'LEAVE PICKUP', 'approach electrical': 'TO ELECTRICAL', 'park at electrical': 'PARK ELECTRICAL', 'leave electrical': 'GOING HOME', 'pickup to mechanical': 'TO MECHANICAL', 'approach the dock': 'GOING HOME', 'park at the dock': 'DOCKING', 'approach mechanical': 'TO MECHANICAL', 'park at mechanical': 'PARK MECHANICAL', 'mechanical to the dock': 'GOING HOME', 'pickup to electrical': 'TO ELECTRICAL', 'pickup to tools': 'TO TOOLS', 'approach tools': 'TO TOOLS', 'park at tools': 'PARK TOOLS', 'leave tools': 'GOING HOME'}   # 16-character OLED text per step


def step(name, fn, needs, motion=False, tries=4, oled=None):
    """Hardware check, then the step; armed only for driving steps and kept armed between consecutive ones
    (24 Nov: disarm / re-arm around every step cost ~1 s each). Retried up to `tries` times."""
    for k in range(1, tries + 1):
        LIVE.update(step=name, stage=stage_of(name), **{'try': k})
        try:
            show(oled or LABELS.get(name, name.upper()))
            check(needs)
            arm_if(motion)
            if motion:
                WHERE['at'] = 'away'
            fn()
            stop()
            return
        except Abort as e:
            stop()
            try:
                arm_if(False)
            except Abort:
                ARMED['on'] = False
            show('RETRY %s' % (oled or LABELS.get(name, name.upper())))
            log('STEP %s try %d failed: %s' % (name, k, e))
            time.sleep(2.0)   # USB resets only when a device fails the check at the start of the next try
    raise Abort('step %s failed %d times' % (name, tries))


# ================= the mission, as retryable steps =================
done = set()
answer = {}


def once(tag, fn):
    """Moves that must not repeat on a retry (a backing or a 90 deg turn)."""
    if tag not in done:
        fn()
        done.add(tag)


def wake():
    call(voice_enable, SetBool.Request(data=True))
    log('LISTENING for "Nontok" (up to %.0f s)' % WAKE_WAIT_S)
    w0, end, checked = st['wake'], time.time() + WAKE_WAIT_S, time.time()
    while st['wake'] == w0:
        if time.time() > end:
            raise Abort('no wake word in %.0f s' % WAKE_WAIT_S)
        if time.time() - checked > 30:   # keep checking the hardware while waiting
            check(())
            call(voice_enable, SetBool.Request(data=True))
            checked = time.time()
        time.sleep(0.1)
    call(voice_enable, SetBool.Request(data=False))
    log('WAKE heard')


def leave_wall(prefix, out_to, turns):
    r = st['range']
    once(prefix + 'back', lambda: drive(-(out_to - r) if r is not None and r < out_to else -0.30))
    for i in range(turns):
        once(prefix + 'turn%d' % i, lambda: turn(90.0))


def approach(prefix, ids, dist=0.90):
    if not tags_seen(ids):
        find_tags(ids, max_deg=40)
    once(prefix + 'drive', lambda: drive(dist, stop_front=0.60, fast=True))
    ensure_far(ids)


PAIR_STANDOFF = {(12, 13): 1.05, (16, 17): 1.05,   # boxes stand ~0.4 m out from these walls; others 0.75 m
                 (10, 11): 0.95}                   # electrical box ~0.22 m out: 0.75 left the front 0.31 m from it
PAIR_STANDOFF_DEFAULT = 0.75
PAIR_GOTO_MIN = 0.12      # sideways from in front of the pair beyond which goto_pair_front moves at all


def goto_pair_front(ids):
    """One diagonal move to straight out in front of a tag pair, facing it: turn toward the point standoff m out
    from the pair along its wall normal, drive there at travel speed, turn to face the pair. 25 Nov: the room
    sketch put the electrical tags ~0.4 m off, the fixed route arrived 0.44 m to the side and the centring
    strafes (slow, slipping) took 40 s; this aims at what the cameras see instead of the map."""
    g = measure(ids)
    if g is None:
        return False
    skew, mx, my = g
    n = math.radians(skew) + math.pi                    # out of the wall, in the robot frame
    standoff = PAIR_STANDOFF.get(tuple(sorted(ids)), PAIR_STANDOFF_DEFAULT)
    tx, ty = mx + standoff * math.cos(n), my + standoff * math.sin(n)
    sideways = -mx * math.cos(n + math.pi / 2) - my * math.sin(n + math.pi / 2)
    if abs(sideways) <= PAIR_GOTO_MIN:
        return False                                     # about in front already: square_and_center finishes
    bearing = math.atan2(ty, tx)
    log('goto front of %s: %.2f m to the side, %.2f m ahead - turn %+.0f deg, drive %.2f m, turn %+.0f deg' % (
        ids, sideways, mx, math.degrees(bearing), math.hypot(tx, ty), math.degrees(wrap(math.radians(skew) - bearing))))
    turn(math.degrees(bearing), limit=20.0)
    drive(math.hypot(tx, ty), stop_front=0.35, limit=30.0, fast=True)
    turn(math.degrees(wrap(math.radians(skew) - bearing)), limit=20.0)
    return True


def ensure_far(ids):
    if not tags_seen(ids):
        find_tags(ids, max_deg=60)
    # after a diagonal move it is in front within the measurement noise (+-5 cm at 0.8 m): the park squares
    # again anyway, so squaring here only chased noise with tiny strafes (25 Nov, round 3: ~10 s a station)
    if not goto_pair_front(ids):
        square_and_center(ids)
    r = st['range']
    # 25 Nov: at 1.4-1.5 m the front ultrasonic flickers between an echo (1.449) and none (1.46): taken as an
    # echo it sent the robot to creep_to, which then saw "nothing ahead" and aborted, try after try. Beyond
    # ULTRA_TRUST_M close in on the camera distance; creep on the ultrasonic only nearer than that.
    if r is None or r >= ULTRA_TRUST_M:
        g = measure(ids)
        if g is not None and g[1] > 0.95:
            log('far: tags %.2f m ahead, ultrasonic %s - driving in on the camera distance' % (
                g[1], 'sees nothing' if r is None or r >= NO_ECHO else '%.2f m (beyond its reliable range)' % r))
            drive(min(1.60, g[1] - 0.85), stop_front=0.60, limit=30.0, fast=True)
            square_and_center(ids)
    r = st['range']
    if r is not None and 0.65 < r < ULTRA_TRUST_M:
        # stops on the 0.60 m reading (0.50 was hit on 24 Nov); travel speed, slowing for the reading
        drive(min(0.80, r - 0.60), stop_front=0.60, limit=20.0, fast=True)


def get_item():
    time.sleep(2.0)
    r = call(tray_reference, Trigger.Request())
    log('empty-tray reference: %s %s' % (r.success, r.message))
    if not r.success:
        raise Abort('no empty-tray reference')
    say('place_item')
    end, asked = time.time() + ITEM_WAIT_S, time.time()
    while True:
        t = st['tray']
        if t is not None and t.state == TrayState.OCCUPIED and t.stable:
            log('item in the tray (%.1f %% changed)' % (100 * t.occupied_fraction))
            break
        if time.time() > end:
            log('no item in %.0f s: going home' % ITEM_WAIT_S)
            answer['item'] = None
            return
        if time.time() - asked > 60:
            say('place_item')
            asked = time.time()
        time.sleep(0.2)
    for attempt in range(3):
        c = call(classify, ClassifyItem.Request(), 30.0)
        log('classify: success %s  item %r  station %r  confidence %.2f' % (c.success, c.item, c.station, c.confidence))
        if c.success:
            answer.update(item=c.item, station=c.station)
            say('item_' + c.item)
            return
        say('reposition')
        time.sleep(4.0)
    answer.update(item='unknown', station='')
    say('item_UNKNOWN_ITEM')   # 25 Nov: it gave up on an item without a word


def dump():
    say('delivered')
    if not call(set_tray, SetTray.Request(angle_deg=120.0), 8).success:
        raise Abort('tray did not tilt')
    time.sleep(1.5)
    call(set_tray, SetTray.Request(angle_deg=0.0), 8)
    done.add('dumped')



OTHER_TAGS = (2, 3, 10, 11, 12, 13, 16, 17)


def docked():
    """At the dock: the dock tags in view within 0.65 m; or right at a wall (<= 0.30 m, where the cameras are
    too close to see the dock tags) when the last place it parked was the dock (or nothing is known yet, i.e.
    just started) and no other station's tags are in view. 24 Nov review: "any wall within 0.30 m" would have
    taken the pickup for the dock after a run stopped there."""
    time.sleep(1.2)
    r = st['range']
    if r is None or r > 0.30:   # 25 Nov: "dock tags in view within 0.65 m" let a robot stopped 0.64 m out count
        return False            # as docked; parked it reads 0.16-0.24 m
    if tags_seen((0, 1)) or (WHERE['at'] in (None, 'dock') and not tags_seen(OTHER_TAGS)):
        anchor_at_dock()
        return True
    return False


def spin_search(ids, step_deg=30):
    """Lost: turn on the spot in 30 deg steps (one full turn) until the tags are in view."""
    for k in range(360 // step_deg):
        time.sleep(1.2)
        if tags_seen(ids):
            log('tags %s in view after %d deg' % (ids, k * step_deg))
            return
        turn(step_deg)
    raise Abort('tags %s not seen in a full turn' % (ids,))


STATION_PAIRS = {(2, 3): 'pickup', (12, 13): 'mechanical', (16, 17): 'tools', (10, 11): 'electrical'}
DOCK_VIEW = (-0.75, 0.0)   # map: 1.1 m out from the dock wall on its centre line - the dock tags are in view here
TAGS_FILE = '/nontok/ros2_ws/src/nontok_localization/config/tags.yaml'


def pose_from_pair(ids, geometry, tags):
    """Robot (x, y, heading) in the map from a station pair seen as (skew_deg, middle_x, middle_y) in the
    robot frame (pair_geometry) and the pair's place in tags.yaml. skew = the turn that would face the wall
    square, so the heading is (the tags' outward normal + 180 deg) - skew."""
    skew, mx, my = geometry
    left, right = tags[min(ids)], tags[max(ids)]
    cx, cy = (left['x'] + right['x']) / 2.0, (left['y'] + right['y']) / 2.0
    th = wrap(math.radians(left['yaw_deg']) + math.pi - math.radians(skew))
    return (cx - (math.cos(th) * mx - math.sin(th) * my), cy - (math.sin(th) * mx + math.cos(th) * my), th)


def locate_from_view():
    """(x, y, heading, station) from a station pair in view now, or None."""
    import yaml
    tags = yaml.safe_load(open(TAGS_FILE))['tags']
    for ids, name in STATION_PAIRS.items():
        if tags_seen(ids) and all(i in tags for i in ids):
            g = measure(ids)
            if g is not None:
                x, y, th = pose_from_pair(ids, g, tags)
                LIVE['located'] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(th, 3), 'tags': list(ids),
                                   'station': name, 't': time.time()}
                log('located on the %s tags %s (%.2f m away): x %+.2f m, y %+.2f m, heading %+.0f deg' % (
                    name, ids, math.hypot(g[1], g[2]), x, y, math.degrees(th)))
                return x, y, th, name
    return None


def walk_to_dock_view(x, y, th):
    """From a located pose: turn toward DOCK_VIEW, drive there, turn to face the dock wall (+x)."""
    tx, ty = DOCK_VIEW
    dist = math.hypot(tx - x, ty - y)
    plan('back to where the dock is in view', [(x, y), DOCK_VIEW, (0.0, 0.0)])
    if dist > 0.15:
        bearing = math.atan2(ty - y, tx - x)
        log('walking %.2f m toward the dock (turn %+.0f deg)' % (dist, math.degrees(wrap(bearing - th))))
        turn(math.degrees(wrap(bearing - th)), limit=20.0)
        drive(dist, stop_front=0.35, limit=40.0, fast=True)
        th = bearing
    turn(math.degrees(wrap(0.0 - th)), limit=20.0)


def find_dock(step_deg=30):
    """Lost: turn on the spot until the dock tags are in view. If a station's tags show up first, work out where
    the robot is from them and walk to where the dock can be seen, then look for it there (25 Nov: from the far
    side of the room the dock tags are out of sight, so spinning alone never finds them)."""
    for k in range(360 // step_deg):
        time.sleep(1.2)
        if tags_seen((0, 1)):
            log('dock tags in view after %d deg' % (k * step_deg))
            return
        located = locate_from_view()
        if located is not None:
            walk_to_dock_view(*located[:3])
            find_tags((0, 1), max_deg=60)
            return
        turn(step_deg)
    raise Abort('neither the dock tags nor any station tags seen in a full turn')


def go_home():
    """From anywhere the dock tags can be seen: back off a wall if close, find the tags, approach, dock."""
    def back_off():
        r = st['range']
        if r is not None and r < 0.45:
            drive(-(0.55 - r))
    step('home: back off', back_off, ('mega',), motion=True)
    step('home: find the dock', lambda: tags_seen((0, 1)) or find_dock(), ('mega', 'cams'), motion=True)
    step('home: approach the dock', lambda: ensure_far((0, 1)), ('mega', 'cams'), motion=True)
    step('home: park at the dock', lambda: dock_centred('dock', (0, 1), guard=0.12, contact=0.16),
         ('mega', 'cams'), motion=True)


# ---- the room (25 Nov layout, tags.yaml): facing the station wall, from left to right: tools box (16, 17,
# the old pickup spot, straight across from the dock), mechanical box (12, 13) ~1.2 m right of it, the pickup
# (2, 3) ~1 m right of mechanical. Electrical (10, 11) is on the left side wall. Routes between stations run
# along a lane ~1 m out from the station wall, clear of the boxes standing out from it.
LANE_FRONT = 1.00        # front reading (to the station wall) that puts the robot on the lane
PICKUP_ALONG = 2.29      # dock / tools line -> pickup (25 Nov: 2.205 m left the pair 8 cm further on)
PICKUP_TO_MECH = 1.09    # pickup -> mechanical (pickup y 2.29 - mechanical y 1.20)
MECH_TO_ROW = 1.20       # mechanical -> the tools / dock line (tag spacing in tags.yaml)
ELEC_PAST_ROW = 0.50     # from the tools / dock line on toward the electrical wall (front then reads ~0.6 m)
ELEC_ACROSS = 0.68       # then across to in front of the electrical tags (lane x -1.02 -> tags x -1.70)


# ---- direct routes on the anchored map pose (25 Nov, 3-4x faster): one turn and one drive per leg instead of
# the lane's L-shapes. Waypoints keep the robot's edge >= 0.14 m clear of the box fronts (mechanical and tools
# boxes stand ~0.4 m out from the station wall, x -1.75). The approach at each station still aims at the tags.
PICKUP_Y = 2.29
W_PICKUP = (-1.05, PICKUP_Y)      # in front of the pickup, ~1.1 m from its tags
W_MECH = (-1.10, 1.20)            # in front of the mechanical box
W_TOOLS = (-1.10, 0.00)           # in front of the tools box
W_ELEC = (-1.30, -0.30)           # toward electrical, its tags ahead; x -1.30 passes the tools box 0.25 m clear
W_DOCK = (-0.75, 0.00)            # in front of the dock, its tags in view


def go_to(point, name):
    """Turn toward a map point and drive there at travel speed (heading held, slows for the front sensor)."""
    p = map_pose()
    if p is None:
        raise Abort('no map pose for the way to %s' % name)
    dx, dy = point[0] - p[0], point[1] - p[1]
    dist = math.hypot(dx, dy)
    if dist < 0.08:
        return
    bearing = math.atan2(dy, dx)
    log('to %s (%.2f, %.2f) from (%.2f, %.2f): turn %+.0f deg, drive %.2f m' % (
        name, point[0], point[1], p[0], p[1], math.degrees(wrap(bearing - p[2])), dist))
    turn(math.degrees(wrap(bearing - p[2])), limit=20.0)
    drive(dist, stop_front=0.35, limit=30.0, fast=True)


def face(heading):
    p = map_pose()
    if p is not None:
        turn(math.degrees(wrap(heading - p[2])), limit=20.0)


def back_to(front, tag):
    """Straight back until the front ultrasonic would read `front` (at least 0.25 m)."""
    r = st['range']
    once(tag, lambda: drive(-max(0.25, front - r) if r is not None and r < front else -0.40, fast=True))


def mission(test_station=None):
    done.clear()
    answer.clear()
    if test_station:
        show('TEST %s' % test_station.upper())
        log('TEST: no wake word, no item, station %s' % test_station)
    else:
        step('wait for the wake word', wake, ('mega', 'cams', 'mic', 'speaker'), oled='SAY NONTOK')
    say('answer')
    LIVE.update(item='', station='')
    plan('dock -> pickup', [(0.0, 0.0), (-0.48, 0.0), W_PICKUP, (-1.81, PICKUP_Y)])
    step('leave the dock', lambda: (back_to(0.70, 'dock.back'),
                                    once('dock.go', lambda: go_to(W_PICKUP, 'the pickup')),
                                    once('dock.face', lambda: face(math.pi))), ('mega',), motion=True)
    step('approach the pickup', lambda: ensure_far((2, 3)), ('mega', 'cams'), motion=True)
    step('park at the pickup', lambda: dock_centred('pickup', (2, 3), guard=0.12, contact=0.213),
         ('mega', 'cams'), motion=True)
    if test_station:
        answer.update(item='test', station=test_station)
    else:
        step('get the item', get_item, ('tray', 'speaker'), oled='PUT ITEM IN')
    show(('%s>%s' % (answer.get('item') or 'NO ITEM', (answer.get('station') or 'HOME')[:4])).upper())
    call(tray_enable, SetBool.Request(data=False))
    station = answer.get('station')
    LIVE.update(item=answer.get('item') or '', station=station or '')
    if station == 'mechanical':
        plan('pickup -> mechanical', [(-1.80, PICKUP_Y), (-1.27, PICKUP_Y), W_MECH, (-1.60, 1.2)])
        step('pickup to mechanical', lambda: (back_to(0.75, 'mech.back'),
                                              once('mech.go', lambda: go_to(W_MECH, 'mechanical')),
                                              once('mech.face', lambda: face(math.pi))), ('mega',), motion=True)
        mechanical_tail()
        return
    if station == 'electrical':
        plan('pickup -> electrical', [(-1.80, PICKUP_Y), (-1.27, PICKUP_Y), W_ELEC, (-1.70, -0.82)])
        step('pickup to electrical', lambda: (back_to(0.75, 'elec.back'),
                                              once('elec.go', lambda: go_to(W_ELEC, 'electrical')),
                                              once('elec.face', lambda: face(-math.pi / 2))), ('mega',), motion=True)
        step('approach electrical', lambda: ensure_far((10, 11)), ('mega', 'cams'), motion=True)
        # no lip check here: the electrical box is low, the 45 deg sensors never see it (25 Nov: refused 4x);
        # front 0.225 m is the field-tested dump spot (24 Nov, robot pressing the box, dump landed)
        step('park at electrical', lambda: dock_centred('electrical', (10, 11), guard=0.12, contact=0.225),
             ('mega', 'cams'), motion=True)
        step('dump', dump, ('mega', 'speaker'), oled='DUMPING')
        say('going_home')
        plan('electrical -> dock', [(-1.70, -0.84), (-1.70, -0.59), W_DOCK, (0.0, 0.0)])
        # back only 0.25 m: backing away from electrical moves toward the tools box corner
        step('leave electrical', lambda: (once('ehome.back', lambda: drive(-0.25, fast=True)),
                                          once('ehome.go', lambda: go_to(W_DOCK, 'the dock')),
                                          once('ehome.face', lambda: face(0.0))), ('mega',), motion=True)
    elif station == 'tools':
        plan('pickup -> tools', [(-1.80, PICKUP_Y), (-1.27, PICKUP_Y), W_TOOLS, (-1.60, 0.0)])
        step('pickup to tools', lambda: (back_to(0.75, 'tools.back'),
                                         once('tools.go', lambda: go_to(W_TOOLS, 'tools')),
                                         once('tools.face', lambda: face(math.pi))), ('mega',), motion=True)
        step('approach tools', lambda: ensure_far((16, 17)), ('mega', 'cams'), motion=True)
        step('park at tools', lambda: park_box('tools', (16, 17)), ('mega', 'cams'), motion=True)
        step('dump', dump, ('mega', 'speaker'), oled='DUMPING')
        say('going_home')
        plan('tools -> dock', [(-1.60, 0.0), W_DOCK, (0.0, 0.0)])
        step('leave tools', lambda: (back_to(0.70, 'thome.back'),
                                     once('thome.go', lambda: go_to(W_DOCK, 'the dock')),
                                     once('thome.face', lambda: face(0.0))), ('mega',), motion=True)
    else:
        log('item %r -> %r: no driven route there yet, taking it home' % (answer.get('item'), station))
        say('going_home')
        plan('pickup -> dock', [(-1.80, PICKUP_Y), (-1.27, PICKUP_Y), W_DOCK, (0.0, 0.0)])
        step('leave the pickup', lambda: (back_to(0.75, 'phome.back'),
                                          once('phome.go', lambda: go_to(W_DOCK, 'the dock')),
                                          once('phome.face', lambda: face(0.0))), ('mega',), motion=True)
    step('approach the dock', lambda: ensure_far((0, 1)), ('mega', 'cams'), motion=True)
    step('park at the dock', lambda: dock_centred('dock', (0, 1), guard=0.12, contact=0.16), ('mega', 'cams'), motion=True)
    say('docked')


def safe_stop():
    stop()
    try:
        set_arm(False)
    except Exception:
        pass
    ARMED['on'] = False
    for client in (tray_enable, voice_enable):
        try:
            call(client, SetBool.Request(data=False), 3)
        except Exception:
            pass


def park_box(station, ids):
    """A return box standing out from its tag wall hides the tags from the low cameras as the robot closes in
    (24 Nov: the dock server lost them every time at 0.23 m to go), and its front face is what the ultrasonic
    reads. So: square and centre on the tags from ~0.55 m out, then push straight in on the encoders until the
    wheels stall against the box (at most the measured gap + 3 cm), then check it is really at the box."""
    r = st['range']
    if r is not None and r < 0.45:   # a retry from close in: back out to where the tags are in view
        drive(-(0.55 - r))
    square_and_center(ids)
    r = st['range']
    gap = r if r is not None and r < NO_ECHO else 0.50
    log('%s: box %.3f m ahead, pushing in' % (station, gap))
    push_in(min(0.65, gap + 0.03), limit=15.0)
    confirm_at_box(station)   # the gap came from the front sensor alone, which may have read the box edge
    WHERE['at'] = station


def park_mechanical():
    park_box('mechanical', (12, 13))


def mechanical_tail():
    step('approach mechanical', lambda: ensure_far((12, 13)), ('mega', 'cams'), motion=True)
    step('park at mechanical', park_mechanical, ('mega', 'cams'), motion=True)
    step('dump', dump, ('mega', 'speaker'), oled='DUMPING')
    say('going_home')
    plan('mechanical -> dock', [(-1.60, 1.2), (-1.20, 1.2), W_DOCK, (0.0, 0.0)])
    step('mechanical to the dock', lambda: (back_to(0.60, 'mhome.back'),
                                            once('mhome.go', lambda: go_to(W_DOCK, 'the dock')),
                                            once('mhome.face', lambda: face(0.0))), ('mega',), motion=True)
    step('approach the dock', lambda: ensure_far((0, 1)), ('mega', 'cams'), motion=True)
    step('park at the dock', lambda: dock_centred('dock', (0, 1), guard=0.12, contact=0.16), ('mega', 'cams'), motion=True)
    say('docked')
    show('DOCKED')


def live_layout():
    """The room for the live view, written once: walls, tags, stations (map metres)."""
    import json
    import yaml
    try:
        cfg = '/nontok/ros2_ws/src/'
        walls = yaml.safe_load(open(cfg + 'nontok_navigation/config/lab_layout.yaml'))
        tags = yaml.safe_load(open(TAGS_FILE))['tags']
        stations = yaml.safe_load(open(cfg + 'nontok_navigation/config/stations.yaml'))['stations']
        out = {'bounds': walls['bounds'], 'walls': [w['rect'] for w in walls.get('walls') or []],
               'tags': {str(i): [t['x'], t['y'], t['yaw_deg']] for i, t in tags.items()},
               'stations': {name: [sum(tags[i]['x'] for i in e['tag_ids']) / 2.0,
                                   sum(tags[i]['y'] for i in e['tag_ids']) / 2.0, e['tag_ids']]
                            for name, e in stations.items() if all(i in tags for i in e['tag_ids'])}}
        with open(LIVE_LAYOUT + '.tmp', 'w') as handle:
            json.dump(out, handle)
        os.replace(LIVE_LAYOUT + '.tmp', LIVE_LAYOUT)
    except Exception as e:
        log('live view: no room layout (%s)' % e)


def live_writer():
    """Five times a second: what the robot is doing and thinking, ~1-2 KB of JSON."""
    import json
    while True:
        try:
            now = time.time()
            g = st.get('gpose') if now - st.get('gpose_t', 0) < 1.0 else None
            o = (st['x'], st['y'], st['yaw']) if st.get('x') is not None and now - st.get('odom_t', 0) < 1.0 else None
            tags = {cam: ids for cam, (t, ids) in st['tags'].items() if now - t < 1.0}
            out = dict(LIVE, t=round(now, 2), up_s=round(now - T0, 1), oled=st.get('oled', ''),
                       armed=ARMED['on'], at=WHERE['at'], front=st['range'], tags=tags,
                       pose_global=None if g is None else [round(v, 3) for v in g],
                       pose_map=None if map_pose() is None else [round(v, 3) for v in map_pose()],
                       anchor=None if not ANCHOR else {'src': ANCHOR['src'], 'age_s': round(now - ANCHOR['t'], 1)},
                       pose_odom=None if o is None else [round(v, 3) for v in o])
            with open(LIVE_FILE + '.tmp', 'w') as handle:
                json.dump(out, handle)
            os.replace(LIVE_FILE + '.tmp', LIVE_FILE)
        except Exception:
            pass
        time.sleep(0.2)


live_layout()
threading.Thread(target=live_writer, name='live', daemon=True).start()


# ---- forever: listen, deliver, come home; after any failure find the dock again ----
time.sleep(1.5)
learn()
log('last known place: %s' % WHERE['at'])
runs = 0
TEST_FILE = '/tmp/nontok_test'   # holds how many unattended test loops are left (no wake word, no item)
TEST_STATIONS = ('mechanical', 'electrical', 'tools')
TEST = {'n': 0, 'fails': 0}
RESUME = '/tmp/resume_mechanical'
if os.path.exists(RESUME):   # a run stopped in front of the mechanical station with the item still aboard
    os.remove(RESUME)
    try:
        log('RESUME: finishing at the mechanical station')
        mechanical_tail()
        log('RESUME DONE: dumped at mechanical, robot docked')
    except Abort as e:
        safe_stop()
        log('RESUME STOPPED: %s' % e)
while True:
    try:
        home_tries = 0
        try:
            check(())   # hardware first: after charging the Mega and cameras were silent and "not docked" was wrong
        except Abort as e:
            log('hardware check before the docked test: %s' % e)
        while not docked() and home_tries < 3:
            home_tries += 1
            log('not docked (front %.2f m, dock tags %s): going home, try %d' % (
                st['range'] or -1, tags_seen((0, 1)), home_tries))
            try:
                go_home()
            except Abort as e:
                safe_stop()
                log('going home failed: %s' % e)
                time.sleep(5.0)
        test = None
        if os.path.exists(TEST_FILE):
            left = int(open(TEST_FILE).read().strip() or 0)
            if left > 0:
                test = TEST_STATIONS[TEST['n'] % len(TEST_STATIONS)]
            else:
                os.remove(TEST_FILE)
        t0 = time.time()
        mission(test)
        show('DOCKED')
        runs += 1
        log('MISSION %d DONE: %s, robot docked' % (runs, answer.get('item')))
        if test:
            TEST['n'] += 1
            TEST['fails'] = 0
            open(TEST_FILE, 'w').write(str(left - 1))
            log('TEST LOOP %d OK: %s in %.0f s' % (TEST['n'], test, time.time() - t0))
            time.sleep(10.0)
    except Abort as e:
        safe_stop()
        log('MISSION STOPPED (safe, disarmed): %s - recovering and listening again' % e)
        if os.path.exists(TEST_FILE):
            TEST['fails'] += 1
            TEST['n'] += 1   # next station anyway, so every station gets tested
            log('TEST LOOP FAILED (%d in a row)' % TEST['fails'])
            if TEST['fails'] >= 2:
                os.remove(TEST_FILE)
                log('TEST: two failures in a row - test loops stopped')
        time.sleep(5.0)
    except Exception as e:   # a bug must not end the service either; the supervisor restarts on a crash
        safe_stop()
        log('ERROR %r - recovering and listening again' % e)
        time.sleep(5.0)
    safe_stop()
