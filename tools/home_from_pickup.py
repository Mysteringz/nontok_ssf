"""From in front of the pickup back to the dock (24 Nov), with a parking beep while the wheels move.
Line up on the pickup tags (search if needed), back 0.50 m, turn left 90 deg twice, find the dock tags (search
+-40 deg if needed), forward at most 0.90 m, creep at most 0.40 m while the ultrasonic sees the wall, dock.
Every move is capped; anything unexpected stops, disarms and says so."""
import math
import os
import struct
import subprocess
import threading
import time
import wave

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool

from nontok_interfaces.action import DockToTags
from nontok_interfaces.msg import TagDetectionArray

SOUNDS = '/nontok/ros2_ws/src/nontok_voice/sounds/'
SPEAKER = 'plughw:CARD=Audio,DEV=0'
BEEP = '/tmp/parking_beep.wav'
BEEP_WHILE_MOVING = False   # 24 Nov: the speaker shares the USB hub with the cameras; they dropped out while it beeped
NO_ECHO = 1.45          # the front ultrasonic reports 1.5 m for "nothing in range"
T0 = time.time()


def log(s):
    print('%6.1fs  %s' % (time.time() - T0, s), flush=True)


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


def say(name):
    path = SOUNDS + name + '.wav'
    if os.path.exists(path):
        log('SAY %s' % name)
        subprocess.run(['aplay', '-q', '-D', SPEAKER, path], timeout=15)


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
st = {'x': None, 'y': None, 'yaw': None, 'range': None, 'tags': {}, 'u': {}}
# camera: (x, y, viewing yaw rad, fx, cx) from robot.yaml and the 24 Nov calibrations
CAMS = {'front_left': (0.12, 0.080, math.radians(-1.1), 845.0, 627.8),
        'front_right': (0.12, -0.080, math.radians(-3.4), 840.0, 598.6)}
HALF_SPACING = 0.15   # every tag pair is 30 cm apart (centre to centre); the lower id is on the left


def on_odom(m):
    q = m.pose.pose.orientation
    st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y
    st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_tags(cam, m):
    now = time.time()
    st['tags'][cam] = (now, [d.id for d in m.detections])
    st['u'][cam] = (now, {d.id: sum(d.corners[0::2]) / 4.0 for d in m.detections})


n.create_subscription(Odometry, 'odometry/local', on_odom, 20)
n.create_subscription(Range, 'range/front', lambda m: st.__setitem__('range', m.range), 5)
for cam in ('front_left', 'front_right'):
    n.create_subscription(TagDetectionArray, 'tags/%s/detections' % cam,
                          lambda m, c=cam: on_tags(c, m), 5)
pub = n.create_publisher(Twist, 'cmd_vel', 10)
arm = n.create_client(SetBool, 'base/arm')
dock = ActionClient(n, DockToTags, 'dock_to_tags')
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


def send(vx=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.angular.z = float(vx), float(wz)
    pub.publish(t)


def stop(sec=0.8):
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


def drive(dist, stop_front=None, limit=20.0):
    x0, y0 = st['x'], st['y']
    sign = 1 if dist > 0 else -1
    end = time.time() + limit
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
        send(vx=sign * max(0.04, min(0.08, 0.8 * (abs(dist) - done))))
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
        w = max(0.12, min(0.35, 1.2 * abs(err)))
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


def find_tags(ids, max_deg=40, step=10):
    """Tags in view now, else turn in steps (+10, -20, +30 ...) up to +-max_deg around here."""
    time.sleep(1.2)
    if tags_seen(ids):
        log('tags %s in view: %s' % (ids, tags_seen(ids)))
        return
    offset = 0
    for k in range(1, 2 * max_deg // step + 1):
        target = (k + 1) // 2 * step * (1 if k % 2 else -1)
        if abs(target) > max_deg:
            break
        turn(target - offset)
        offset = target
        time.sleep(1.2)
        if tags_seen(ids):
            log('tags %s found at %+d deg: %s' % (ids, offset, tags_seen(ids)))
            return
    turn(-offset)
    raise Abort('tags %s not found within +-%d deg' % (ids, max_deg))


def pair_bearing(ids):
    """Bearing (rad, + = left) from the robot centre to the middle of the tag pair, from the pixel positions,
    the camera mounts and the front range (no pose solve). None if no tag of the pair is in view."""
    left_id, right_id = min(ids), max(ids)
    d = st['range'] if st['range'] is not None and 0.2 < st['range'] < NO_ECHO else 0.6
    points = []
    now = time.time()
    for cam, (t, us) in st['u'].items():
        if now - t > 1.0 or cam not in CAMS:
            continue
        cx_, cy_, cyaw, f, c0 = CAMS[cam]
        found = {}
        for i in ids:
            if i in us:
                b = cyaw - math.atan((us[i] - c0) / f)
                found[i] = (cx_ + d * math.cos(b), cy_ + d * math.sin(b))   # wall depth ~ front range
        if left_id in found and right_id in found:
            points.append(((found[left_id][0] + found[right_id][0]) / 2, (found[left_id][1] + found[right_id][1]) / 2))
        elif left_id in found:
            points.append((found[left_id][0], found[left_id][1] - HALF_SPACING))
        elif right_id in found:
            points.append((found[right_id][0], found[right_id][1] + HALF_SPACING))
    if not points:
        return None
    x = sum(q[0] for q in points) / len(points)
    y = sum(q[1] for q in points) / len(points)
    return math.atan2(y, x)


def center_on(ids, tries=4):
    """Turn until the middle of the tag pair is straight ahead (both cameras then see both tags)."""
    for _ in range(tries):
        time.sleep(1.2)
        b = pair_bearing(ids)
        if b is None:
            find_tags(ids, max_deg=60)
            continue
        log('pair %s middle at %+.1f deg, seen %s' % (ids, math.degrees(b), tags_seen(ids)))
        if abs(math.degrees(b)) < 2.5:
            return
        turn(max(-30.0, min(30.0, math.degrees(b))))
    log('centred on %s: %s' % (ids, tags_seen(ids)))


def visual_park(ids, contact):
    """No pose solve: keep the middle of the tag pair straight ahead in the pictures and creep in 5 cm steps
    until the front ultrasonic reads `contact` (24 Nov: the pickup pair fits 8-10 px, over the solver limit)."""
    log('visual park on %s to front %.3f m' % (ids, contact))
    for _ in range(16):
        r = st['range']
        if r is not None and r <= contact + 0.005:
            break
        center_on(ids, tries=3)
        step = min(0.05, (r or 0.3) - contact)
        if step <= 0.005:
            break
        drive(step, stop_front=contact)
    log('parked by eye: front %.3f m, pair middle %s deg' % (
        st['range'] or -1, '%+.1f' % math.degrees(pair_bearing(ids)) if pair_bearing(ids) is not None else '?'))


def dock_centred(station, ids, guard, contact):
    center_on(ids)
    try:
        dock_at(station, guard)
    except Abort as e:
        if 'not seen' not in str(e):
            raise
        visual_park(ids, contact)


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
        raise Abort('docking at %s failed: %s' % (station, r.message))


armed = False
try:
    time.sleep(1.5)
    log('start: front %.3f m' % (st['range'] or -1))
    # both tag cameras alive (24 Nov: they dropped off USB once and the detectors went silent)
    time.sleep(1.5)
    stale = [c for c in ('front_left', 'front_right') if time.time() - st['tags'].get(c, (0, []))[0] > 1.0]
    if stale:
        raise Abort('camera(s) not sending: %s' % stale)
    log('cameras OK: %s' % {c: v for c, (t, v) in st['tags'].items()})
    armed = set_arm(True)
    # a known start: lined up on the pickup tags. Too close to read them (24 Nov: nose against boxes beside
    # the pickup, tag 2 cut off at the image edge): back off first, then search wider.
    time.sleep(1.2)
    if os.environ.get('FROM') == 'dock_front':   # already facing the dock wall: creep to 0.60 m and dock
        find_tags((0, 1), max_deg=20)
        if st['range'] is not None and 0.65 < st['range'] < NO_ECHO:
            creep_to(0.60, max_dist=0.50, limit=20.0)
        dock_centred('dock', (0, 1), guard=0.12, contact=0.16)
        raise StopIteration
    if not tags_seen((2, 3)):
        drive(-0.30)
    find_tags((2, 3), max_deg=60)
    center_on((2, 3))
    r = st['range']
    if r is not None and r < 0.70:   # same spot as the pickup route's start: 0.70 m out from the pickup wall
        drive(-(0.70 - r))
    say('going_home')
    turn(90.0)
    turn(90.0)
    find_tags((0, 1))
    drive(0.90, stop_front=0.60)
    if st['range'] is not None and 0.65 < st['range'] < NO_ECHO:
        creep_to(0.60, max_dist=0.50, limit=20.0)
    find_tags((0, 1), max_deg=20)
    dock_centred('dock', (0, 1), guard=0.12, contact=0.16)
    set_arm(False)
    armed = False
    say('docked')
    log('HOME: docked')
except StopIteration:
    set_arm(False)
    armed = False
    say('docked')
    log('HOME: docked')
except Abort as e:
    log('STOPPED: %s' % e)
    stop()
    if armed:
        set_arm(False)
        armed = False
    say('fault')
finally:
    stop()
    if armed:
        set_arm(False)
