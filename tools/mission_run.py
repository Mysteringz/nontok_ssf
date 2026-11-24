"""Voice-called return run (24 Nov 2026), scripted routes in the real room, no Nav2. Start docked.
"Nontok" -> dock to pickup -> empty-tray reference, ask for the item, classify -> electrical depot, dump
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

import rclpy
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
st = {'x': None, 'y': None, 'yaw': None, 'range': None, 'tags': {}, 'u': {}, 'tray': None, 'wake': 0}
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


n.create_subscription(Odometry, 'odometry/local', on_odom, 20)
n.create_subscription(Range, 'range/front', lambda m: (st.__setitem__('range', m.range), st.__setitem__('range_t', time.time())), 5)
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
    """No pose solve: centre on the tag pair while farther than 0.42 m (5 cm steps), then straight in on the
    encoders to the contact distance. Closer than ~0.4 m one tag leaves a camera's view (24 Nov run), so no
    more centring there - the dock server's blind finish works the same way."""
    log('visual park on %s to front %.3f m' % (ids, contact))
    for _ in range(12):
        r = st['range']
        if r is None or r <= 0.42:
            break
        center_on(ids, tries=3)
        drive(max(0.02, min(0.05, r - 0.42)), stop_front=0.42)
    r = st['range'] or 0.3
    if r > contact + 0.005:
        drive(min(0.30, r - contact), stop_front=contact)
    log('parked by eye: front %.3f m' % (st['range'] or -1))


def dock_centred(station, ids, guard, contact):
    r = st['range']
    if r is not None and r < 0.45:   # a retry from close in: back out to where both tags are in view
        drive(-(0.55 - r))
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
             SHARE % ('nontok_perception', 'nontok_perception', 'tray.yaml')]}
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


def check(needs):
    """Hardware check before every step: everything in ALL_HW plus what the step needs."""
    needs = set(needs) | set(ALL_HW)
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
        raise Abort('USB devices still silent: %s' % bad)
    log('HW ok: Mega (front %.2f m), cameras %s, speaker, %s, tray camera' % (
        st['range'] or -1, sorted(c for c in ('front_left', 'front_right') if cam_ok(c)), mic_status()[1]))


def step(name, fn, needs, motion=False, tries=4):
    for k in range(1, tries + 1):
        try:
            check(needs)
            if motion:
                set_arm(True)
            fn()
            if motion:
                stop()
                set_arm(False)
            return
        except Abort as e:
            stop()
            try:
                set_arm(False)
            except Abort:
                pass
            log('STEP %s try %d failed: %s' % (name, k, e))
            if k >= 2:   # devices looked fine but the step still fails: reset the cameras it relies on anyway
                for cam in ('front_left', 'front_right'):
                    if 'cams' in needs:
                        usb_reset(cam)
                        restart_node(cam)
            time.sleep(2.0)
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


def approach(prefix, ids):
    if not tags_seen(ids):
        find_tags(ids, max_deg=40)
    once(prefix + 'drive', lambda: drive(0.90, stop_front=0.60))
    ensure_far(ids)


def ensure_far(ids):
    if not tags_seen(ids):
        find_tags(ids, max_deg=60)
    center_on(ids)
    if st['range'] is not None and 0.65 < st['range'] < NO_ECHO:
        creep_to(0.60, max_dist=0.80, limit=30.0)   # stops on the 0.60 m reading; 0.50 was hit on 24 Nov


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


def dump():
    say('delivered')
    if not call(set_tray, SetTray.Request(angle_deg=120.0), 8).success:
        raise Abort('tray did not tilt')
    time.sleep(1.5)
    call(set_tray, SetTray.Request(angle_deg=0.0), 8)
    done.add('dumped')



def docked():
    time.sleep(1.2)
    r = st['range']
    return r is not None and r <= 0.30 and bool(tags_seen((0, 1)))


def spin_search(ids, step_deg=30):
    """Lost: turn on the spot in 30 deg steps (one full turn) until the tags are in view."""
    for k in range(360 // step_deg):
        time.sleep(1.2)
        if tags_seen(ids):
            log('tags %s in view after %d deg' % (ids, k * step_deg))
            return
        turn(step_deg)
    raise Abort('tags %s not seen in a full turn' % (ids,))


def go_home():
    """From anywhere the dock tags can be seen: back off a wall if close, find the tags, approach, dock."""
    def back_off():
        r = st['range']
        if r is not None and r < 0.45:
            drive(-(0.55 - r))
    step('home: back off', back_off, ('mega',), motion=True)
    step('home: find the dock', lambda: tags_seen((0, 1)) or spin_search((0, 1)), ('mega', 'cams'), motion=True)
    step('home: approach the dock', lambda: ensure_far((0, 1)), ('mega', 'cams'), motion=True)
    step('home: park at the dock', lambda: dock_centred('dock', (0, 1), guard=0.12, contact=0.16),
         ('mega', 'cams'), motion=True)


def mission():
    done.clear()
    answer.clear()
    step('wait for the wake word', wake, ('mega', 'cams', 'mic', 'speaker'))
    say('answer')
    step('leave the dock', lambda: leave_wall('dock.', 0.70, 2), ('mega',), motion=True)
    step('approach the pickup', lambda: approach('pickup.', (2, 3)), ('mega', 'cams'), motion=True)
    step('park at the pickup', lambda: dock_centred('pickup', (2, 3), guard=0.12, contact=0.213),
         ('mega', 'cams'), motion=True)
    step('get the item', get_item, ('tray', 'speaker'))
    call(tray_enable, SetBool.Request(data=False))
    if answer.get('station') == 'electrical':
        step('leave the pickup', lambda: (once('pickup.back', lambda: drive(-0.10)),
                                          once('pickup.turn', lambda: turn(90.0))), ('mega',), motion=True)
        step('approach electrical', lambda: ensure_far((10, 11)), ('mega', 'cams'), motion=True)
        step('park at electrical', lambda: dock_centred('electrical', (10, 11), guard=0.12, contact=0.225),
             ('mega', 'cams'), motion=True)
        step('dump', dump, ('mega', 'speaker'))
        say('going_home')
        step('leave electrical', lambda: (once('elec.back9', lambda: drive(-0.90)),
                                          once('elec.turn', lambda: turn(90.0))), ('mega',), motion=True)
    else:
        log('item %r -> %r: no driven route there yet, taking it home' % (answer.get('item'), answer.get('station')))
        say('going_home')
        step('leave the pickup', lambda: leave_wall('pick.', 0.70, 2), ('mega',), motion=True)
    step('approach the dock', lambda: approach('home.', (0, 1)), ('mega', 'cams'), motion=True)
    step('park at the dock', lambda: dock_centred('dock', (0, 1), guard=0.12, contact=0.16), ('mega', 'cams'), motion=True)
    say('docked')


def safe_stop():
    stop()
    try:
        set_arm(False)
    except Exception:
        pass
    for client in (tray_enable, voice_enable):
        try:
            call(client, SetBool.Request(data=False), 3)
        except Exception:
            pass


# ---- forever: listen, deliver, come home; after any failure find the dock again ----
time.sleep(1.5)
learn()
runs = 0
while True:
    try:
        home_tries = 0
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
        mission()
        runs += 1
        log('MISSION %d DONE: %s, robot docked' % (runs, answer.get('item')))
    except Abort as e:
        safe_stop()
        log('MISSION STOPPED (safe, disarmed): %s - recovering and listening again' % e)
        time.sleep(5.0)
    except Exception as e:   # a bug must not end the service either; the supervisor restarts on a crash
        safe_stop()
        log('ERROR %r - recovering and listening again' % e)
        time.sleep(5.0)
    safe_stop()
