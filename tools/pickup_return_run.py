"""Run in the container (docker exec nontok-dumptest ... python3 /nontok/tools/pickup_return_run.py) with the
base bridge, localization, dock_server and tray_node running. 24 Nov test run, not part of the mission node.

At the pickup: empty-tray reference, wait for an item, classify it; if it belongs at the electrical depot,
back 0.10 m, turn left 90 deg (wheel encoders), dock to the electrical tags (front contact) and dump at 120 deg.
Any problem stops and disarms."""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool, Trigger

from nontok_interfaces.action import DockToTags
from nontok_interfaces.msg import TagDetectionArray, TrayState
from nontok_interfaces.srv import ClassifyItem, SetTray

T0 = time.time()
REFERENCE_AFTER_S = 20.0
ITEM_WAIT_S = 180.0


def log(s):
    print('%6.1fs  %s' % (time.time() - T0, s), flush=True)


rclpy.init()
n = Node('return_item')
st = {'x': None, 'y': None, 'yaw': None, 'range': None, 'tags': {}, 'tray': None}


def on_odom(m):
    q = m.pose.pose.orientation
    st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y
    st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(Odometry, 'odometry/local', on_odom, 20)
n.create_subscription(Range, 'range/front', lambda m: st.__setitem__('range', m.range), 5)
n.create_subscription(TrayState, 'tray/state', lambda m: st.__setitem__('tray', m), 5)
for cam in ('front_left', 'front_right'):
    n.create_subscription(TagDetectionArray, 'tags/%s/detections' % cam,
                          lambda m, c=cam: st['tags'].__setitem__(c, (time.time(), [d.id for d in m.detections])), 5)
pub = n.create_publisher(Twist, 'cmd_vel', 10)
arm = n.create_client(SetBool, 'base/arm')
tray_enable = n.create_client(SetBool, 'tray/enable')
tray_reference = n.create_client(Trigger, 'tray/capture_reference')
classify = n.create_client(ClassifyItem, 'tray/classify')
set_tray = n.create_client(SetTray, 'base/set_tray')
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


def set_arm(on):
    ok = call(arm, SetBool.Request(data=on), 5).success
    log('arm(%s) -> %s' % (on, ok))
    return ok


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def back(dist, limit=10.0):
    x0, y0 = st['x'], st['y']
    end = time.time() + limit
    while True:
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        if done >= dist - 0.005:
            break
        if time.time() > end:
            stop()
            raise Abort('backing: only %.2f m in %.0f s' % (done, limit))
        send(vx=-max(0.04, min(0.08, 0.8 * (dist - done))))
        time.sleep(0.05)
    stop()
    log('backed %.3f m (encoders), front %.3f m' % (math.hypot(st['x'] - x0, st['y'] - y0), st['range'] or -1))


def turn(deg, limit=12.0):
    yaw0, target = st['yaw'], math.radians(deg)
    end = time.time() + limit
    while True:
        err = target - wrap(st['yaw'] - yaw0)
        if abs(err) < math.radians(1.5):
            break
        if time.time() > end:
            stop()
            raise Abort('turn: %.1f of %.0f deg in %.0f s' % (math.degrees(wrap(st['yaw'] - yaw0)), deg, limit))
        w = max(0.12, min(0.35, 1.2 * abs(err)))
        send(wz=w if err > 0 else -w)
        time.sleep(0.05)
    stop()
    log('turned %.1f deg (encoders)' % math.degrees(wrap(st['yaw'] - yaw0)))


def creep_to(front, limit=20.0):
    end = time.time() + limit
    while True:
        r = st['range']
        if r is not None and r <= front + 0.005:
            break
        if r is not None and r < 0.15:
            stop()
            raise Abort('front %.3f m < 0.15 m' % r)
        if time.time() > end:
            stop()
            raise Abort('creep: front still %.3f m after %.0f s' % (r or -1, limit))
        send(vx=0.06)
        time.sleep(0.05)
    stop()
    log('crept in: front %.3f m' % (st['range'] or -1))


def tags_seen(ids):
    now = time.time()
    return {c: v for c, (t, v) in st['tags'].items() if now - t < 1.0 and any(i in v for i in ids)}


armed = False
try:
    # ---- the item ----
    call(tray_enable, SetBool.Request(data=True))
    log('tray camera on: KEEP THE TRAY EMPTY; reference in %.0f s' % REFERENCE_AFTER_S)
    time.sleep(REFERENCE_AFTER_S)
    r = call(tray_reference, Trigger.Request())
    log('empty-tray reference: %s %s' % (r.success, r.message))
    if not r.success:
        raise Abort('no empty-tray reference')
    log('PUT THE ITEM IN THE TRAY NOW (waiting up to %.0f s)' % ITEM_WAIT_S)
    end = time.time() + ITEM_WAIT_S
    last = 0.0
    while True:
        t = st['tray']
        if t is not None and t.state == TrayState.OCCUPIED and t.stable:
            log('item in the tray: %.1f %% of the tray changed, stable' % (100 * t.occupied_fraction))
            break
        if time.time() > end:
            raise Abort('no item placed in %.0f s' % ITEM_WAIT_S)
        if t is not None and time.time() - last > 5:
            last = time.time()
            log('  tray: state %d, %.1f %% changed, stable %s' % (t.state, 100 * t.occupied_fraction, t.stable))
        time.sleep(0.2)
    answer = None
    for attempt in range(3):
        c = call(classify, ClassifyItem.Request(), 30.0)
        log('classify: success %s  item %r  station %r  confidence %.2f' % (c.success, c.item, c.station, c.confidence))
        if c.success:
            answer = c
            break
        time.sleep(2.0)
    call(tray_enable, SetBool.Request(data=False))
    if answer is None:
        raise Abort('item not recognised')
    if answer.station != 'electrical':
        raise Abort('item %r goes to %r: only the route to the electrical depot is known here' % (answer.item, answer.station))
    # ---- to the electrical depot ----
    armed = set_arm(True)
    if not armed:
        raise Abort('arm failed')
    back(0.10)
    turn(90.0)
    creep_to(0.60)   # 24 Nov: from ~1.2 m the tags were too small to trust; from 0.6 m they work
    time.sleep(1.5)
    seen = tags_seen((10, 11))
    log('electrical tags in view: %s' % seen)
    if not seen:
        raise Abort('electrical tags 10 / 11 not in view after the turn')
    dock.wait_for_server(timeout_sec=5)
    lastfb = [0.0]

    def fb(m):
        f = m.feedback
        if time.time() - lastfb[0] > 0.7:
            lastfb[0] = time.time()
            log('dock %-8s lateral %+.3f m  distance %+.3f m  yaw %+.1f deg  front %.3f m' % (
                f.phase, f.lateral_error_m, f.distance_error_m, math.degrees(f.yaw_error_rad), st['range'] or -1))

    gf = dock.send_goal_async(DockToTags.Goal(station='electrical', require_charge=False), feedback_callback=fb)
    end = time.time() + 5
    while not gf.done() and time.time() < end:
        time.sleep(0.05)
    gh = gf.result() if gf.done() else None
    if not gh or not gh.accepted:
        raise Abort('dock goal rejected')
    rf = gh.get_result_async()
    end = time.time() + 45
    while not rf.done() and time.time() < end:
        if st['range'] is not None and 0 < st['range'] < 0.08:
            gh.cancel_goal_async()
            raise Abort('front %.3f m < 0.08 m' % st['range'])
        time.sleep(0.05)
    if not rf.done():
        gh.cancel_goal_async()
        raise Abort('docking did not finish in 45 s')
    res = rf.result().result
    log('dock: success %s  lateral %+.3f m  distance %+.3f m  yaw %+.1f deg  %r' % (
        res.success, res.lateral_error_m, res.distance_error_m, math.degrees(res.yaw_error_rad), res.message))
    if not res.success:
        raise Abort('docking failed: %s' % res.message)
    set_arm(False)
    armed = False
    log('dump: tray 120, hold 1.5 s, lower')
    log('set_tray(120) -> %s' % call(set_tray, SetTray.Request(angle_deg=120.0), 8).success)
    time.sleep(1.5)
    log('set_tray(0) -> %s' % call(set_tray, SetTray.Request(angle_deg=0.0), 8).success)
    log('DONE: %s returned to the electrical depot' % answer.item)
except Abort as e:
    log('STOPPED: %s' % e)
finally:
    stop()
    if armed:
        set_arm(False)
    try:
        call(tray_enable, SetBool.Request(data=False), 3)
    except Exception:
        pass
