"""Back 0.84 m, turn right 90 deg (wheel encoders), find the pickup tags, line up on them, creep to 0.20 m
on the front ultrasonic. Every move has a time limit; the robot is disarmed at the end or on any problem."""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool

from nontok_interfaces.action import DockToTags
from nontok_interfaces.msg import TagDetectionArray

T0 = time.time()


def log(s):
    print('%6.1fs  %s' % (time.time() - T0, s), flush=True)


rclpy.init()
n = Node('to_pickup')
st = {'x': None, 'y': None, 'yaw': None, 'range': None, 'tags': {}}


def on_odom(m):
    q = m.pose.pose.orientation
    st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y
    st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(Odometry, 'odometry/local', on_odom, 20)
n.create_subscription(Range, 'range/front', lambda m: st.__setitem__('range', m.range), 5)
for cam in ('front_left', 'front_right'):
    n.create_subscription(TagDetectionArray, 'tags/%s/detections' % cam,
                          lambda m, c=cam: st['tags'].__setitem__(c, (time.time(), [d.id for d in m.detections])), 5)
pub = n.create_publisher(Twist, 'cmd_vel', 10)
arm = n.create_client(SetBool, 'base/arm')
dock = ActionClient(n, DockToTags, 'dock_to_tags')
ex = rclpy.executors.MultiThreadedExecutor()
ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()


class Abort(Exception):
    pass


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
    f = arm.call_async(SetBool.Request(data=on))
    end = time.time() + 5
    while not f.done() and time.time() < end:
        time.sleep(0.05)
    ok = f.done() and f.result().success
    log('arm(%s) -> %s' % (on, ok))
    return ok


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def back(dist, limit=16.0):
    x0, y0 = st['x'], st['y']
    end = time.time() + limit
    while True:
        done = math.hypot(st['x'] - x0, st['y'] - y0)
        if done >= dist - 0.005:
            break
        if time.time() > end:
            stop()
            raise Abort('backing: only %.2f m in %.0f s' % (done, limit))
        left = dist - done
        send(vx=-max(0.04, min(0.08, 0.8 * left)))
        time.sleep(0.05)
    stop()
    log('backed %.3f m (encoders), front now %.3f m' % (math.hypot(st['x'] - x0, st['y'] - y0), st['range'] or -1))


def turn(deg, limit=12.0):
    yaw0 = st['yaw']
    target = math.radians(deg)
    end = time.time() + limit
    while True:
        turned = wrap(st['yaw'] - yaw0)
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
    log('turned %.1f deg (encoders)' % math.degrees(wrap(st['yaw'] - yaw0)))


def pickup_tags_seen():
    now = time.time()
    seen = {c: ids for c, (t, ids) in st['tags'].items() if now - t < 1.0}
    return {c: ids for c, ids in seen.items() if 2 in ids or 3 in ids}, seen


def creep_to(front, limit=15.0):
    end = time.time() + limit
    while True:
        r = st['range']
        if r is not None and r <= front + 0.003:
            break
        if r is not None and r < 0.15:
            stop()
            raise Abort('front %.3f m < 0.15 m' % r)
        if time.time() > end:
            stop()
            raise Abort('creep: front still %.3f m after %.0f s' % (r or -1, limit))
        left = (r - front) if r is not None else 0.1
        send(vx=max(0.025, min(0.05, 0.5 * left)))
        time.sleep(0.05)
    stop()
    time.sleep(0.5)
    log('stopped: front ultrasonic %.3f m' % st['range'])


time.sleep(1.5)
log('start: front %.3f m, odometry yaw %.1f deg' % (st['range'] or -1, math.degrees(st['yaw'] or 0)))
try:
    if not set_arm(True):
        raise Abort('arm failed')
    back(0.84)
    turn(-90.0)
    time.sleep(1.5)
    found, seen = pickup_tags_seen()
    log('tags in view: %s' % seen)
    if not found:
        raise Abort('pickup tags 2 / 3 not in view after the turn')
    dock.wait_for_server(timeout_sec=5)
    last = [0.0]

    def fb(m):
        f = m.feedback
        if time.time() - last[0] > 0.7:
            last[0] = time.time()
            log('dock %-8s lateral %+.3f m  distance %+.3f m  yaw %+.1f deg  front %.3f m' % (
                f.phase, f.lateral_error_m, f.distance_error_m, math.degrees(f.yaw_error_rad), st['range'] or -1))

    gf = dock.send_goal_async(DockToTags.Goal(station='pickup', require_charge=False), feedback_callback=fb)
    end = time.time() + 5
    while not gf.done() and time.time() < end:
        time.sleep(0.05)
    gh = gf.result() if gf.done() else None
    if not gh or not gh.accepted:
        raise Abort('dock goal rejected')
    rf = gh.get_result_async()
    end = time.time() + 40
    while not rf.done() and time.time() < end:
        if st['range'] is not None and 0 < st['range'] < 0.15:
            gh.cancel_goal_async()
            raise Abort('front %.3f m < 0.15 m during the line-up' % st['range'])
        time.sleep(0.05)
    if not rf.done():
        gh.cancel_goal_async()
        raise Abort('line-up did not finish in 40 s')
    r = rf.result().result
    log('line-up: success %s  lateral %+.3f m  distance %+.3f m  yaw %+.1f deg  %r' % (
        r.success, r.lateral_error_m, r.distance_error_m, math.degrees(r.yaw_error_rad), r.message))
    if not r.success:
        raise Abort('line-up failed: %s' % r.message)
    if not set_arm(True):   # the dock server disarms nothing, but make sure
        raise Abort('arm failed')
    creep_to(0.20)
except Abort as e:
    log('STOPPED: %s' % e)
finally:
    stop()
    set_arm(False)
    log('end: front %.3f m' % (st['range'] or -1))
