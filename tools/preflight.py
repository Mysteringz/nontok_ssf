#!/usr/bin/env python3
"""Start-of-day check (PLAN.md section 13.13). Run on the Jetson with the robot on its dock
and nontok.launch.py (or base + localization) running:

    python3 tools/preflight.py

Prints PASS or FAIL per line; exit status 1 if anything failed.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

from nontok_interfaces.msg import BaseStatus, LocalizationStatus, TagDetectionArray

RANGES = ['left45', 'right45', 'front', 'left', 'right']
CAMERAS = ['front_left', 'front_right', 'rear']
MIN_BATTERY_V = 7.0


class Preflight(Node):
    def __init__(self):
        super().__init__('preflight')
        self.status = None
        self.odom_count = 0
        self.ranges = {}
        self.tags = {}
        self.localization = None
        self.pose = None
        self.create_subscription(BaseStatus, 'base/status', lambda m: setattr(self, 'status', m), 5)
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 20)
        self.create_subscription(Odometry, 'odometry/global', lambda m: setattr(self, 'pose', m.pose.pose), 5)
        self.create_subscription(LocalizationStatus, 'localization/status',
                                 lambda m: setattr(self, 'localization', m), 5)
        for name in RANGES:
            self.create_subscription(Range, 'range/' + name, lambda m, n=name: self.ranges.__setitem__(n, m.range), 5)
        for name in CAMERAS:
            self.create_subscription(TagDetectionArray, 'tags/%s/detections' % name,
                                     lambda m, n=name: self.tags.__setitem__(n, max(self.tags.get(n, 0), len(m.detections))), 5)

    def on_odom(self, msg):
        self.odom_count += 1


def main():
    rclpy.init()
    node = Preflight()
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    checks = []
    checks.append(('Mega link: odometry arriving', node.odom_count >= 60, '%d messages in 5 s' % node.odom_count))
    status = node.status
    checks.append(('Mega status received', status is not None, ''))
    if status is not None:
        checks.append(('No firmware fault (gyro ready)', status.fault == '', status.fault or 'none'))
        if status.battery_v > 0.1:
            checks.append(('Motor battery', status.battery_v >= MIN_BATTERY_V, '%.1f V' % status.battery_v))
        else:
            print('SKIP  Motor battery: voltage sensing not fitted')
    for name in RANGES:
        checks.append(('Ultrasonic %s' % name, name in node.ranges, '%.2f m' % node.ranges[name] if name in node.ranges else 'silent'))
    for name in CAMERAS:
        checks.append(('Camera %s alive' % name, name in node.tags, ''))
    checks.append(('Dock tags visible from the dock', sum(node.tags.get(n, 0) for n in ('front_left', 'front_right')) >= 2, ''))
    checks.append(('Localisation healthy', node.localization is not None and node.localization.healthy, ''))
    if node.pose is not None:
        offset = math.hypot(node.pose.position.x, node.pose.position.y)
        checks.append(('Pose within 50 mm of the dock origin', offset <= 0.05, '%.0f mm' % (offset * 1000)))
    else:
        checks.append(('Global pose available', False, ''))

    failed = 0
    for name, ok, note in checks:
        print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, (' (%s)' % note) if note else ''))
        failed += not ok
    print('\n%s' % ('All checks passed.' if not failed else '%d check(s) failed.' % failed))
    print('By hand: tags present and flat, lights on, tray empty, cut-off switch within reach.')
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
