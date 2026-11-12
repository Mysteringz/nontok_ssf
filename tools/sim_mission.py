#!/usr/bin/env python3
"""Play the human in a simulated mission and report whether the robot completes it.

Start the simulated robot first:
    ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=resistor
then run this. It says "Nontok", puts an item in the tray when the robot asks for one, takes
it out when the tray tips, and waits for the robot to be back on the dock.
Exit code 0 = mission completed, 1 = fault or timeout.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from nontok_interfaces.msg import MissionState


class Human(Node):
    def __init__(self):
        super().__init__('sim_human')
        self.state, self.detail = None, ''
        self.history = []
        self.started = time.monotonic()
        self.wake = self.create_publisher(Empty, 'wake', 5)
        self.item = self.create_publisher(Bool, 'tray/sim_occupied', 5)
        self.create_subscription(MissionState, 'mission/state', self.on_state, 10)

    def on_state(self, msg: MissionState) -> None:
        if msg.state == self.state:
            return
        self.state, self.detail = msg.state, msg.detail
        self.history.append(msg.state)
        print('%7.1f s  %-18s %s' % (time.monotonic() - self.started, msg.state, msg.detail), flush=True)
        if msg.state == 'S3_WAIT_ITEM':
            self.item.publish(Bool(data=True))
        elif msg.state == 'S7_DUMP':
            self.item.publish(Bool(data=False))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--start-wait', type=float, default=180.0, help='time allowed to reach S0_DOCKED')
    args = parser.parse_args()
    rclpy.init()
    node = Human()
    deadline = time.monotonic() + args.timeout
    woken_at = None
    result = 1
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if woken_at is None:
            if node.state == 'S0_DOCKED':
                # Repeat until the robot leaves the dock: it ignores the word until it is ready.
                node.wake.publish(Empty())
                time.sleep(1.0)
            elif node.state not in (None, 'S0_DOCKED'):
                woken_at = now
            elif now - node.started > args.start_wait:
                print('The mission never reported S0_DOCKED')
                break
            continue
        if node.state == 'FAULT':
            print('FAULT: %s' % node.detail)
            break
        if node.state == 'S0_DOCKED' and 'S7_DUMP' in node.history:
            print('Mission completed in %.0f s' % (now - woken_at))
            result = 0
            break
    else:
        print('Timed out in state %s' % node.state)
    node.destroy_node()
    rclpy.try_shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
