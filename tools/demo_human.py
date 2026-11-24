#!/usr/bin/env python3
"""Runs INSIDE the simulation container for tools/demo_voice.py: the link between the Mac's
microphone and speakers and the simulated robot.

    stdin   one line per call: publishes /wake ("Nontok!")
    stdout  STATE <state> <detail>   on every change of the mission state
            SHOW <text>              on every change of the robot's display

It also plays the person at the pickup point: puts an item in the tray when asked and takes it
out when the tray tips, mission after mission.
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from nontok_interfaces.msg import MissionState


def main() -> None:
    rclpy.init()
    node = Node('demo_human')
    wake = node.create_publisher(Empty, 'wake', 5)
    item = node.create_publisher(Bool, 'tray/sim_occupied', 5)
    last = {'state': None, 'show': None}

    def on_state(msg: MissionState) -> None:
        if (msg.state, msg.detail) == last['state']:
            return
        changed = last['state'] is None or msg.state != last['state'][0]
        last['state'] = (msg.state, msg.detail)
        print('STATE %s %s' % (msg.state, msg.detail), flush=True)
        if changed and msg.state == 'S3_WAIT_ITEM':
            item.publish(Bool(data=True))
        elif changed and msg.state == 'S7_DUMP':
            item.publish(Bool(data=False))

    def on_show(msg: String) -> None:
        if msg.data != last['show']:
            last['show'] = msg.data
            print('SHOW %s' % msg.data, flush=True)

    node.create_subscription(MissionState, 'mission/state', on_state, 10)
    node.create_subscription(String, 'base/oled', on_show, 10)

    def calls() -> None:
        for _ in sys.stdin:
            wake.publish(Empty())
        rclpy.try_shutdown()

    threading.Thread(target=calls, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
