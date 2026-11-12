"""ROS 2 node: runs the mission state machine at 10 Hz (PLAN.md section 13.12).

    ros2 service call /mission/stop std_srvs/srv/Trigger     stop now and go to FAULT
    ros2 service call /mission/reset std_srvs/srv/Trigger    robot is back on the dock: leave FAULT
"""
import datetime
import json
import os
import threading
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.executors import MultiThreadedExecutor

from nontok_interfaces.msg import MissionState

from .robot_api import MissionIo, RosRobotApi
from .state_machine import Mission, MissionConfig


def main(args=None):
    rclpy.init(args=args)
    io = MissionIo()
    cfg = MissionConfig()
    for name, default in vars(cfg).items():
        setattr(cfg, name, io.declare_parameter(name, default).value)
    charge_threshold = io.declare_parameter('charge_threshold_v', 4.0).value
    wait_for_nav2 = io.declare_parameter('wait_for_nav2', True).value
    log_dir = os.path.expanduser(io.declare_parameter('log_dir', '~/nontok_logs').value)
    os.makedirs(log_dir, exist_ok=True)

    def log(event: dict) -> None:
        event = dict(event, time=datetime.datetime.now().isoformat(timespec='milliseconds'))
        path = os.path.join(log_dir, datetime.date.today().isoformat() + '.jsonl')
        with open(path, 'a') as handle:
            handle.write(json.dumps(event) + '\n')
        io.get_logger().info(json.dumps(event))

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(io)
    threading.Thread(target=executor.spin, daemon=True).start()

    navigator = BasicNavigator()     # its own node; it spins itself when polled
    if wait_for_nav2:
        # localizer='bt_navigator': there is no AMCL to wait for; localisation is tag-based.
        navigator.waitUntilNav2Active(localizer='bt_navigator')
    mission = Mission(RosRobotApi(io, navigator, charge_threshold), cfg, log)
    state_pub = io.create_publisher(MissionState, 'mission/state', 5)
    log({'event': 'start'})
    try:
        while rclpy.ok():
            if io.reset_flag:
                io.reset_flag = False
                mission.reset()
            mission.tick(time.monotonic())
            msg = MissionState()
            msg.header.stamp = io.get_clock().now().to_msg()
            msg.state, msg.detail = mission.state, mission.detail
            state_pub.publish(msg)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
