"""Starts a mission without the wake word:  ros2 run nontok_voice trigger"""
import time

import rclpy
from std_msgs.msg import Empty


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wake_trigger')
    publisher = node.create_publisher(Empty, 'wake', 5)
    deadline = time.monotonic() + 3.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    publisher.publish(Empty())
    rclpy.spin_once(node, timeout_sec=0.2)
    print('Wake sent' if publisher.get_subscription_count() else 'Wake sent, but nothing is listening on /wake')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
