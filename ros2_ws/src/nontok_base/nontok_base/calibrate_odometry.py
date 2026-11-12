"""Odometry calibration and the CP2 square test (PLAN.md sections 8 and 13.5).

    ros2 run nontok_base calibrate_odometry straight   # wheel diameter
    ros2 run nontok_base calibrate_odometry strafe     # sideways scale
    ros2 run nontok_base calibrate_odometry spin       # gyro scale
    ros2 run nontok_base calibrate_odometry square     # closing error of a 1 m square

Needs base_bridge running, the robot on the floor with clear space, and a tape measure.
The robot is driven on raw wheel odometry (/wheel/odom); you then type what it really did.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool


class Driver(Node):
    def __init__(self):
        super().__init__('calibrate_odometry')
        self.pose = None
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 20)
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_client = self.create_client(SetBool, 'base/arm')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     2.0 * math.atan2(q.z, q.w))

    def arm(self, on: bool) -> None:
        self.arm_client.wait_for_service(5.0)
        future = self.arm_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def wait_pose(self):
        while self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose

    def drive(self, vx=0.0, vy=0.0, wz=0.0, distance=0.0, angle=0.0, timeout=60.0):
        """Drive at a constant velocity until the odometry says the distance or angle is done."""
        x0, y0, yaw0 = self.wait_pose()
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            x, y, yaw = self.pose
            if distance and math.hypot(x - x0, y - y0) >= distance:
                break
            if angle and abs(yaw - yaw0) >= angle:
                break
            self.cmd.publish(twist)
        self.cmd.publish(Twist())
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        x, y, yaw = self.pose
        return math.hypot(x - x0, y - y0), yaw - yaw0


def ask(prompt: str) -> float:
    return float(input(prompt))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('mode', choices=['straight', 'strafe', 'spin', 'square'])
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Driver()
    node.arm(True)
    try:
        if args.mode == 'straight':
            reported, _ = node.drive(vx=0.15, distance=2.0)
            real = ask('Odometry says %.3f m. Taped distance in metres: ' % reported)
            print('Multiply wheel_diameter_m (or scale_x) by %.4f' % (real / reported))
        elif args.mode == 'strafe':
            reported, _ = node.drive(vy=0.08, distance=1.0)
            real = ask('Odometry says %.3f m sideways. Taped distance in metres: ' % reported)
            print('Multiply scale_y by %.4f' % (real / reported))
        elif args.mode == 'spin':
            turns = 10
            _, reported = node.drive(wz=0.6, angle=turns * 2 * math.pi, timeout=180.0)
            extra = ask('Gyro says %.1f deg. Real angle past %d full turns, in degrees (+ = further): '
                        % (math.degrees(reported), turns))
            real = turns * 2 * math.pi + math.radians(extra)
            print('Multiply gyro_scale by %.4f' % (real / reported))
        else:
            x0, y0, yaw0 = node.wait_pose()
            for _ in range(4):  # nose-first square, left turns
                node.drive(vx=0.15, distance=1.0)
                node.drive(wz=0.5, angle=math.pi / 2)
            x, y, yaw = node.pose
            print('Odometry closing error: %.0f mm, %.1f deg' % (
                1000 * math.hypot(x - x0, y - y0), math.degrees(yaw - yaw0) - 360.0))
            print('Now measure where the robot really is: the CP2 limit is 100 mm and 5 deg.')
    finally:
        node.cmd.publish(Twist())
        node.arm(False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
