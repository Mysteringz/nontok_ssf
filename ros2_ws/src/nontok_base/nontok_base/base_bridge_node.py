"""ROS 2 node: the only program that talks to the Mega (PLAN.md section 13.5).

/cmd_vel in; wheel odometry, gyro, ultrasonic ranges and status out; services for arming,
stopping, the tray and the gyro bias.
"""
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger

from nontok_interfaces.msg import BaseStatus
from nontok_interfaces.srv import SetTray

from . import protocol
from .bridge_core import BridgeCore, OdomSample
from .kinematics import MecanumParams
from .protocol import Ready, Reply, Sta


class BaseBridge(Node):
    def __init__(self):
        super().__init__('base_bridge')
        p = self.declare_parameter
        self.port = p('port', '/dev/nontok_mega').value
        self.baud = p('baud', 115200).value
        params = MecanumParams(
            wheel_diameter_m=p('wheel_diameter_m', 0.080).value,
            counts_per_rev=p('counts_per_rev', 660.0).value,
            rotation_arm_m=p('rotation_arm_m', 0.160).value,
            scale_x=p('scale_x', 1.0).value,
            scale_y=p('scale_y', 1.0).value,
        )
        self.max_range = p('max_range_m', 1.5).value
        self.core = BridgeCore(
            params,
            gyro_scale=p('gyro_scale', 1.0).value,
            cmd_timeout_s=p('cmd_timeout_s', 0.25).value,
            max_range=self.max_range,
        )
        self.var_vx = p('vx_std', 0.02).value ** 2
        self.var_vy = p('vy_std', 0.05).value ** 2
        self.var_gyro = p('gyro_std', 0.01).value ** 2
        self.range_fov = p('range_fov_rad', 0.26).value
        self.clock_offset = 0.0  # ROS time - time.monotonic(), refreshed in the reader

        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 20)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 20)
        self.range_pubs = {name: self.create_publisher(Range, 'range/' + name, 5)
                           for name in protocol.RANGE_NAMES}
        self.status_pub = self.create_publisher(BaseStatus, 'base/status', 5)
        self.wake_pub = self.create_publisher(Empty, 'wake', 5)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, 'base/oled', self.on_oled, 5)
        services = ReentrantCallbackGroup()
        self.create_service(SetBool, 'base/arm', self.on_arm, callback_group=services)
        self.create_service(Trigger, 'base/stop', self.on_stop, callback_group=services)
        self.create_service(SetTray, 'base/set_tray', self.on_set_tray, callback_group=services)
        self.create_service(Trigger, 'base/calibrate_gyro', self.on_cal, callback_group=services)

        self._write_lock = threading.Lock()
        self._reply_cond = threading.Condition()
        self._replies = []
        self._last_fault = ''
        self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
        self._running = True
        threading.Thread(target=self._reader, name='mega-reader', daemon=True).start()
        # The 20 Hz velocity stream has its own thread. As an executor timer it was held up
        # by other callbacks for more than 300 ms now and then, which trips the Mega's watchdog.
        threading.Thread(target=self._sender, name='mega-sender', daemon=True).start()
        self.create_timer(0.5, self.on_health_timer)
        self.get_logger().info('Base bridge on %s' % self.port)

    # ---- serial ----
    def _write(self, line: str) -> None:
        with self._write_lock:
            self.serial.write((line + '\n').encode('ascii'))

    def _sender(self) -> None:
        next_time = time.monotonic()
        while self._running and rclpy.ok():
            try:
                self.on_send_timer()
            except serial.SerialException as error:
                self.get_logger().error('Serial write failed: %s' % error)
            next_time = max(next_time + 0.05, time.monotonic() - 0.05)
            time.sleep(max(0.0, next_time - time.monotonic()))

    def _reader(self) -> None:
        while self._running and rclpy.ok():
            try:
                raw = self.serial.readline()
            except serial.SerialException as error:
                self.get_logger().error('Serial read failed: %s' % error)
                time.sleep(0.5)
                continue
            if not raw:
                continue
            now = time.monotonic()
            self.clock_offset = self.get_clock().now().nanoseconds * 1e-9 - now
            result = self.core.handle_line(raw.decode('ascii', errors='replace'), now)
            if isinstance(result, OdomSample):
                self._publish_odom(result)
            elif isinstance(result, list):
                for sample in result:
                    self._publish_range(sample)
            elif isinstance(result, Sta):
                self._publish_status(result)
            elif isinstance(result, Ready):
                self.get_logger().warn('Mega (re)started: %s v%d' % (result.name, result.version))
            elif isinstance(result, Reply):
                if result.kind == 'EVT' and result.arg == 'WAKE':
                    self.wake_pub.publish(Empty())
                if result.kind == 'ERR':
                    self.get_logger().warn('Mega: ERR %s' % result.arg)
                with self._reply_cond:
                    self._replies.append(result)
                    del self._replies[:-20]
                    self._reply_cond.notify_all()

    def _wait_reply(self, kind: str, arg: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._reply_cond:
            while True:
                for reply in self._replies:
                    if reply.kind == kind and reply.arg == arg:
                        self._replies.remove(reply)
                        return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._reply_cond.wait(remaining)

    def _command(self, line: str, kind: str, arg: str, timeout_s: float = 1.0) -> bool:
        with self._reply_cond:
            self._replies[:] = [r for r in self._replies if not (r.kind == kind and r.arg == arg)]
        self._write(line)
        return self._wait_reply(kind, arg, timeout_s)

    def _stamp(self, monotonic_time: float):
        return Time(nanoseconds=int((monotonic_time + self.clock_offset) * 1e9)).to_msg()

    # ---- publishers ----
    def _publish_odom(self, s: OdomSample) -> None:
        stamp = self._stamp(s.stamp)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = s.x
        odom.pose.pose.position.y = s.y
        odom.pose.pose.orientation.z = math.sin(s.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(s.yaw / 2.0)
        odom.pose.covariance[0] = odom.pose.covariance[7] = 1.0   # not fused: twist only
        odom.pose.covariance[35] = 1.0
        odom.twist.twist.linear.x = s.vx
        odom.twist.twist.linear.y = s.vy
        odom.twist.twist.angular.z = s.yaw_rate
        odom.twist.covariance[0] = self.var_vx
        odom.twist.covariance[7] = self.var_vy
        odom.twist.covariance[35] = self.var_gyro
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation_covariance[0] = -1.0          # no orientation from this sensor
        imu.linear_acceleration_covariance[0] = -1.0  # no acceleration either
        imu.angular_velocity.z = s.yaw_rate
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = 1e3
        imu.angular_velocity_covariance[8] = self.var_gyro
        self.imu_pub.publish(imu)

    def _publish_range(self, sample) -> None:
        msg = Range()
        msg.header.stamp = self._stamp(sample.stamp)
        msg.header.frame_id = 'range_' + sample.name
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.range_fov
        msg.min_range = 0.03
        msg.max_range = self.max_range
        msg.range = float(sample.range_m)
        self.range_pubs[sample.name].publish(msg)

    def _publish_status(self, sta: Sta) -> None:
        msg = BaseStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.armed = sta.armed
        msg.tray_deg = sta.tray_deg
        msg.charge_v = sta.charge_v
        msg.battery_v = sta.battery_v
        msg.fault = sta.fault
        self.status_pub.publish(msg)

    # ---- subscriptions and timers ----
    def on_cmd_vel(self, msg: Twist) -> None:
        self.core.set_command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def on_oled(self, msg: String) -> None:
        self._write(protocol.encode_oled(msg.data))

    def on_send_timer(self) -> None:
        now = time.monotonic()
        was_wanted = self.core.want_armed
        line = self.core.velocity_line(now)
        if line is not None:
            self._write(line)
            return
        line = self.core.rearm_line(now)
        if line is not None:
            self.get_logger().warn('Re-arming after a watchdog trip')
            self._write(line)
        elif was_wanted and not self.core.want_armed:
            self.get_logger().error('Watchdog tripped too often: staying disarmed')

    def on_health_timer(self) -> None:
        fault = self.core.link_fault(time.monotonic())
        if fault != self._last_fault:
            if fault:
                self.get_logger().error('Mega link: %s' % fault)
            else:
                self.get_logger().info('Mega link restored')
            self._last_fault = fault

    # ---- services ----
    def on_arm(self, request, response):
        if request.data:
            fault = self.core.link_fault(time.monotonic())
            if fault:
                response.success, response.message = False, fault
                return response
            self.core.set_command(0.0, 0.0, 0.0, time.monotonic())
            response.success = self._command(protocol.ARM, 'OK', 'ARM')
        else:
            response.success = self._command(protocol.DISARM, 'OK', 'DISARM')
        response.message = '' if response.success else 'no reply from the Mega'
        return response

    def on_stop(self, request, response):
        response.success = self._command(protocol.STOP, 'OK', 'STOP')
        response.message = '' if response.success else 'no reply from the Mega'
        return response

    def on_set_tray(self, request, response):
        if not self._command(protocol.encode_tray(request.angle_deg), 'OK', 'TRAY'):
            response.success, response.message = False, 'no reply from the Mega'
            return response
        response.success = self._wait_reply('DONE', 'TRAY', 5.0)
        response.message = '' if response.success else 'tray did not reach the angle in 5 s'
        return response

    def on_cal(self, request, response):
        response.success = self._command(protocol.CAL, 'OK', 'CAL')
        response.message = 'keep the robot still for 2 s' if response.success else 'no reply'
        return response

    def destroy_node(self):
        self._running = False
        try:
            self._write(protocol.STOP)
            self.serial.close()
        except Exception:  # noqa: BLE001 - shutting down; nothing useful to do with the error
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
