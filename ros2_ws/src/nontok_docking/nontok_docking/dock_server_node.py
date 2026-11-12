"""ROS 2 node: tag-guided alignment at stations and the charger (PLAN.md section 13.9).

Action /dock_to_tags drives the robot from a station's approach pose to its standoff.
Service /undock reverses a fixed short distance (the only reversing the robot does).
Publishes /cmd_vel only while one of them is running.
"""
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from nontok_interfaces.action import DockToTags
from nontok_interfaces.msg import BaseStatus, TagDetectionArray
from nontok_localization.camera import make_camera
from nontok_localization.pose_solver import SolverConfig, solve
from nontok_localization.ros_util import detections_of, matrix_of, pose2d_of, seconds_of
from nontok_localization.tag_geometry import TagMap, pair_map
from nontok_navigation.stations import load_stations, tag_spacing

from .dock_controller import DockConfig, DockController, PoseBuffer


class DockServer(Node):
    def __init__(self):
        super().__init__('dock_server')
        loc_share = get_package_share_directory('nontok_localization')
        nav_share = get_package_share_directory('nontok_navigation')
        p = self.declare_parameter
        self.tag_map = TagMap.load(p('tags_file', os.path.join(loc_share, 'config', 'tags.yaml')).value)
        self.stations = load_stations(p('stations_file', os.path.join(nav_share, 'config', 'stations.yaml')).value)
        self.cameras_dir = p('cameras_dir', os.path.join(loc_share, 'config', 'cameras')).value
        camera_names = p('cameras', ['front_left', 'front_right']).value
        for name, default in [
                ('approach_speed', 0.08), ('final_speed', 0.04), ('max_yaw_rate', 0.3),
                ('gain_forward', 0.8), ('gain_sideways', 1.0), ('gain_yaw', 1.5), ('pre_distance_m', 0.15),
                ('lateral_tolerance_m', 0.015), ('distance_tolerance_m', 0.020), ('yaw_tolerance_deg', 3.0),
                ('dock_lateral_tolerance_m', 0.010), ('dock_distance_tolerance_m', 0.010),
                ('dock_yaw_tolerance_deg', 2.0), ('blind_distance_m', 0.30), ('total_timeout_s', 30.0),
                ('charge_threshold_v', 4.0), ('undock_distance_m', 0.30), ('undock_speed', 0.08),
                ('max_measurement_age_s', 0.4)]:
            p(name, default)
        self.solver_cfg = SolverConfig(max_range_m=2.0)

        self.tf_buffer = Buffer()
        # /tf is busy; its own thread keeps it from delaying the measurements below.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.cameras = {}
        self.odom = PoseBuffer()
        self.front_range = None
        self.charge_v = 0.0
        self.lock = threading.Lock()
        self.controller = None       # set while an action runs
        self.worst_age = 0.0         # oldest tag message seen during the current action
        self.dropped = 0             # tag messages dropped for being too old
        self.odom_age = self.worst_odom_age = 0.0
        self.odom_count = 0
        self.unsolved = 0            # ... giving no trustworthy pose
        self.unmatched = 0           # ... with no odometry from the same moment
        self.pair = None

        group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # The sensor inputs live on their own node, served by their own thread (see main).
        # Sharing an executor with the long-running action starved them: odometry arrived
        # at 3-8 Hz and up to a second late, and the robot then steered on old data.
        self.inputs = rclpy.create_node('dock_server_inputs')
        self.inputs.create_subscription(Odometry, 'odometry/local', self.on_odometry, 20)
        self.inputs.create_subscription(Range, 'range/front', self.on_front, 5)
        self.inputs.create_subscription(BaseStatus, 'base/status', self.on_status, 5)
        for name in camera_names:
            # Depth 1: only the newest view of the tags is of any use.
            self.inputs.create_subscription(TagDetectionArray, 'tags/%s/detections' % name, self.on_detections, 1)
        self.create_service(Trigger, 'undock', self.on_undock, callback_group=group)
        self.action = ActionServer(
            self, DockToTags, 'dock_to_tags', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=group)

    # ---- inputs ----
    def on_odometry(self, msg):
        stamp = seconds_of(msg.header.stamp)
        self.odom_age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        self.worst_odom_age = max(self.worst_odom_age, self.odom_age)
        self.odom_count += 1
        with self.lock:
            self.odom.add(stamp, pose2d_of(msg.pose.pose))

    def on_front(self, msg):
        self.front_range = msg.range if msg.range < msg.max_range else None

    def on_status(self, msg):
        self.charge_v = msg.charge_v

    def camera_for(self, msg):
        camera = self.cameras.get(msg.camera)
        if camera is None:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time(), Duration(seconds=0.0))
            except TransformException:
                return None
            camera = make_camera(msg.camera, os.path.join(self.cameras_dir, msg.camera + '.yaml'),
                                 matrix_of(tf.transform))
            self.cameras[msg.camera] = camera
        return camera

    def on_detections(self, msg):
        with self.lock:
            controller, pair = self.controller, self.pair
        if controller is None or not msg.detections:
            return
        # Steering on an old view of the tags makes the robot overshoot and weave, so a late
        # measurement is dropped, and the controller is told how old the ones it gets are.
        age = self.get_clock().now().nanoseconds * 1e-9 - seconds_of(msg.header.stamp)
        self.worst_age = max(self.worst_age, age)
        if age > self.value('max_measurement_age_s'):
            self.dropped += 1
            return
        camera = self.camera_for(msg)
        if camera is None:
            return
        fix = solve(detections_of(msg), camera, pair, self.solver_cfg)
        if fix is None or not fix.yaw_trusted:
            self.unsolved += 1
            return
        with self.lock:
            odom_then = self.odom.at(seconds_of(msg.header.stamp))
        if odom_then is None:
            self.unmatched += 1
            return
        with self.lock:
            if self.controller is controller:
                controller.on_measurement((fix.x, fix.y, fix.yaw), odom_then, time.monotonic() - max(age, 0.0))

    # ---- helpers ----
    def publish(self, vx=0.0, vy=0.0, wz=0.0):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(twist)

    def value(self, name):
        return self.get_parameter(name).value

    def config_for(self, station) -> DockConfig:
        prefix = 'dock_' if station.is_dock else ''
        return DockConfig(
            standoff_m=station.standoff_m, lateral_offset_m=station.lateral_offset_m,
            pre_distance_m=self.value('pre_distance_m'),
            approach_speed=self.value('approach_speed'), final_speed=self.value('final_speed'),
            max_yaw_rate=self.value('max_yaw_rate'), gain_forward=self.value('gain_forward'),
            gain_sideways=self.value('gain_sideways'), gain_yaw=self.value('gain_yaw'),
            lateral_tolerance_m=self.value(prefix + 'lateral_tolerance_m'),
            distance_tolerance_m=self.value(prefix + 'distance_tolerance_m'),
            yaw_tolerance_rad=math.radians(self.value(prefix + 'yaw_tolerance_deg')),
            blind_distance_m=self.value('blind_distance_m'), total_timeout_s=self.value('total_timeout_s'),
            charge_threshold_v=self.value('charge_threshold_v'))

    # ---- action ----
    def on_goal(self, goal):
        if goal.station not in self.stations:
            self.get_logger().error('Unknown station "%s"' % goal.station)
            return GoalResponse.REJECT
        with self.lock:
            return GoalResponse.REJECT if self.controller is not None else GoalResponse.ACCEPT

    def execute(self, goal_handle):
        station = self.stations[goal_handle.request.station]
        left = self.tag_map.get(station.left_tag)
        pair = pair_map(station.left_tag, station.right_tag, tag_spacing(station, self.tag_map), left.size, left.z)
        controller = DockController(self.config_for(station), goal_handle.request.require_charge)
        with self.lock:
            self.controller, self.pair = controller, pair
            self.worst_age, self.dropped, self.unsolved, self.unmatched = 0.0, 0, 0, 0
            self.worst_odom_age, self.odom_count = 0.0, 0
            started = time.monotonic()
        controller.start(time.monotonic())
        feedback = DockToTags.Feedback()
        result = DockToTags.Result()
        try:
            while rclpy.ok() and not controller.done:
                if goal_handle.is_cancel_requested:
                    self.publish()
                    goal_handle.canceled()
                    result.message = 'cancelled'
                    return result
                with self.lock:
                    pose = self.odom.latest()
                if pose is None:
                    time.sleep(0.05)
                    continue
                with self.lock:
                    command = controller.step(pose, time.monotonic(), self.front_range, self.charge_v)
                self.publish(*command)
                feedback.phase = controller.phase
                feedback.distance_error_m, feedback.lateral_error_m, feedback.yaw_error_rad = (
                    float(e) for e in controller.errors)
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
        finally:
            self.publish()
            with self.lock:
                self.controller, self.pair = None, None
        result.success = controller.success
        result.distance_error_m, result.lateral_error_m, result.yaw_error_rad = (float(e) for e in controller.errors)
        result.message = controller.message
        self.get_logger().info('Docking at %s: %s (%s); errors %.0f mm forward, %.0f mm sideways, %.1f deg; '
                               '%d tag fixes, last %.1f s before the end; oldest tag message %.2f s; rejected: %d late, %d no pose, %d no odometry; odometry %.0f Hz, up to %.2f s old' % (
                                   station.name, controller.phase, controller.message,
                                   result.distance_error_m * 1000, result.lateral_error_m * 1000,
                                   math.degrees(result.yaw_error_rad), controller.measurements,
                                   controller.tag_age(time.monotonic()), self.worst_age, self.dropped, self.unsolved,
                                   self.unmatched, self.odom_count / max(time.monotonic() - started, 0.1),
                                   self.worst_odom_age))
        if controller.success:
            goal_handle.succeed()
        else:
            self.get_logger().warn('Docking at %s failed: %s' % (station.name, controller.message))
            goal_handle.abort()
        return result

    # ---- undock ----
    def on_undock(self, request, response):
        with self.lock:
            busy = self.controller is not None
        start = self.odom.latest()
        if busy or start is None:
            response.success, response.message = False, 'busy' if busy else 'no odometry'
            return response
        distance, speed = self.value('undock_distance_m'), self.value('undock_speed')
        deadline = time.monotonic() + 2.0 * distance / speed + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            pose = self.odom.latest()
            if math.hypot(pose[0] - start[0], pose[1] - start[1]) >= distance:
                response.success = True
                break
            self.publish(vx=-speed)
            time.sleep(0.05)
        self.publish()
        if not response.success:
            response.message = 'did not move the full distance'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DockServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    inputs = SingleThreadedExecutor()
    inputs.add_node(node.inputs)
    threading.Thread(target=inputs.spin, name='inputs', daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
