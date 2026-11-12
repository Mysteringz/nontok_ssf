"""ROS 2 node: turns tag detections into position fixes for the global EKF (PLAN.md 13.7)."""
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from nontok_interfaces.msg import LocalizationStatus, TagDetectionArray

from . import fix_gate
from .camera import make_camera
from .fix_gate import FixGate, HealthTracker
from .pose_solver import SolverConfig, solve
from .ros_util import detections_of, matrix_of, pose2d_of, seconds_of, set_yaw
from .tag_geometry import TagMap


class TagLocalizer(Node):
    def __init__(self):
        super().__init__('tag_localizer')
        p = self.declare_parameter
        self.tag_map = TagMap.load(p('tags_file', '').value)
        self.cameras_dir = p('cameras_dir', '').value
        self.camera_names = p('cameras', ['front_left', 'front_right', 'rear']).value
        self.cfg = SolverConfig(
            max_range_m=p('max_range_m', 2.5).value,
            max_view_angle_rad=math.radians(p('max_view_angle_deg', 60.0).value),
            yaw_range_m=p('yaw_range_m', 1.5).value,
            max_reprojection_px=p('max_reprojection_px', 3.0).value,
            sigma_xy_1m=p('sigma_xy_1m', 0.02).value,
            sigma_yaw_1m=math.radians(p('sigma_yaw_1m_deg', 2.0).value),
        )
        self.gate = FixGate(
            max_jump_m=p('max_jump_m', 0.5).value,
            max_jump_rad=math.radians(p('max_jump_deg', 20.0).value),
            needed=p('relocalise_after', 3).value,
        )
        self.health = HealthTracker(p('healthy_age_s', 10.0).value, p('healthy_travel_m', 3.0).value)
        # Without tags (bench work with the fake Mega) report healthy so the mission can run.
        self.require_tags = p('require_tags', True).value
        self.dock_pose = p('dock_pose', [0.0, 0.0, 0.0]).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.cameras = {}          # name -> Camera, filled once its transform is known
        self.estimate = None       # (x, y, yaw) from the global EKF
        self.tags_in_view = {}     # camera -> (count, time)
        self.last_reset = -1e9     # when the filter was last reset from tags

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'tag_pose', 10)
        self.reset_pub = self.create_publisher(PoseWithCovarianceStamped, 'ekf_global/set_pose', 2)
        self.status_pub = self.create_publisher(LocalizationStatus, 'localization/status', 2)
        self.create_subscription(Odometry, 'odometry/global', self.on_odometry, 10)
        for name in self.camera_names:
            self.create_subscription(TagDetectionArray, 'tags/%s/detections' % name, self.on_detections, 2)
        self.create_service(Trigger, 'localization/reset_to_dock', self.on_reset_to_dock)
        self.create_timer(0.5, self.on_status_timer)
        self.get_logger().info('Tag localiser: %d tags in the map' % len(self.tag_map))

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odometry(self, msg: Odometry) -> None:
        self.estimate = pose2d_of(msg.pose.pose)
        self.health.on_odometry(self.estimate[0], self.estimate[1])

    def camera_for(self, msg):
        camera = self.cameras.get(msg.camera)
        if camera is not None:
            return camera
        try:
            tf = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time(),
                                                 Duration(seconds=0.0))
        except TransformException:
            return None   # robot_state_publisher not up yet
        path = os.path.join(self.cameras_dir, msg.camera + '.yaml')
        camera = make_camera(msg.camera, path, matrix_of(tf.transform))
        self.cameras[msg.camera] = camera
        return camera

    def on_detections(self, msg: TagDetectionArray) -> None:
        stamp = seconds_of(msg.header.stamp)
        self.tags_in_view[msg.camera] = (len(msg.detections), stamp)
        if not msg.detections:
            return
        camera = self.camera_for(msg)
        if camera is None:
            return
        prior_yaw = self.estimate[2] if self.estimate is not None else None
        fix = solve(detections_of(msg), camera, self.tag_map, self.cfg, prior_yaw)
        if fix is None:
            return
        decision = self.gate.check((fix.x, fix.y, fix.yaw), self.estimate, fix.yaw_trusted)
        if decision == fix_gate.REJECT:
            return
        out = self.pose_message(fix.x, fix.y, fix.yaw, msg.header.stamp, *fix.variances(self.cfg))
        if decision == fix_gate.RELOCALISE:
            if self.now_s() - self.last_reset < 2.0:
                return   # give the filter time to take the last reset in
            self.last_reset = self.now_s()
            self.get_logger().warn('Pose reset from tags to (%.2f, %.2f, %.0f deg)' % (
                fix.x, fix.y, math.degrees(fix.yaw)))
            self.reset_pub.publish(out)
        else:
            self.pose_pub.publish(out)
        self.health.on_fix(stamp)

    def pose_message(self, x, y, yaw, stamp, var_xy, var_yaw) -> PoseWithCovarianceStamped:
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = 'map'
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        set_yaw(out.pose.pose.orientation, yaw)
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = float(var_xy)
        covariance[14] = covariance[21] = covariance[28] = 1e-6   # z, roll, pitch: on the floor
        covariance[35] = float(var_yaw)
        out.pose.covariance = covariance
        return out

    def on_reset_to_dock(self, request, response):
        x, y, yaw = self.dock_pose
        self.reset_pub.publish(self.pose_message(x, y, yaw, self.get_clock().now().to_msg(), 1e-4, 1e-4))
        self.health.on_fix(self.now_s())
        response.success = True
        return response

    def on_status_timer(self) -> None:
        now = self.now_s()
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.tags_in_view = min(255, sum(count for count, t in self.tags_in_view.values() if now - t < 1.0))
        since, travelled = self.health.seconds_since_fix(now), self.health.metres_since_fix()
        msg.seconds_since_fix = float(min(since, 1e6))
        msg.metres_since_fix = float(min(travelled, 1e6))
        msg.healthy = self.health.healthy(now) or not self.require_tags
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
