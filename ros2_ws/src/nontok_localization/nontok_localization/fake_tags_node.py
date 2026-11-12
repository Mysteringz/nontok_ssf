"""ROS 2 node: stands in for the tag detectors when there is no robot (fake:=true).

It takes the simulated Mega's wheel odometry as the robot's true pose (the simulated wheels
never slip, and the robot starts on the dock at the map origin), works out which tags each
camera would see from there, and publishes their corners with a little pixel noise on the
same topics the real detectors use. Everything downstream - pose solver, filter, docking -
runs unchanged.
"""
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from nontok_interfaces.msg import TagDetection, TagDetectionArray

from .camera import make_camera
from .geometry import pose2d_to_matrix
from .ros_util import matrix_of, pose2d_of
from .synthetic import detections_from_pose
from .tag_geometry import TagMap


def compose(start, odom):
    """Map pose of a robot that started at `start` and has since moved by `odom`."""
    m = pose2d_to_matrix(start) @ pose2d_to_matrix(odom)
    return float(m[0, 3]), float(m[1, 3]), float(np.arctan2(m[1, 0], m[0, 0]))


class FakeTags(Node):
    def __init__(self):
        super().__init__('fake_tags')
        p = self.declare_parameter
        self.tag_map = TagMap.load(p('tags_file', '').value)
        self.cameras_dir = p('cameras_dir', '').value
        self.camera_names = p('cameras', ['front_left', 'front_right', 'rear']).value
        self.noise_px = p('noise_px', 0.3).value
        self.max_range = p('max_range_m', 3.0).value
        self.start_pose = p('start_pose', [0.0, 0.0, 0.0]).value
        self.rng = np.random.default_rng(1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.cameras = {}
        self.truth = None
        self.pubs = {name: self.create_publisher(TagDetectionArray, 'tags/%s/detections' % name, 5)
                     for name in self.camera_names}
        self.create_subscription(Odometry, 'wheel/odom', self.on_odometry, 20)
        self.create_timer(1.0 / p('rate_hz', 8.0).value, self.on_timer)
        self.get_logger().info('Simulating tag detections for %s' % ', '.join(self.camera_names))

    def on_odometry(self, msg: Odometry) -> None:
        self.truth = compose(self.start_pose, pose2d_of(msg.pose.pose))

    def camera(self, name):
        camera = self.cameras.get(name)
        if camera is None:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'cam_%s_optical' % name, Time(),
                                                     Duration(seconds=0.0))
            except TransformException:
                return None
            camera = make_camera(name, os.path.join(self.cameras_dir, name + '.yaml'), matrix_of(tf.transform))
            self.cameras[name] = camera
        return camera

    def on_timer(self) -> None:
        if self.truth is None:
            return
        stamp = self.get_clock().now().to_msg()
        for name, pub in self.pubs.items():
            camera = self.camera(name)
            if camera is None:
                continue
            msg = TagDetectionArray()
            msg.header.stamp = stamp
            msg.header.frame_id = 'cam_%s_optical' % name
            msg.camera = name
            for d in detections_from_pose(self.tag_map, camera, self.truth, self.noise_px, self.rng):
                tag = self.tag_map.get(d.id)
                if np.hypot(tag.x - self.truth[0], tag.y - self.truth[1]) > self.max_range:
                    continue
                out = TagDetection()
                out.id = int(d.id)
                out.hamming = 0
                out.decision_margin = 100.0
                out.corners = [float(v) for v in d.corners.reshape(8)]
                msg.detections.append(out)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTags()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
