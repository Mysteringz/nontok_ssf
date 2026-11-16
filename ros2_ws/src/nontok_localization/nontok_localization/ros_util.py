"""Conversions between ROS messages and the plain types used by the logic modules."""
import math

import numpy as np

from .geometry import transform
from .pose_solver import Detection


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose2d_of(pose) -> tuple:
    """geometry_msgs/Pose -> (x, y, yaw)."""
    return (pose.position.x, pose.position.y, yaw_of(pose.orientation))


def set_yaw(q, yaw: float) -> None:
    q.x, q.y = 0.0, 0.0
    q.z, q.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def matrix_of(tf) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 matrix."""
    q = tf.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return transform(rotation, (tf.translation.x, tf.translation.y, tf.translation.z))


def detections_of(msg) -> list:
    """nontok_interfaces/TagDetectionArray -> list of Detection."""
    return [Detection(d.id, np.array(d.corners, dtype=float).reshape(4, 2), d.decision_margin, d.hamming)
            for d in msg.detections]


def seconds_of(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class CameraFinder:
    """Looks up where each camera is mounted, then stops listening to /tf.

    The mounting never changes, so one lookup per camera is enough. Listening to /tf for
    ever costs a Python node a large share of a Jetson Nano core, because every transform
    the filters publish has to be unpacked.

    The lookup runs on a throw-away node with its own thread, which is destroyed when all
    cameras are found. (Removing a /tf subscription from the owner's node while it was
    spinning left the node hung, publishing nothing.)
    """

    def __init__(self, node, names, cameras_dir, make_camera):
        import threading
        self.logger = node.get_logger()
        self.node_name = node.get_name() + '_camera_finder'
        self.names, self.cameras_dir, self.make_camera = list(names), cameras_dir, make_camera
        self.cameras = {}
        threading.Thread(target=self._run, name='camera_finder', daemon=True).start()

    def _run(self):
        import os
        import rclpy
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformException, TransformListener
        node = rclpy.create_node(self.node_name)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        buffer = Buffer()
        listener = TransformListener(buffer, node, spin_thread=False)
        try:
            while rclpy.ok() and len(self.cameras) < len(self.names):
                executor.spin_once(timeout_sec=0.2)
                for name in self.names:
                    if name in self.cameras:
                        continue
                    try:
                        tf = buffer.lookup_transform('base_link', 'cam_%s_optical' % name, Time(),
                                                     Duration(seconds=0.0))
                    except TransformException:
                        continue   # robot_state_publisher not up yet
                    path = os.path.join(self.cameras_dir, name + '.yaml')
                    self.cameras[name] = self.make_camera(name, path, matrix_of(tf.transform))
            if len(self.cameras) == len(self.names):
                self.logger.info('Camera mountings found: %s' % ', '.join(self.names))
        except Exception as error:   # shutting down
            self.logger.debug('Camera finder stopped: %s' % error)
        finally:
            try:
                executor.remove_node(node)
                executor.shutdown(timeout_sec=0.0)
                node.destroy_node()
            except Exception:
                pass
            del listener

    def get(self, name):
        return self.cameras.get(name)
