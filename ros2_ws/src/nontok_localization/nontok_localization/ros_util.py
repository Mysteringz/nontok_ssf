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
