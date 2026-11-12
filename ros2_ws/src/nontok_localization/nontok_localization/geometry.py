"""Small planar and 3D transform helpers shared by localisation and docking. NumPy only."""
import math
from typing import Sequence, Tuple

import numpy as np

Pose2D = Tuple[float, float, float]  # x, y, yaw


def wrap(angle: float) -> float:
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """Pose b, given in frame a, expressed in a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def between(a: Pose2D, b: Pose2D) -> Pose2D:
    """Pose b expressed in frame a (both given in the same parent frame)."""
    return compose(inverse(a), b)


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def invert(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def pose2d_to_matrix(pose: Pose2D, z: float = 0.0) -> np.ndarray:
    return transform(rot_z(pose[2]), (pose[0], pose[1], z))


def matrix_to_pose2d(t: np.ndarray) -> Pose2D:
    return (float(t[0, 3]), float(t[1, 3]), math.atan2(t[1, 0], t[0, 0]))


def tilt_of(t: np.ndarray) -> float:
    """Angle between the frame's z axis and the parent's z axis (0 for a robot on a flat floor)."""
    return math.acos(max(-1.0, min(1.0, float(t[2, 2]))))


# Camera optical frame (z forward, x right, y down) relative to a body-style frame
# (x forward, y left, z up) at the same place.
BODY_TO_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def camera_mount(x: float, y: float, z: float, yaw: float, pitch: float = 0.0) -> np.ndarray:
    """base_link -> camera optical frame, for a camera at (x, y, z) looking along `yaw`.

    pitch > 0 tilts the camera downwards.
    """
    return transform(rot_z(yaw) @ rot_y(pitch) @ BODY_TO_OPTICAL, (x, y, z))
