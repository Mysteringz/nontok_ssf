"""Robot pose from the tags seen by one camera in one image (PLAN.md section 13.7).

The robot drives on a flat floor and the camera mounting is known, so only three numbers
are unknown: x, y and heading. They are found by adjusting the pose until the tags' known
corners, projected into the image, land on the detected corners. Solving for three numbers
instead of a free 3D pose is better conditioned, and a pose that would need the robot to
float or lean simply cannot come out.

Pure Python + NumPy + OpenCV; no ROS.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import cv2
import numpy as np

from .geometry import invert, pose2d_to_matrix, wrap
from .tag_geometry import TagMap


@dataclass
class Detection:
    id: int
    corners: np.ndarray           # 4x2 pixels: top-left, top-right, bottom-right, bottom-left
    decision_margin: float = 100.0
    hamming: int = 0


@dataclass
class Camera:
    name: str
    matrix: np.ndarray            # 3x3 intrinsics
    distortion: np.ndarray        # OpenCV distortion coefficients
    base_to_camera: np.ndarray    # 4x4: optical frame expressed in base_link
    width: int = 1280
    height: int = 720


@dataclass
class SolverConfig:
    max_range_m: float = 2.5
    max_view_angle_rad: float = math.radians(60.0)
    yaw_range_m: float = 1.5            # a single tag gives a trusted heading only this close
    ambiguity_ratio: float = 2.0        # the mirror solution must fit this much worse to be ruled out
    max_reprojection_px: float = 3.0
    sigma_xy_1m: float = 0.02           # position noise at 1 m; grows with distance squared
    sigma_yaw_1m: float = math.radians(2.0)
    untrusted_yaw_variance: float = 1000.0
    prior_yaw_error_rad: float = math.radians(3.0)   # assumed heading error behind a position-only fix


@dataclass
class Fix:
    x: float
    y: float
    yaw: float
    n_tags: int
    distance: float               # mean camera-to-tag distance
    view_angle: float             # smallest angle off a tag's normal
    reprojection_px: float
    yaw_trusted: bool
    map_to_base: np.ndarray = field(repr=False, default=None)

    def variances(self, cfg: SolverConfig):
        """(variance of x and y, variance of yaw) for the filter."""
        d = max(self.distance, 0.5)
        var_xy = (cfg.sigma_xy_1m * d * d) ** 2
        if not self.yaw_trusted:
            var_xy += (d * cfg.prior_yaw_error_rad) ** 2
        var_yaw = (cfg.sigma_yaw_1m * d) ** 2 if self.yaw_trusted else cfg.untrusted_yaw_variance
        return var_xy, var_yaw


def _projection(camera: Camera):
    """Constants for projecting robot-frame points into the camera, worked out once per camera."""
    cached = getattr(camera, '_projection', None)
    if cached is None:
        camera_from_base = invert(camera.base_to_camera)
        k = camera.matrix
        cached = (camera_from_base[:3, :3].T.copy(), camera_from_base[:3, 3].copy(),
                  float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]))
        camera._projection = cached
    return cached


def _undistorted(img: np.ndarray, camera: Camera) -> np.ndarray:
    """Detected corners with the lens distortion taken out, still in pixels. Doing this once
    lets the fit below use plain pinhole arithmetic, which is what makes it fast enough for
    the Jetson Nano (it took 18-35 ms per image there when every step called OpenCV)."""
    return cv2.undistortPoints(img.reshape(-1, 1, 2).copy(), camera.matrix, camera.distortion,
                               P=camera.matrix).reshape(-1, 2)


def _residuals(pose, obj: np.ndarray, img: np.ndarray, camera: Camera, jacobian: bool = False):
    """Pixel errors of all corners for a robot pose (x, y, yaw); None if a corner is behind
    the camera. `img` holds undistorted pixels. With jacobian=True also returns the
    derivative of the errors with respect to x, y and yaw."""
    rotation_t, translation, fx, fy, cx, cy = _projection(camera)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = obj[:, 0] - pose[0], obj[:, 1] - pose[1]
    base = np.empty((obj.shape[0], 3))
    base[:, 0] = c * dx + s * dy
    base[:, 1] = -s * dx + c * dy
    base[:, 2] = obj[:, 2]
    points = base @ rotation_t + translation
    z = points[:, 2]
    if np.any(z <= 0.02):
        return None
    residual = np.empty(obj.shape[0] * 2)
    residual[0::2] = fx * points[:, 0] / z + cx - img[:, 0]
    residual[1::2] = fy * points[:, 1] / z + cy - img[:, 1]
    if not jacobian:
        return residual
    out = np.empty((residual.size, 3))
    zero = np.zeros(obj.shape[0])
    for k, (bx, by) in enumerate(((np.full(obj.shape[0], -c), np.full(obj.shape[0], s)),
                                  (np.full(obj.shape[0], -s), np.full(obj.shape[0], -c)),
                                  (base[:, 1], -base[:, 0]))):
        d = np.stack((bx, by, zero), axis=1) @ rotation_t
        out[0::2, k] = fx * (d[:, 0] * z - points[:, 0] * d[:, 2]) / (z * z)
        out[1::2, k] = fy * (d[:, 1] * z - points[:, 1] * d[:, 2]) / (z * z)
    return residual, out


def _refine(start, obj: np.ndarray, img: np.ndarray, camera: Camera):
    """Levenberg-Marquardt over (x, y, yaw). Returns (pose, rms pixel error) or None."""
    pose = np.array(start, dtype=float)
    first = _residuals(pose, obj, img, camera, jacobian=True)
    if first is None:
        return None
    residual, jacobian = first
    damping = 1e-3
    for _ in range(40):
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        try:
            delta = np.linalg.solve(normal + damping * np.diag(np.diag(normal)) + 1e-12 * np.eye(3), -gradient)
        except np.linalg.LinAlgError:
            return None
        trial = _residuals(pose + delta, obj, img, camera, jacobian=True)
        if trial is not None and trial[0] @ trial[0] < residual @ residual:
            gain = 1.0 - (trial[0] @ trial[0]) / max(residual @ residual, 1e-30)
            pose = pose + delta
            residual, jacobian = trial
            damping = max(damping * 0.3, 1e-9)
            # Stop at 0.01 mm / 0.001 deg, or when a step no longer improves the fit:
            # corner noise is worth far more than that, and each step costs the Jetson time.
            if (abs(delta[0]) < 1e-5 and abs(delta[1]) < 1e-5 and abs(delta[2]) < 2e-5) or gain < 1e-6:
                break
        else:
            if abs(delta[0]) < 1e-5 and abs(delta[1]) < 1e-5 and abs(delta[2]) < 2e-5:
                break
            damping *= 10.0
            if damping > 1e8:
                break
    rms = float(np.sqrt(residual @ residual / (residual.size / 2)))
    return (float(pose[0]), float(pose[1]), wrap(float(pose[2]))), rms


def _camera_planar(camera: Camera):
    """The camera's position and viewing direction in the robot's horizontal plane."""
    m = camera.base_to_camera
    return float(m[0, 3]), float(m[1, 3]), math.atan2(m[1, 2], m[0, 2])


def _tag_in_base(detection: Detection, tag, camera: Camera) -> np.ndarray:
    """Rough position of a tag relative to the robot, from its apparent size and place in the image."""
    corners = np.asarray(detection.corners, dtype=float).reshape(4, 2)
    undistorted = cv2.undistortPoints(corners.reshape(-1, 1, 2).copy(), camera.matrix,
                                      camera.distortion).reshape(4, 2)   # normalised image coordinates
    sides = [np.linalg.norm(undistorted[i] - undistorted[(i + 1) % 4]) for i in range(4)]
    centre = undistorted.mean(axis=0)
    ray = np.array([centre[0], centre[1], 1.0])
    depth = tag.size / max(float(np.mean(sides)), 1e-9)   # pinhole: size / (size in normalised units)
    point = camera.base_to_camera @ np.append(ray * depth, 1.0)
    return point[:3]


def _start_pose(tag, tag_in_base: np.ndarray, camera: Camera, off_normal: float):
    """A robot pose that puts the tag where it was seen, with the camera `off_normal` radians
    to one side of the tag's normal (0 = straight in front of it)."""
    cx, cy, _ = _camera_planar(camera)
    to_tag = math.atan2(tag_in_base[1] - cy, tag_in_base[0] - cx)   # camera -> tag, in the robot frame
    yaw = wrap(tag.yaw + math.pi + off_normal - to_tag)
    c, s = math.cos(yaw), math.sin(yaw)
    return (tag.x - (c * tag_in_base[0] - s * tag_in_base[1]),
            tag.y - (s * tag_in_base[0] + c * tag_in_base[1]), yaw)


def _off_normal(pose, tag, camera: Camera) -> float:
    """Signed angle between the tag's normal and the direction from the tag to the camera."""
    cx, cy, _ = _camera_planar(camera)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    camera_x, camera_y = pose[0] + c * cx - s * cy, pose[1] + s * cx + c * cy
    return wrap(math.atan2(camera_y - tag.y, camera_x - tag.x) - tag.yaw)


def solve(detections: Sequence[Detection], camera: Camera, tag_map: TagMap,
          cfg: Optional[SolverConfig] = None, prior_yaw: Optional[float] = None) -> Optional[Fix]:
    """The robot's pose in the tag map's frame, or None if no trustworthy answer exists.

    prior_yaw is the current heading estimate. It is used when the image alone cannot fix
    the heading: a single tag that is far away or seen almost square-on.
    """
    cfg = cfg or SolverConfig()
    known: List[Detection] = []
    seen = set()
    for detection in detections:
        if detection.id in tag_map and detection.id not in seen:
            seen.add(detection.id)
            known.append(detection)
    if not known:
        return None
    tags = [tag_map.get(d.id) for d in known]
    obj = np.vstack([tag.corners_map() for tag in tags])
    img = _undistorted(np.vstack([np.asarray(d.corners, dtype=float).reshape(4, 2) for d in known]), camera)

    # Start from the largest tag in the image.
    areas = [cv2.contourArea(np.asarray(d.corners, dtype=np.float32).reshape(4, 2)) for d in known]
    lead = int(np.argmax(areas))
    rough = _tag_in_base(known[lead], tags[lead], camera)
    best = _refine(_start_pose(tags[lead], rough, camera, 0.0), obj, img, camera)
    if best is None:
        return None
    pose, rms = best

    # A flat square fits two poses almost equally well when it is small in the image: the
    # camera a little to the left of the tag's normal, or the same amount to the right.
    # Look for the mirror solution and see whether the image can tell them apart.
    unambiguous = True
    angle = _off_normal(pose, tags[lead], camera)
    mirror = _refine(_start_pose(tags[lead], rough, camera, -angle), obj, img, camera)
    if mirror is not None:
        other_pose, other_rms = mirror
        distinct = (math.hypot(other_pose[0] - pose[0], other_pose[1] - pose[1]) > 0.01
                    or abs(wrap(other_pose[2] - pose[2])) > math.radians(0.5))
        if distinct:
            if other_rms < rms:
                pose, rms, other_rms = other_pose, other_rms, rms
            if other_rms < cfg.ambiguity_ratio * max(rms, 0.05):
                unambiguous = False
    if rms > cfg.max_reprojection_px:
        return None   # nothing on the floor explains this image: wrong tag pose, corner order or calibration

    c, s = math.cos(pose[2]), math.sin(pose[2])
    cx, cy, _ = _camera_planar(camera)
    camera_position = np.array([pose[0] + c * cx - s * cy, pose[1] + s * cx + c * cy,
                                float(camera.base_to_camera[2, 3])])
    distances, angles = [], []
    for tag in tags:
        to_camera = camera_position - tag.centre()
        distance = float(np.linalg.norm(to_camera))
        distances.append(distance)
        angles.append(math.acos(max(-1.0, min(1.0, float(to_camera @ tag.normal()) / max(distance, 1e-9)))))
    distance, view_angle = float(np.mean(distances)), float(min(angles))
    if distance > cfg.max_range_m or view_angle > cfg.max_view_angle_rad:
        return None

    n = len(tags)
    trusted = unambiguous and (n >= 2 or distance <= cfg.yaw_range_m)
    fix = Fix(pose[0], pose[1], pose[2], n, distance, view_angle, rms, trusted, pose2d_to_matrix(pose))
    if trusted or prior_yaw is None:
        return fix
    # Position-only fix. The image fixes where the tag is relative to the robot well, but
    # not the angle the robot sees it from, and every degree of that angle error becomes
    # centimetres of sideways position error. Use the current heading estimate instead.
    lead_tag = tags[lead]
    tag_in_base = invert(pose2d_to_matrix(pose)) @ np.append(lead_tag.centre(), 1.0)
    c, s = math.cos(prior_yaw), math.sin(prior_yaw)
    fix.x = lead_tag.x - (c * tag_in_base[0] - s * tag_in_base[1])
    fix.y = lead_tag.y - (s * tag_in_base[0] + c * tag_in_base[1])
    fix.yaw = prior_yaw
    fix.map_to_base = pose2d_to_matrix((fix.x, fix.y, fix.yaw))
    return fix
