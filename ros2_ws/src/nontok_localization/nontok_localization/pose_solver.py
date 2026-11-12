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


def _residuals(pose, obj: np.ndarray, img: np.ndarray, camera: Camera) -> Optional[np.ndarray]:
    """Pixel errors of all corners for a robot pose (x, y, yaw); None if a corner is behind the camera."""
    camera_from_map = invert(pose2d_to_matrix(pose) @ camera.base_to_camera)
    points = obj @ camera_from_map[:3, :3].T + camera_from_map[:3, 3]
    if np.any(points[:, 2] <= 0.02):
        return None
    pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), camera.matrix, camera.distortion)
    return (pixels.reshape(-1, 2) - img).ravel()


def _refine(start, obj: np.ndarray, img: np.ndarray, camera: Camera):
    """Levenberg-Marquardt over (x, y, yaw). Returns (pose, rms pixel error) or None."""
    pose = np.array(start, dtype=float)
    residual = _residuals(pose, obj, img, camera)
    if residual is None:
        return None
    damping = 1e-3
    for _ in range(40):
        jacobian = np.empty((residual.size, 3))
        for k, step in enumerate((1e-5, 1e-5, 1e-5)):
            moved = pose.copy()
            moved[k] += step
            shifted = _residuals(moved, obj, img, camera)
            if shifted is None:
                return None
            jacobian[:, k] = (shifted - residual) / step
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        try:
            delta = np.linalg.solve(normal + damping * np.diag(np.diag(normal)) + 1e-12 * np.eye(3), -gradient)
        except np.linalg.LinAlgError:
            return None
        trial = _residuals(pose + delta, obj, img, camera)
        if trial is not None and trial @ trial < residual @ residual:
            pose, residual = pose + delta, trial
            damping = max(damping * 0.3, 1e-9)
            if abs(delta[0]) < 1e-6 and abs(delta[1]) < 1e-6 and abs(delta[2]) < 1e-7:
                break
        else:
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
    img = np.vstack([np.asarray(d.corners, dtype=float).reshape(4, 2) for d in known])

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
