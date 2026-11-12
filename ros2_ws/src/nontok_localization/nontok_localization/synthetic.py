"""Synthetic camera views of tags, for tests and for trying the pipeline without hardware."""
from typing import List, Optional

import cv2
import numpy as np

from .detector import tag_image
from .geometry import invert, pose2d_to_matrix
from .pose_solver import Camera, Detection
from .tag_geometry import TagMap


def nominal_camera(name: str, base_to_camera: np.ndarray, width: int = 1280, height: int = 720,
                   hfov_deg: float = 60.0) -> Camera:
    """An ideal pinhole camera with the given horizontal field of view."""
    focal = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    matrix = np.array([[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]])
    return Camera(name, matrix, np.zeros(5), base_to_camera, width, height)


def project_tag(tag, camera: Camera, robot_pose) -> Optional[np.ndarray]:
    """Pixel corners of a tag seen from a robot pose, or None if it is not fully in view."""
    camera_to_map = invert(pose2d_to_matrix(robot_pose) @ camera.base_to_camera)
    points = tag.corners_map() @ camera_to_map[:3, :3].T + camera_to_map[:3, 3]
    if np.any(points[:, 2] <= 0.05):
        return None
    # The tag must face the camera.
    camera_position = invert(camera_to_map)[:3, 3]
    if (camera_position - tag.centre()) @ tag.normal() <= 0:
        return None
    pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), camera.matrix, camera.distortion)
    pixels = pixels.reshape(4, 2)
    inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < camera.width) & \
             (pixels[:, 1] >= 0) & (pixels[:, 1] < camera.height)
    return pixels if inside.all() else None


def detections_from_pose(tag_map: TagMap, camera: Camera, robot_pose, noise_px: float = 0.0,
                         rng: Optional[np.random.Generator] = None) -> List[Detection]:
    """What a perfect detector would report from this pose."""
    detections = []
    for tag in tag_map.tags.values():
        corners = project_tag(tag, camera, robot_pose)
        if corners is None:
            continue
        if noise_px > 0:
            rng = rng or np.random.default_rng(0)
            corners = corners + rng.normal(0.0, noise_px, corners.shape)
        detections.append(Detection(tag.id, corners))
    return detections


def render(tag_map: TagMap, camera: Camera, robot_pose, background: int = 150) -> np.ndarray:
    """A grey image of what the camera would see: each visible tag with its white border."""
    image = np.full((camera.height, camera.width), background, dtype=np.uint8)
    side = 400
    border = side // 4   # white margin, as on the printed sheet
    for tag in tag_map.tags.values():
        corners = project_tag(tag, camera, robot_pose)
        if corners is None:
            continue
        sheet = np.full((side + 2 * border, side + 2 * border), 255, dtype=np.uint8)
        sheet[border:border + side, border:border + side] = tag_image(tag.id, side)
        source = np.array([[border, border], [border + side, border],
                           [border + side, border + side], [border, border + side]], dtype=np.float32)
        homography = cv2.getPerspectiveTransform(source, corners.astype(np.float32))
        size = (camera.width, camera.height)
        warped = cv2.warpPerspective(sheet, homography, size, flags=cv2.INTER_AREA)
        mask = cv2.warpPerspective(np.full_like(sheet, 255), homography, size)
        image[mask > 127] = warped[mask > 127]
    return cv2.GaussianBlur(image, (3, 3), 0)
