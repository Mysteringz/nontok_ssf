"""A camera that always hands out its newest frame, with the time it was captured."""
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

from .pose_solver import Camera


class LatestFrameCamera:
    """Reads a V4L2 camera in a thread and keeps only the most recent frame.

    Tag detection is slower than the camera; without this, frames queue up inside the driver
    and the detector works on old images.
    """

    def __init__(self, device, width: int = 1280, height: int = 720, fps: int = 30,
                 exposure: Optional[float] = None):
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2) if hasattr(cv2, 'CAP_V4L2') \
            else cv2.VideoCapture(device)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if exposure is not None:
            # A fixed short exposure limits motion blur. On V4L2, 1 = manual and 3 = automatic;
            # the units of the exposure value depend on the camera (check with v4l2-ctl).
            self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._stamp = 0.0
        self._fresh = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, name='camera', daemon=True)
        self._thread.start()

    def is_open(self) -> bool:
        return self.capture.isOpened()

    def _loop(self) -> None:
        while self._running:
            if not self.capture.grab():
                time.sleep(0.05)
                continue
            stamp = time.monotonic()
            ok, frame = self.capture.retrieve()
            if ok:
                with self._lock:
                    self._frame, self._stamp, self._fresh = frame, stamp, True

    def read(self) -> Optional[Tuple[np.ndarray, float]]:
        """(frame, capture time on time.monotonic()) once per new frame; None if nothing new."""
        with self._lock:
            if not self._fresh:
                return None
            self._fresh = False
            return self._frame, self._stamp

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self.capture.release()


def load_intrinsics(path: str):
    """(camera matrix, distortion, width, height, calibrated?) from a calibration YAML file."""
    with open(path) as handle:
        data = yaml.safe_load(handle)
    matrix = np.array(data['camera_matrix'], dtype=float).reshape(3, 3)
    distortion = np.array(data['distortion'], dtype=float).ravel()
    return matrix, distortion, int(data['width']), int(data['height']), bool(data.get('calibrated', False))


def make_camera(name: str, intrinsics_path: str, base_to_camera: np.ndarray) -> Camera:
    matrix, distortion, width, height, _ = load_intrinsics(intrinsics_path)
    return Camera(name, matrix, distortion, base_to_camera, width, height)
