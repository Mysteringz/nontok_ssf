"""A camera that always hands out its newest frame, with the time it was captured."""
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

from .pose_solver import Camera


def decode_frame(raw: np.ndarray, gray: bool) -> Optional[np.ndarray]:
    """What the camera handed over -> an image. `raw` is a JPEG byte buffer when the camera
    is read undecoded, or an already decoded frame from a driver that ignores that request."""
    if raw is None:
        return None
    if raw.ndim == 3 or (raw.ndim == 2 and min(raw.shape) > 1):
        return cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY) if gray and raw.ndim == 3 else raw
    return cv2.imdecode(raw.reshape(-1), cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR)


class LatestFrameCamera:
    """Reads a V4L2 camera in a thread and keeps only the most recent frame.

    Tag detection is slower than the camera; without this, frames queue up inside the driver
    and the detector works on old images.

    The frames stay as the camera sends them (JPEG) until read() is called, and with
    gray=True they are decoded straight to grey. On the Jetson Nano decoding every 1280x720
    frame to colour took 19 ms, 30 times a second, for a detector that looks at a few frames
    a second in grey (7.7 ms each).
    """

    def __init__(self, device, width: int = 1280, height: int = 720, fps: int = 30,
                 exposure: Optional[float] = None, gray: bool = False):
        self.gray = gray
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2) if hasattr(cv2, 'CAP_V4L2') \
            else cv2.VideoCapture(device)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # hand over the JPEG; decode_frame copes if refused
        if exposure is not None:
            # A fixed short exposure limits motion blur. On V4L2, 1 = manual and 3 = automatic;
            # the units of the exposure value depend on the camera (check with v4l2-ctl).
            self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self._lock = threading.Lock()
        self._raw: Optional[np.ndarray] = None
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
            ok, raw = self.capture.retrieve()
            if ok:
                with self._lock:
                    self._raw, self._stamp, self._fresh = raw, stamp, True

    def read(self) -> Optional[Tuple[np.ndarray, float]]:
        """(frame, capture time on time.monotonic()) once per new frame; None if nothing new."""
        with self._lock:
            if not self._fresh:
                return None
            self._fresh = False
            raw, stamp = self._raw, self._stamp
        frame = decode_frame(np.asarray(raw), self.gray)
        return None if frame is None else (frame, stamp)

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
