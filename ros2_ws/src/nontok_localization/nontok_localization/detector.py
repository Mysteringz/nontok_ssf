"""AprilTag 36h11 detection behind one function, with two interchangeable backends.

- 'pupil': the reference AprilTag library, from the pupil-apriltags or dt-apriltags package. Preferred.
- 'opencv': OpenCV's ArUco module with the AprilTag 36h11 dictionary. Fallback.

The two libraries list a tag's corners in different orders. Rather than trust the
documentation, the detector checks itself at start-up: it draws a tag with the same
generator the printed tags come from (tools: make_tags), detects it, and works out the
order. Every detection is then returned as top-left, top-right, bottom-right, bottom-left.
"""
from typing import List

import cv2
import numpy as np

from .pose_solver import Detection

_aruco = cv2.aruco


def _dictionary():
    if hasattr(_aruco, 'getPredefinedDictionary'):
        return _aruco.getPredefinedDictionary(_aruco.DICT_APRILTAG_36h11)
    return _aruco.Dictionary_get(_aruco.DICT_APRILTAG_36h11)


def tag_image(tag_id: int, side_px: int) -> np.ndarray:
    """The black square of a tag (white border not included), as printed by make_tags."""
    dictionary = _dictionary()
    if hasattr(_aruco, 'generateImageMarker'):
        return _aruco.generateImageMarker(dictionary, tag_id, side_px)
    return _aruco.drawMarker(dictionary, tag_id, side_px)


class TagDetector:
    def __init__(self, backend: str = 'auto', quad_decimate: float = 2.0, threads: int = 2,
                 min_margin: float = 50.0, max_hamming: int = 0):
        self.min_margin = min_margin
        self.max_hamming = max_hamming
        self.backend = backend
        self._pupil = None
        self._opencv = None
        if backend in ('auto', 'pupil'):
            # Two packagings of the same AprilTag library. dt-apriltags has a ready-made
            # arm64 build, which is what the Jetson uses; pupil-apriltags would not build there.
            Detector = None
            for module in ('pupil_apriltags', 'dt_apriltags'):
                try:
                    Detector = __import__(module).Detector
                    break
                except ImportError:
                    continue
            if Detector is not None:
                self._pupil = Detector(families='tag36h11', nthreads=threads,
                                       quad_decimate=quad_decimate, refine_edges=1)
                self.backend = 'pupil'
            elif backend == 'pupil':
                raise ImportError('neither pupil_apriltags nor dt_apriltags is installed')
        if self._pupil is None:
            self.backend = 'opencv'
            if hasattr(_aruco, 'ArucoDetector'):
                parameters = _aruco.DetectorParameters()
                parameters.cornerRefinementMethod = _aruco.CORNER_REFINE_SUBPIX
                self._opencv = _aruco.ArucoDetector(_dictionary(), parameters)
            else:  # OpenCV before 4.7
                self._legacy_parameters = _aruco.DetectorParameters_create()
                self._legacy_parameters.cornerRefinementMethod = _aruco.CORNER_REFINE_SUBPIX
        self._order = self._find_corner_order()

    # ---- raw detection: (id, corners in the backend's own order, margin, hamming) ----
    def _raw(self, gray: np.ndarray):
        if self._pupil is not None:
            return [(d.tag_id, np.asarray(d.corners, dtype=float), float(d.decision_margin), int(d.hamming))
                    for d in self._pupil.detect(gray)]
        if self._opencv is not None:
            corners, ids, _ = self._opencv.detectMarkers(gray)
        else:
            corners, ids, _ = _aruco.detectMarkers(gray, _dictionary(), parameters=self._legacy_parameters)
        if ids is None:
            return []
        # OpenCV reports no decision margin; its own bit-error check has already passed.
        return [(int(i), np.asarray(c, dtype=float).reshape(4, 2), 100.0, 0)
                for i, c in zip(ids.flatten(), corners)]

    def _find_corner_order(self) -> List[int]:
        """Index of the backend's corner for each of top-left, top-right, bottom-right, bottom-left."""
        test_id, side, margin = 7, 240, 120
        canvas = np.full((side + 2 * margin, side + 2 * margin), 255, dtype=np.uint8)
        canvas[margin:margin + side, margin:margin + side] = tag_image(test_id, side)
        found = [d for d in self._raw(canvas) if d[0] == test_id]
        if not found:
            raise RuntimeError('Tag detector self-check failed: the %s backend does not detect a tag '
                               'drawn by the tag generator.' % self.backend)
        corners = found[0][1]
        expected = np.array([[margin, margin], [margin + side, margin],
                             [margin + side, margin + side], [margin, margin + side]], dtype=float)
        order = [int(np.argmin(np.linalg.norm(corners - target, axis=1))) for target in expected]
        if sorted(order) != [0, 1, 2, 3]:
            raise RuntimeError('Tag detector self-check failed: corners %s' % corners.tolist())
        return order

    def detect(self, gray: np.ndarray) -> List[Detection]:
        detections = []
        for tag_id, corners, margin, hamming in self._raw(gray):
            if hamming > self.max_hamming or margin < self.min_margin:
                continue
            detections.append(Detection(tag_id, corners[self._order], margin, hamming))
        return detections
