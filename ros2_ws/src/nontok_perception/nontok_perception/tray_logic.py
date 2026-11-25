"""Is something in the tray, has it stopped moving, and where is it? (PLAN.md 7.3, 13.10)

Pure Python + OpenCV. The same item_crop() is used when collecting training images and when
classifying, so the classifier always sees items framed the same way.
"""
from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Sequence

import cv2
import numpy as np

EMPTY, OCCUPIED, MOVING = 0, 1, 2


@dataclass
class TrayConfig:
    # Tray floor in the image as fractions of width and height, corners in order.
    roi: Sequence[Sequence[float]] = ((0.10, 0.10), (0.90, 0.10), (0.90, 0.90), (0.10, 0.90))
    diff_threshold: int = 25          # grey levels: a pixel this different from the reference has changed
    occupied_on: float = 0.02         # share of tray pixels changed -> occupied
    occupied_off: float = 0.01        # ...and below this -> empty again (gap prevents flicker)
    motion_threshold: float = 0.005   # share of pixels changing between frames -> something is moving
    stable_s: float = 2.0
    crop_margin: float = 0.20
    min_crop_px: int = 96
    crop_size: int = 224


@dataclass
class TrayReading:
    state: int
    occupied_fraction: float
    stable: bool
    has_reference: bool


def roi_mask(shape, roi) -> np.ndarray:
    height, width = shape[:2]
    points = np.array([[int(round(x * width)), int(round(y * height))] for x, y in roi], dtype=np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [points], 255)
    return mask


def _grey(frame: np.ndarray) -> np.ndarray:
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    return cv2.GaussianBlur(grey, (5, 5), 0)


class TrayLogic:
    def __init__(self, cfg: Optional[TrayConfig] = None):
        self.cfg = cfg or TrayConfig()
        self.reference: Optional[np.ndarray] = None
        self._mask: Optional[np.ndarray] = None
        self._pixels = 1
        self._previous: Optional[np.ndarray] = None
        self._occupied = False
        self._still_since: Optional[float] = None
        self._changed: Optional[np.ndarray] = None

    def set_reference(self, frames: List[np.ndarray]) -> None:
        """The empty tray: the average of a few frames."""
        stack = np.stack([_grey(f).astype(np.float32) for f in frames])
        self.reference = stack.mean(axis=0).astype(np.uint8)
        self._mask = roi_mask(self.reference.shape, self.cfg.roi)
        self._pixels = max(1, int(np.count_nonzero(self._mask)))
        self._previous = None
        self._occupied = False
        self._still_since = None

    def _changed_mask(self, grey: np.ndarray) -> np.ndarray:
        # Remove an overall brightness change (camera auto-exposure, a light switched on).
        inside = self._mask > 0
        offset = float(np.median(grey[inside])) - float(np.median(self.reference[inside]))
        diff = np.abs(grey.astype(np.float32) - offset - self.reference.astype(np.float32))
        changed = ((diff > self.cfg.diff_threshold) & inside).astype(np.uint8) * 255
        return cv2.morphologyEx(changed, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # drop speckle

    def update(self, frame: np.ndarray, now: float) -> TrayReading:
        if self.reference is None:
            return TrayReading(EMPTY, 0.0, False, False)
        cfg = self.cfg
        grey = _grey(frame)
        self._changed = self._changed_mask(grey)
        fraction = np.count_nonzero(self._changed) / self._pixels
        if fraction >= cfg.occupied_on:
            self._occupied = True
        elif fraction <= cfg.occupied_off:
            self._occupied = False

        moving = False
        if self._previous is not None:
            step = cv2.absdiff(grey, self._previous)
            moved = np.count_nonzero((step > cfg.diff_threshold) & (self._mask > 0)) / self._pixels
            moving = moved > cfg.motion_threshold
        self._previous = grey
        if moving or self._still_since is None:
            self._still_since = now
        stable = (now - self._still_since) >= cfg.stable_s
        state = MOVING if moving else (OCCUPIED if self._occupied else EMPTY)
        return TrayReading(state, float(fraction), stable, True)

    def item_box(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """(x, y, w, h) around everything that differs from the empty tray, or None."""
        if self.reference is None:
            return None
        points = cv2.findNonZero(self._changed_mask(_grey(frame)))
        return None if points is None else tuple(int(v) for v in cv2.boundingRect(points))

    def item_crop(self, frame: np.ndarray, box: Optional[Tuple[int, int, int, int]] = None) -> Optional[np.ndarray]:
        """A square colour image of the item, or None if nothing differs from the empty tray."""
        cfg = self.cfg
        box = box or self.item_box(frame)
        if box is None:
            return None
        x, y, w, h = box
        side = int(max(w, h) * (1.0 + 2.0 * cfg.crop_margin))
        side = max(side, cfg.min_crop_px)
        cx, cy = x + w / 2.0, y + h / 2.0
        x0, y0 = int(round(cx - side / 2.0)), int(round(cy - side / 2.0))
        height, width = frame.shape[:2]
        # Pad instead of clamping, so the item stays centred even at the tray's edge.
        pad = max(0, -x0, -y0, x0 + side - width, y0 + side - height)
        if pad > 0:
            frame = cv2.copyMakeBorder(frame, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
            x0, y0 = x0 + pad, y0 + pad
        crop = frame[y0:y0 + side, x0:x0 + side]
        return cv2.resize(crop, (cfg.crop_size, cfg.crop_size), interpolation=cv2.INTER_AREA)
