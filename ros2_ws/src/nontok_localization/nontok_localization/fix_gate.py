"""Decides what to do with each tag fix, and tracks localisation health. Pure Python."""
import math
from typing import Optional

from .geometry import Pose2D, wrap

ACCEPT, REJECT, RELOCALISE = 'accept', 'reject', 'relocalise'


class FixGate:
    """Consistency rule from PLAN.md section 5.4.

    A fix far from the current estimate is normally a bad detection and is rejected. But if
    several consecutive fixes agree with each other and not with the estimate, the estimate
    is the one that is wrong (the robot was carried, or drifted): the filter is reset.
    """

    def __init__(self, max_jump_m: float = 0.5, max_jump_rad: float = math.radians(20.0),
                 agree_m: float = 0.2, agree_rad: float = math.radians(10.0), needed: int = 3):
        self.max_jump_m = max_jump_m
        self.max_jump_rad = max_jump_rad
        self.agree_m = agree_m
        self.agree_rad = agree_rad
        self.needed = needed
        self._outlier: Optional[Pose2D] = None
        self._count = 0

    def check(self, fix: Pose2D, estimate: Optional[Pose2D], yaw_trusted: bool) -> str:
        if estimate is None:
            return RELOCALISE if yaw_trusted else REJECT
        jump = math.hypot(fix[0] - estimate[0], fix[1] - estimate[1])
        turn = abs(wrap(fix[2] - estimate[2])) if yaw_trusted else 0.0
        if jump <= self.max_jump_m and turn <= self.max_jump_rad:
            self._outlier, self._count = None, 0
            return ACCEPT
        if not yaw_trusted:
            return REJECT  # a reset needs a full pose
        if (self._outlier is not None
                and math.hypot(fix[0] - self._outlier[0], fix[1] - self._outlier[1]) <= self.agree_m
                and abs(wrap(fix[2] - self._outlier[2])) <= self.agree_rad):
            self._count += 1
        else:
            self._count = 1
        self._outlier = fix
        if self._count >= self.needed:
            self._outlier, self._count = None, 0
            return RELOCALISE
        return REJECT


class HealthTracker:
    """Healthy = a fix within max_age_s, or less than max_travel_m driven since the last fix."""

    def __init__(self, max_age_s: float = 10.0, max_travel_m: float = 3.0):
        self.max_age_s = max_age_s
        self.max_travel_m = max_travel_m
        self._fix_time: Optional[float] = None
        self._travel = 0.0
        self._last_xy = None

    def on_fix(self, now: float) -> None:
        self._fix_time = now
        self._travel = 0.0

    def on_odometry(self, x: float, y: float) -> None:
        if self._last_xy is not None:
            step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if step < 0.5:  # ignore filter resets
                self._travel += step
        self._last_xy = (x, y)

    def seconds_since_fix(self, now: float) -> float:
        return float('inf') if self._fix_time is None else now - self._fix_time

    def metres_since_fix(self) -> float:
        return float('inf') if self._fix_time is None else self._travel

    def healthy(self, now: float) -> bool:
        if self._fix_time is None:
            return False
        return now - self._fix_time <= self.max_age_s or self._travel <= self.max_travel_m
