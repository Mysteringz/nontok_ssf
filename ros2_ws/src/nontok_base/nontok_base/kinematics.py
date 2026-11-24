"""Mecanum wheel kinematics (PLAN.md section 5.4). Pure Python.

Wheel order: front-left, front-right, rear-left, rear-right.
Body frame: x forward, y left, yaw counter-clockwise.
"""
import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass
class MecanumParams:
    wheel_diameter_m: float = 0.080
    counts_per_rev: float = 660.0
    rotation_arm_m: float = 0.1925  # half wheelbase + half track (165/2 + 220/2 mm, 23 Nov)
    scale_x: float = 1.0            # true distance / encoder distance, forward
    scale_y: float = 1.0            # same, sideways (rollers slip more)

    @property
    def metres_per_count(self) -> float:
        return math.pi * self.wheel_diameter_m / self.counts_per_rev


def body_to_wheels(vx: float, vy: float, wz: float, arm_m: float) -> Tuple[float, float, float, float]:
    """Body velocity -> wheel rim speeds (same length unit as vx)."""
    w = wz * arm_m
    return (vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w)


def wheels_to_body(wheels: Sequence[float], arm_m: float) -> Tuple[float, float, float]:
    """Wheel rim speeds (or distances) -> body vx, vy, wz (or dx, dy, dyaw)."""
    fl, fr, rl, rr = wheels
    return (
        (fl + fr + rl + rr) / 4.0,
        (-fl + fr + rl - rr) / 4.0,
        (-fl + fr - rl + rr) / (4.0 * arm_m),
    )


def count_delta(new: int, old: int) -> int:
    """Difference of two cumulative 32-bit counters, correct across a wrap."""
    return (new - old + 2 ** 31) % 2 ** 32 - 2 ** 31


def counts_to_body(d_counts: Sequence[int], params: MecanumParams) -> Tuple[float, float, float]:
    """Encoder count changes -> distance moved in the body frame (dx, dy in m, dyaw in rad).

    dyaw from the wheels is returned for diagnostics only: the gyro is the heading source.
    """
    distances = [c * params.metres_per_count for c in d_counts]
    dx, dy, dyaw = wheels_to_body(distances, params.rotation_arm_m)
    return dx * params.scale_x, dy * params.scale_y, dyaw
